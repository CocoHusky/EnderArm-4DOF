#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "board.h"
#include "accel.h"
#include "fsl_debug_console.h"
#include "fsl_os_abstraction.h"
#include "fsl_port_hal.h"

#define FW_VERSION "0.3.0"
#define STREAM_PERIOD_MS 50U
#define ACCEL_REG_WHO_AM_I 0x0dU
#define ACCEL_REG_OUT_X_MSB 0x01U
#define MMA8451_WHO_AM_I 0x1aU

#define LED_RED_PIN 6U
#define LED_GREEN_PIN 7U
#define LED_BLUE_PIN 10U
#define LED_MASK ((1UL << LED_RED_PIN) | (1UL << LED_GREEN_PIN) | (1UL << LED_BLUE_PIN))

#define TOUCH_LEFT_PIN 13U
#define TOUCH_RIGHT_PIN 12U
#define TOUCH_MAX_COUNT 1023U
#define TOUCH_REPEATS 8U
#define TOUCH_BASELINE_SAMPLES 48U

void hardware_init(void);

static accel_dev_t g_accel_dev;
static accel_dev_interface_t g_accel_if;
static accel_i2c_interface_t g_i2c_if;
static bool g_accel_ok = false;
static uint8_t g_whoami = 0U;

static uint32_t g_seq = 0U;
static uint16_t g_last_osa_ms = 0U;
static uint32_t g_osa_epoch = 0U;

static uint16_t g_touch_left_base = 0U;
static uint16_t g_touch_right_base = 0U;
static bool g_slider_ok = false;
static bool g_touch_active = false;
static uint16_t g_touch_start_pos = 500U;
static uint16_t g_touch_last_pos = 500U;
static uint32_t g_touch_start_ms = 0U;
static const char *g_swipe = "none";
static const char *g_gesture = "none";
static uint32_t g_gesture_id = 0U;
static uint16_t g_touch_min_pos = 500U, g_touch_max_pos = 500U;
static uint32_t g_touch_pos_sum = 0U, g_touch_samples = 0U;
static const char *g_led = "off";
static const char *g_mode = "manual";
static const char *g_orientation = "unknown";
static const char *g_motion = "none";
static const char *g_tap = "none";
static uint32_t g_last_tap_ms = 0U;
static uint32_t g_last_motion_ms = 0U;
static int32_t g_last_x = 0, g_last_y = 0, g_last_z = 0;
static uint8_t g_tap_count = 0U;

static char g_cmd[48];
static uint8_t g_cmd_len = 0U;
/* Receive during I2C reads and telemetry printing. Polling UART once per
 * millisecond loses bytes from a normal 115200-baud host command. */
static volatile uint8_t g_rx_head = 0U, g_rx_tail = 0U;
static volatile bool g_rx_overflow = false;
static char g_rx[128];
static bool g_cmd_discard = false;

void UART0_IRQHandler(void)
{
    uint8_t status = UART0->S1;
    if (status & (UART0_S1_RDRF_MASK | UART0_S1_OR_MASK))
    {
        char c = (char)UART0->D;
        uint8_t next = (uint8_t)((g_rx_head + 1U) & 127U);
        if (status & UART0_S1_OR_MASK) g_rx_overflow = true;
        if (next == g_rx_tail) g_rx_overflow = true;
        else { g_rx[g_rx_head] = c; g_rx_head = next; }
    }
}

static uint32_t monotonic_ms(void)
{
    uint16_t now = (uint16_t)(OSA_TimeGetMsec() & 0xffffU);
    if (now < g_last_osa_ms)
    {
        g_osa_epoch += 0x10000UL;
    }
    g_last_osa_ms = now;
    return g_osa_epoch + now;
}

static void led_apply(const char *name)
{
    uint32_t on = 0U;
    if (!strcmp(name, "red")) on = (1UL << LED_RED_PIN);
    else if (!strcmp(name, "green")) on = (1UL << LED_GREEN_PIN);
    else if (!strcmp(name, "blue")) on = (1UL << LED_BLUE_PIN);
    else if (!strcmp(name, "yellow")) on = (1UL << LED_RED_PIN) | (1UL << LED_GREEN_PIN);
    else if (!strcmp(name, "cyan")) on = (1UL << LED_GREEN_PIN) | (1UL << LED_BLUE_PIN);
    else if (!strcmp(name, "magenta")) on = (1UL << LED_RED_PIN) | (1UL << LED_BLUE_PIN);
    else if (!strcmp(name, "white")) on = LED_MASK;
    else name = "off";

    GPIOB->PSOR = LED_MASK;
    if (on) GPIOB->PCOR = on;
    g_led = name;
}

static void leds_init(void)
{
    PORT_HAL_SetMuxMode(PORTB, LED_RED_PIN, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB, LED_GREEN_PIN, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB, LED_BLUE_PIN, kPortMuxAsGpio);
    GPIOB->PDDR |= LED_MASK;
    led_apply("off");
}

static uint16_t touch_measure(GPIO_Type *gpio, PORT_Type *port, uint32_t pin)
{
    uint32_t mask = 1UL << pin;
    uint32_t total = 0U;
    for (uint32_t sample = 0; sample < TOUCH_REPEATS; sample++)
    {
        port->PCR[pin] = PORT_PCR_MUX(1U);
        gpio->PCOR = mask;
        gpio->PDDR |= mask;
        for (volatile uint32_t delay = 0; delay < 24U; delay++) { __NOP(); }

        /* Board already has 510k external charge resistors. An internal
         * pull-up shunts these, collapsing touch charge times to ~2 counts. */
        port->PCR[pin] = PORT_PCR_MUX(1U);
        gpio->PDDR &= ~mask;

        uint32_t count = 0U;
        while (((gpio->PDIR & mask) == 0U) && count < TOUCH_MAX_COUNT)
        {
            count++;
        }
        total += count;
    }
    return (uint16_t)total;
}

static void slider_init(void)
{
    uint32_t left = 0U, right = 0U;
    for (uint32_t i = 0; i < TOUCH_BASELINE_SAMPLES; i++)
    {
        left += touch_measure(GPIOA, PORTA, TOUCH_LEFT_PIN);
        right += touch_measure(GPIOB, PORTB, TOUCH_RIGHT_PIN);
        OSA_TimeDelay(2U);
    }
    g_touch_left_base = (uint16_t)(left / TOUCH_BASELINE_SAMPLES);
    g_touch_right_base = (uint16_t)(right / TOUCH_BASELINE_SAMPLES);
    g_slider_ok = (g_touch_left_base < TOUCH_MAX_COUNT * TOUCH_REPEATS && g_touch_right_base < TOUCH_MAX_COUNT * TOUCH_REPEATS);
}

typedef struct
{
    uint16_t raw_left;
    uint16_t raw_right;
    uint16_t position;
    bool active;
} slider_sample_t;

static slider_sample_t slider_read(uint32_t now_ms)
{
    slider_sample_t s;
    s.raw_left = touch_measure(GPIOA, PORTA, TOUCH_LEFT_PIN);
    s.raw_right = touch_measure(GPIOB, PORTB, TOUCH_RIGHT_PIN);

    int32_t dl = (int32_t)s.raw_left - (int32_t)g_touch_left_base;
    int32_t dr = (int32_t)s.raw_right - (int32_t)g_touch_right_base;
    if (dl < 0) dl = 0;
    if (dr < 0) dr = 0;
    uint32_t delta = (uint32_t)(dl + dr);
    uint32_t baseline = ((uint32_t)g_touch_left_base + (uint32_t)g_touch_right_base) / 2U;
    uint32_t threshold = baseline / 5U;
    if (threshold < 3U) threshold = 3U;

    s.active = delta > threshold;
    if (s.active)
        s.position = (uint16_t)(((uint32_t)dr * 1000U) / delta);
    else
        s.position = g_touch_last_pos;

    g_swipe = "none";
    if (s.active)
    {
        g_touch_last_pos = s.position;
        if (!g_touch_active)
        {
            g_touch_active = true;
            g_touch_start_pos = s.position;
            g_touch_start_ms = now_ms;
            g_touch_min_pos = s.position;
            g_touch_max_pos = s.position;
            g_touch_pos_sum = 0U;
            g_touch_samples = 0U;
        }
        if (s.position < g_touch_min_pos) g_touch_min_pos = s.position;
        if (s.position > g_touch_max_pos) g_touch_max_pos = s.position;
        g_touch_pos_sum += s.position;
        g_touch_samples++;
    }
    else if (g_touch_active)
    {
        uint32_t duration = now_ms - g_touch_start_ms;
        int32_t travel = (int32_t)g_touch_last_pos - (int32_t)g_touch_start_pos;
        if (duration >= 40U && duration <= 1200U)
        {
            if (travel >= 300) g_swipe = "right";
            else if (travel <= -300) g_swipe = "left";
            if (strcmp(g_swipe, "none"))
            {
                g_gesture = !strcmp(g_swipe, "left") ? "swipe_left" : "swipe_right";
                g_gesture_id++;
            }
            else if (g_touch_max_pos - g_touch_min_pos <= 150U && g_touch_samples)
            {
                uint32_t pos = g_touch_pos_sum / g_touch_samples;
                g_gesture = pos < 333U ? "press_1" : (pos > 666U ? "press_3" : "press_2");
                g_gesture_id++;
            }
        }
        g_touch_active = false;
    }

    if (!s.active)
    {
        g_touch_left_base = (uint16_t)(((uint32_t)g_touch_left_base * 63U + s.raw_left) / 64U);
        g_touch_right_base = (uint16_t)(((uint32_t)g_touch_right_base * 63U + s.raw_right) / 64U);
    }
    g_slider_ok = (s.raw_left < TOUCH_MAX_COUNT * TOUCH_REPEATS && s.raw_right < TOUCH_MAX_COUNT * TOUCH_REPEATS);
    return s;
}

static bool accel_read_mg(int32_t *x, int32_t *y, int32_t *z)
{
    uint8_t reg = ACCEL_REG_OUT_X_MSB;
    uint8_t data[6] = {0};
    i2c_status_t status = I2C_DRV_MasterReceiveDataBlocking(
        g_accel_dev.bus, &g_accel_dev.slave, &reg, 1U, data, 6U, 50U);
    if (status != kStatus_I2C_Success) return false;

    int16_t rx = (int16_t)((uint16_t)data[0] << 8 | data[1]);
    int16_t ry = (int16_t)((uint16_t)data[2] << 8 | data[3]);
    int16_t rz = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    rx >>= 2;
    ry >>= 2;
    rz >>= 2;

    /* Freescale's KL02 demo configures the MMA8451Q for +/-4 g: 2048 counts/g. */
    *x = ((int32_t)rx * 1000L) / 2048L;
    *y = ((int32_t)ry * 1000L) / 2048L;
    *z = ((int32_t)rz * 1000L) / 2048L;
    return true;
}

static void accel_init_board(void)
{
    g_i2c_if.i2c_init = I2C_DRV_MasterInit;
    g_i2c_if.i2c_read = I2C_DRV_MasterReceiveDataBlocking;
    g_i2c_if.i2c_write = I2C_DRV_MasterSendDataBlocking;

    g_accel_dev.i2c = &g_i2c_if;
    g_accel_dev.accel = &g_accel_if;
    g_accel_dev.slave.baudRate_kbps = BOARD_ACCEL_BAUDRATE;
    g_accel_dev.slave.address = BOARD_ACCEL_ADDR;
    g_accel_dev.bus = BOARD_ACCEL_I2C_INSTANCE;

    g_accel_ok = (accel_init(&g_accel_dev) == 0);
    g_whoami = accel_read_reg(&g_accel_dev, ACCEL_REG_WHO_AM_I);
    if (g_whoami != MMA8451_WHO_AM_I) g_accel_ok = false;
}

static void command_apply(char *cmd)
{
    if (!strncmp(cmd, "MODE ", 5))
    {
        char *m = cmd + 5;
        for (char *p=m; *p; p++) if (*p >= 'A' && *p <= 'Z') *p=(char)(*p-'A'+'a');
        if (!strcmp(m,"manual") || !strcmp(m,"acceleration") ||
            !strcmp(m,"orientation") || !strcmp(m,"slider") ||
            !strcmp(m,"tap") || !strcmp(m,"motion"))
        {
            if (!strcmp(m,"manual")) g_mode="manual";
            else if (!strcmp(m,"acceleration")) g_mode="acceleration";
            else if (!strcmp(m,"orientation")) g_mode="orientation";
            else if (!strcmp(m,"slider")) g_mode="slider";
            else if (!strcmp(m,"tap")) g_mode="tap";
            else g_mode="motion";
            led_apply("off");
        }
        return;
    }
    if (!strncmp(cmd, "LED ", 4))
    {
        char *name = cmd + 4;
        for (char *p = name; *p; p++)
            if (*p >= 'A' && *p <= 'Z') *p = (char)(*p - 'A' + 'a');
        g_mode="manual";
        led_apply(name);
    }
}

static void command_poll(void)
{
    if (g_rx_overflow)
    {
        g_rx_overflow = false;
        g_cmd_len = 0U;
        g_cmd_discard = true;
    }
    while (g_rx_tail != g_rx_head)
    {
        char c = g_rx[g_rx_tail];
        g_rx_tail = (uint8_t)((g_rx_tail + 1U) & 127U);
        if (c == '\r' || c == '\n')
        {
            if (g_cmd_discard) { g_cmd_discard = false; g_cmd_len = 0U; }
            else if (g_cmd_len)
            {
                g_cmd[g_cmd_len] = '\0';
                command_apply(g_cmd);
                g_cmd_len = 0U;
            }
        }
        else if (g_cmd_discard) { continue; }
        else if (g_cmd_len + 1U < sizeof(g_cmd))
        {
            g_cmd[g_cmd_len++] = c;
        }
        else
        {
            g_cmd_len = 0U;
            g_cmd_discard = true;
        }
    }
}

/* Teaching/LED feedback mode logic. Tap/impact is firmware-side acceleration
 * transient detection, not yet the MMA8451Q hardware pulse interrupt. */
static void update_led_feedback(int32_t x,int32_t y,int32_t z,
                                slider_sample_t s,uint32_t now)
{
    int32_t ax=x<0?-x:x, ay=y<0?-y:y, az=z<0?-z:z;
    int32_t dx=x-g_last_x,dy=y-g_last_y,dz=z-g_last_z;
    int32_t delta=(dx<0?-dx:dx)+(dy<0?-dy:dy)+(dz<0?-dz:dz);
    g_last_x=x;g_last_y=y;g_last_z=z;
    g_tap="none";
    if (delta>1100 && now-g_last_tap_ms>100U)
    {
        g_tap=(now-g_last_tap_ms<450U)?"double":"single";
        g_last_tap_ms=now;
    }
    if (ax>=ay && ax>=az) g_orientation=x>=0?"x+":"x-";
    else if (ay>=az) g_orientation=y>=0?"y+":"y-";
    else g_orientation=z>=0?"z+":"z-";
    int32_t sum=ax+ay+az;
    if (sum<250) g_motion="freefall";
    else if(delta>300) { g_motion="moving";g_last_motion_ms=now; }
    else if(now-g_last_motion_ms>300U) g_motion="stationary";
    if (!strcmp(g_mode,"manual")) return;
    if (!strcmp(g_mode,"acceleration"))
        led_apply(ax>=ay && ax>=az?"red":(ay>=az?"green":"blue"));
    else if (!strcmp(g_mode,"orientation"))
    {
        if (!strcmp(g_orientation,"x+")) led_apply("red");
        else if (!strcmp(g_orientation,"x-")) led_apply("cyan");
        else if (!strcmp(g_orientation,"y+")) led_apply("green");
        else if (!strcmp(g_orientation,"y-")) led_apply("magenta");
        else if (!strcmp(g_orientation,"z+")) led_apply("blue");
        else led_apply("yellow");
    }
    else if (!strcmp(g_mode,"slider"))
        led_apply(!s.active?"off":(s.position<333U?"red":(s.position>666U?"blue":"green")));
    else if (!strcmp(g_mode,"tap"))
        led_apply(!strcmp(g_tap,"none")?"off":"white");
    else if (!strcmp(g_mode,"motion"))
        led_apply(!strcmp(g_motion,"freefall")?"magenta":(!strcmp(g_motion,"stationary")?"green":"yellow"));
}

int main(void)
{
    hardware_init();
    UART0->C2 |= UART0_C2_RIE_MASK;
    NVIC_EnableIRQ(UART0_IRQn);
    OSA_Init();
    leds_init();
    accel_init_board();
    slider_init();

    uint32_t next_ms = monotonic_ms();
    while (1)
    {
        command_poll();
        uint32_t now = monotonic_ms();
        if ((int32_t)(now - next_ms) < 0)
        {
            OSA_TimeDelay(1U);
            continue;
        }
        next_ms = now + STREAM_PERIOD_MS;

        int32_t ax = 0, ay = 0, az = 0;
        /* A transient I2C read failure is reported unhealthy but retried on
         * the next sample. Initial identity failure remains fail-closed. */
        bool read_ok = g_accel_ok && accel_read_mg(&ax, &ay, &az);
        slider_sample_t slider = slider_read(now);
        if (read_ok) update_led_feedback(ax,ay,az,slider,now);
        else if (strcmp(g_mode,"manual")) led_apply("off");

        PRINTF("{\"v\":1,\"device\":\"FRDM-KL02Z\",\"fw\":\"" FW_VERSION
               "\",\"seq\":%lu,\"ms\":%lu,\"accel_mg\":[%ld,%ld,%ld],"
               "\"slider\":{\"raw_left\":%u,\"raw_right\":%u,\"position\":%u,"
               "\"active\":%s,\"swipe\":\"%s\",\"gesture\":\"%s\",\"gesture_id\":%lu},\"led\":\"%s\","
               "\"mode\":\"%s\",\"events\":{\"tap\":\"%s\",\"motion\":\"%s\",\"orientation\":\"%s\"},"
               "\"health\":{\"accel\":%s,\"slider\":%s,\"whoami\":%u}}\r\n",
               (unsigned long)g_seq++, (unsigned long)now,
               (long)ax, (long)ay, (long)az,
               (unsigned)slider.raw_left, (unsigned)slider.raw_right,
               (unsigned)slider.position, slider.active ? "true" : "false",
               g_swipe, g_gesture, (unsigned long)g_gesture_id, g_led, g_mode, g_tap, g_motion, g_orientation,
               read_ok ? "true" : "false",
               g_slider_ok ? "true" : "false", (unsigned)g_whoami);
    }
}
