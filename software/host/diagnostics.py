import collections, json, math, statistics, threading, time, urllib.request

VALID_WINDOWS=(60,180,300,600,1800,3600)

class Diagnostics:
    def __init__(self, moonraker="http://127.0.0.1:7125", max_samples=20000):
        self.moonraker=moonraker
        self.max_samples=max_samples
        self.mode="BASIC"
        self.window_s=300
        self.lock=threading.Lock()
        self.series=collections.defaultdict(lambda: collections.deque(maxlen=max_samples))
        self.events=collections.deque(maxlen=1000)
        self.started=time.time()
        self._stop=threading.Event()
        self._thread=None
        self._last_motion_sample=None

    def set_mode(self, mode):
        mode=str(mode).upper()
        if mode not in ("OFF","BASIC","TRACE"):
            raise ValueError("mode must be OFF, BASIC, or TRACE")
        with self.lock:
            self.mode=mode
        self.event("debug_mode",{"mode":mode})
        return mode

    def set_window(self, seconds):
        seconds=int(seconds)
        if seconds not in VALID_WINDOWS:
            raise ValueError("window must be one of "+",".join(map(str,VALID_WINDOWS)))
        with self.lock:
            self.window_s=seconds
        return seconds

    def reset(self):
        with self.lock:
            self.series.clear()
            self.events.clear()
            self.started=time.time()

    def event(self, name, fields=None):
        with self.lock:
            if self.mode=="OFF": return
            self.events.append({"t":time.time(),"name":name,"fields":fields or {}})

    def sample(self, name, value, fields=None):
        try:v=float(value)
        except Exception:return
        now=time.time()
        with self.lock:
            if self.mode=="OFF": return
            self.series[name].append((now,v))
            if self.mode=="TRACE" and fields is not None:
                self.events.append({"t":now,"name":name,"value":v,"fields":fields})

    def latency(self, name, seconds, fields=None):
        self.sample("latency."+name,float(seconds)*1000.0,fields)

    @staticmethod
    def _stats(vals):
        if not vals:return {"count":0}
        s=sorted(vals)
        def pct(p):
            if len(s)==1:return s[0]
            k=(len(s)-1)*p;lo=int(math.floor(k));hi=int(math.ceil(k))
            if lo==hi:return s[lo]
            return s[lo]+(s[hi]-s[lo])*(k-lo)
        return {"count":len(s),"current":vals[-1],"mean":statistics.fmean(s),
                "median":statistics.median(s),"p90":pct(.90),"p95":pct(.95),
                "p99":pct(.99),"max":max(s),"min":min(s),
                "stddev":statistics.pstdev(s) if len(s)>1 else 0.0}

    @staticmethod
    def _downsample(pairs,max_points=140):
        n=len(pairs)
        if n<=max_points:return pairs
        step=n/max_points
        return [pairs[min(n-1,int(i*step))] for i in range(max_points)]

    def snapshot(self):
        now=time.time()
        with self.lock:
            mode=self.mode;window=self.window_s
            raw={k:list(v) for k,v in self.series.items()}
            events=list(self.events)[-120:]
        cutoff=now-window
        stats={};compact={}
        for k,pairs in raw.items():
            recent=[p for p in pairs if p[0]>=cutoff]
            stats[k]=self._stats([v for _,v in recent])
            compact[k]=self._downsample(recent,140)
        return {"mode":mode,"window_s":window,"uptime_s":now-self.started,
                "stats":stats,"series":compact,"events":events}

    def _fetch(self,path):
        t=time.perf_counter()
        with urllib.request.urlopen(self.moonraker+path,timeout=1.5) as r:
            obj=json.loads(r.read().decode())
        self.latency("diagnostic_poll",time.perf_counter()-t)
        return obj

    def _poll_once(self):
        q=self._fetch("/printer/objects/query?arm_robot&mcu&system_stats&motion_report&toolhead")
        st=q.get("result",{}).get("status",{})
        ar=st.get("arm_robot",{});mcu=st.get("mcu",{}).get("last_stats",{})
        sys=st.get("system_stats",{});motion=st.get("motion_report",{})
        toolhead=st.get("toolhead",{})
        for n,key,mult in (
            ("mcu.srtt_ms","srtt",1000.0),("mcu.rttvar_ms","rttvar",1000.0),
            ("mcu.rto_ms","rto",1000.0),("mcu.retransmit_bytes","bytes_retransmit",1.0),
            ("mcu.invalid_bytes","bytes_invalid",1.0),("mcu.awake_ms","mcu_awake",1000.0),
            ("mcu.task_avg_us","mcu_task_avg",1e6),("mcu.task_stddev_us","mcu_task_stddev",1e6)):
            if key in mcu:self.sample(n,float(mcu[key])*mult)
        if "sysload" in sys:self.sample("system.load",sys["sysload"])
        if "memavail" in sys:self.sample("system.mem_available_kb",sys["memavail"])
        if "live_velocity" in motion:self.sample("motion.xyz_live_velocity",motion["live_velocity"])
        if "print_time" in toolhead and "estimated_print_time" in toolhead:
            self.sample("klipper.buffer_time_s",max(
                0.0,float(toolhead["print_time"])-float(toolhead["estimated_print_time"])))
        if "stalls" in toolhead:
            self.sample("klipper.print_stalls",toolhead["stalls"])
        pos=motion.get("live_position")
        now=time.time()
        if isinstance(pos,list) and len(pos)>=8:
            cur=(now,float(pos[4]),float(pos[5]),float(pos[6]),float(pos[7]))
            if self._last_motion_sample is not None:
                prev=self._last_motion_sample
                dt=max(1e-6,cur[0]-prev[0])
                for i,n in enumerate(("A","B","C","D"),start=1):
                    self.sample("motion."+n+"_speed",abs(cur[i]-prev[i])/dt)
            self._last_motion_sample=cur
        for key in ("queue_ahead_s","dance_cycle_time","dance_sample_dt_s","dance_sample","dance_cycle"):
            if key in ar:self.sample("arm."+key,ar[key])
        self.sample("arm.dance_active",1.0 if ar.get("dance_active") else 0.0)

    def _loop(self):
        next_t=time.monotonic()
        while not self._stop.is_set():
            now=time.monotonic()
            self.sample("diagnostics.loop_jitter_ms",max(0.0,now-next_t)*1000.0)
            if self.mode!="OFF":
                try:self._poll_once()
                except Exception as exc:self.event("diagnostic_poll_error",{"error":str(exc)})
            next_t=max(next_t+1.0,time.monotonic()+0.05)
            self._stop.wait(max(0.02,next_t-time.monotonic()))

    def start(self):
        if self._thread and self._thread.is_alive():return
        self._thread=threading.Thread(target=self._loop,name="arm-diagnostics",daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
