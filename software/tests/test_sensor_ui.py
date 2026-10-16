"""Check the protection form without connecting to a controller."""
from pathlib import Path
import shutil
import subprocess
import unittest


class SensorUiTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('node'), 'Node is required for UI checks')
    def test_stream_grid_labels_and_scaled_traces(self):
        path = Path(__file__).resolve().parents[1] / 'host/robot_mapper_klipper.py'
        subprocess.run(['node', '-e', r'''
const fs=require('fs'),assert=require('assert');
const source=fs.readFileSync(process.argv[1],'utf8');
const start=source.indexOf('function drawSensorStream(){');
const end=source.indexOf('function startSensorStream()',start);
let labels=[],points=[],clips=0,dashes=[];
const ctx={clearRect(){},fillText(text,x,y){labels.push([text,x,y])},
  beginPath(){},moveTo(x,y){points.push([x,y])},lineTo(x,y){points.push([x,y])},
  stroke(){},save(){},rect(){},clip(){clips++},restore(){},setLineDash(v){dashes.push(v)}};
const canvas={width:900,height:220,getContext(){return ctx}};
const draw=new Function('$','sensorGraphSamples',
  'let sensorGraphRaf=0;'+source.slice(start,end)+';drawSensorStream();');
for(const samples of [[],[{t:10,accel_g:[0,1,0]}],
    [{t:0,accel_g:[0,1,0]},{t:10,accel_g:[3.1,-2,0]}]]){
  labels=[];points=[];draw(()=>canvas,samples);
  const texts=labels.map(l=>l[0]);
  assert(texts.includes('−10 s'));assert(texts.includes('−5 s'));assert(texts.includes('now'));
  assert(texts.includes('Mag'));
  assert(texts.some(t=>/^0(?:\.0)? g$/.test(t)));
  assert(texts.some(t=>t.startsWith('-')&&t.endsWith(' g')));
  assert(points.every(p=>p.every(Number.isFinite)));
  assert(labels.every(([,x,y])=>x>=0&&x<=900&&y>=0&&y<=220));
}
assert.equal(clips,2);
assert.deepEqual(dashes.slice(-4),[[],[],[],[6,5]]);
// Magnitude, not the largest individual component, must drive autoscaling.
labels=[];points=[];draw(()=>canvas,[{t:10,accel_g:[3,4,0]}]);
assert(labels.some(([text])=>text==='6 g'));
assert(points.some(([x,y])=>x===882&&Math.abs(y-(30+160*(1-5/6)/2))<1e-6));
''', str(path)], check=True)

    @unittest.skipUnless(shutil.which('node'), 'Node is required for UI checks')
    def test_protection_action_and_autosave(self):
        path = Path(__file__).resolve().parents[1] / 'host/robot_mapper_klipper.py'
        subprocess.run(['node', '-e', r'''
const fs=require('fs'),assert=require('assert');
const source=fs.readFileSync(process.argv[1],'utf8');
const start=source.indexOf('const sensorProtectionInputs=');
const end=source.indexOf("$('sensorAck').addEventListener",start);
const elements={};
const $=id=>elements[id]??={value:'',checked:false,disabled:false,dataset:{},
  textContent:'',valid:true,checkValidity(){return this.valid},addEventListener(){}};
let calls=[],fail=false,rendered;
const api=async(path,body)=>{
  if(path==='/sensor')return {monitor:{enabled:true,kill_enabled:true}};
  calls.push(body);
  if(fail)throw Error('Offline');
  return {monitor:body};
};
const renderSensor=value=>{rendered=value};
const save=new Function('$','api','renderSensor',source.slice(start,end)+';return saveTiltProtection;')($,api,renderSensor);
$('sensorTiltLimit').value='15';$('sensorPersistence').value='.75';$('sensorTiltFilter').value='.25';
$('sensorHardKill').value='50';
(async()=>{
  for(const [enabled,action,kill,correct] of [
    [true,'kill',true,false],[true,'correct',false,true],
    [false,'correct',false,false],[false,'kill',false,false]]){
    $('sensorProtection').dataset.dirty='1';
    $('sensorMonitorEnabled').checked=enabled;$('sensorTiltAction').value=action;
    await save();
    const payload=calls.at(-1);
    assert.equal(payload.enabled,enabled);assert.equal(payload.kill_enabled,kill);
    assert.equal(payload.correct_enabled,correct);assert.equal(payload.allowed_deg,15);
    assert.equal(payload.persistence_s,.75);assert.equal(payload.filter_s,.25);
    assert.equal(payload.hard_kill_deg,50);
    assert.equal($('sensorProtection').dataset.dirty,undefined);
    assert.equal($('sensorMonitorEnabled').disabled,false);
  }
  const count=calls.length;$('sensorTiltLimit').valid=false;
  await save();assert.equal(calls.length,count);
  $('sensorTiltLimit').valid=true;fail=true;
  await save();assert.match($('sensorSaveStatus').textContent,/Not saved: Offline/);
  assert.equal(rendered.monitor.kill_enabled,true);
  assert.equal($('sensorMonitorEnabled').disabled,false);
})().catch(error=>{console.error(error);process.exitCode=1});
''', str(path)], check=True)
