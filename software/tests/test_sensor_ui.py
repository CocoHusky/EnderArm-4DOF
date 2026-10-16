"""Check the protection form without connecting to a controller."""
from pathlib import Path
import shutil
import subprocess
import unittest


class SensorUiTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('node'), 'Node is required for UI checks')
    def test_protection_action_and_autosave(self):
        path = Path(__file__).resolve().parents[1] / 'host/robot_mapper_klipper.py'
        subprocess.run(['node', '-e', r'''
const fs=require('fs'),assert=require('assert');
const source=fs.readFileSync(process.argv[1],'utf8');
const start=source.indexOf('const sensorProtectionInputs=');
const end=source.indexOf("$('sensorRecoverySave').addEventListener",start);
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
