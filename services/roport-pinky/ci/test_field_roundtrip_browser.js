// Offline browser logic tests: fake DOM, fetch, timers. No robot, ROS or network.
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const script=fs.readFileSync(path.join(__dirname,'../scripts/field_roundtrip/index.html'),'utf8').match(/<script>([\s\S]*?)<\/script>/)[1];
const settle=()=>new Promise(resolve=>setImmediate(resolve));
const status={fresh:true,phase:'READY',start_enabled:true,delivery_enabled:true,restart_enabled:true,physically_stopped:true,mission_id:'test-mission',stop_confirmed:false};
function harness(){
  const elements={},requests=[],timers=new Map(),intervals=[],listeners={};let nextTimer=0;
  const document={hidden:false,getElementById(id){return elements[id]||(elements[id]={textContent:'',disabled:false})},addEventListener(name,fn){listeners[name]=fn}};
  const context=vm.createContext({document,crypto:{randomUUID:()=> '00000000-0000-4000-8000-000000000001'},AbortController,console,
    setTimeout(fn,delay){const id=++nextTimer;timers.set(id,{fn,delay});return id},clearTimeout(id){timers.delete(id)},setInterval(fn,delay){intervals.push({fn,delay})},
    fetch(url,options={}){return new Promise((resolve,reject)=>{
      const request={url,options,resolve(data,ok=true){resolve({ok,json:async()=>data})},reject};requests.push(request);
      options.signal?.addEventListener('abort',()=>reject(new Error('aborted')),{once:true});
    })}
  });
  vm.runInContext(script,context);
  function latest(url){return requests.filter(r=>r.url===url).at(-1)}
  async function ready(){latest('/heartbeat').resolve({success:true,control_epoch:0});latest('/status').resolve(status);await settle();assert.equal(elements.start.disabled,false)}
  function poll(){return vm.runInContext('poll()',context)}
  async function update(data){const pending=poll();latest('/status').resolve({...status,...data});await pending}
  function fire(delay){for(const [id,timer] of [...timers])if(timer.delay===delay){timers.delete(id);timer.fn()}}
  function visibility(hidden){document.hidden=hidden;listeners.visibilitychange()}
  return {elements,requests,latest,ready,poll,update,fire,visibility,intervals,context};
}
async function testBusyStartPriorityStopAndOldResponses(){
  const h=harness();await h.ready();
  const start=h.elements.start.onclick();const oldStatus=h.poll();
  assert.equal(h.elements.start.disabled,true);
  const stop=h.elements.stop.onclick();
  assert.equal(h.requests.filter(r=>r.url==='/stop').length,1,'Stop bypasses pending Start');
  assert.match(h.elements.safetyAlert.textContent,/전송 중/);
  assert.equal(h.elements.stop.disabled,false);
  h.latest('/stop').resolve({success:true,message:'ignored server text'});await stop;
  const notice=h.elements.safetyAlert.textContent;
  assert.match(notice,/전달됨/);assert.equal(h.elements.start.disabled,true);
  h.latest('/start').resolve({success:true,message:'late start success'});await start;
  h.latest('/status').resolve({...status,stop_confirmed:true,error:'obsolete status'});await oldStatus;
  assert.equal(h.elements.safetyAlert.textContent,notice);
  assert.notEqual(h.elements.message.textContent,'late start success');
  assert.notEqual(h.elements.error.textContent,'obsolete status');
  const body=JSON.parse(h.latest('/stop').options.body);
  assert.equal(body.browser_id,JSON.parse(h.latest('/start').options.body).browser_id);
  assert.equal(body.browser_id,JSON.parse(h.latest('/heartbeat').options.body).browser_id);
}
async function testStopFailureIsProminentAndPersistent(){
  const h=harness();await h.ready();
  const stop=h.elements.stop.onclick();h.latest('/stop').resolve({success:false,message:'stop rejected'});await stop;
  const alert=h.elements.safetyAlert.textContent;assert.match(alert,/정지 요청 실패/);
  await h.update({stop_confirmed:true,message:'unrelated success'});
  assert.equal(h.elements.safetyAlert.textContent,alert);
  assert.equal(h.elements.start.disabled,true);assert.equal(h.elements.stop.disabled,false);
  const rejected=h.elements.stop.onclick();h.latest('/stop').resolve({success:true},false);await rejected;
  assert.match(h.elements.safetyAlert.textContent,/정지 요청 실패/,'HTTP failure is never accepted');
  const retry=h.elements.stop.onclick();h.latest('/stop').resolve({success:true});await retry;
  assert.match(h.elements.safetyAlert.textContent,/전달됨/);
}
async function testStopSentIsNotPhysicalConfirmation(){
  const h=harness();await h.ready();
  const stop=h.elements.stop.onclick();const earlyStatus=h.poll();
  h.latest('/stop').resolve({success:true});await stop;
  h.latest('/status').resolve({...status,stop_confirmed:true});await earlyStatus;
  assert.match(h.elements.safetyAlert.textContent,/전달됨/,'pre-ack status cannot confirm stop');
  await h.update({phase:'STOPPED',physically_stopped:true,stop_confirmed:false});
  assert.match(h.elements.safetyAlert.textContent,/확인 대기/);
  await h.update({stop_confirmed:true});assert.match(h.elements.safetyAlert.textContent,/정지 상태 수신 확인/);
  await h.update({fresh:false,stop_confirmed:false});assert.match(h.elements.safetyAlert.textContent,/확인 대기/);
}
async function testHeartbeatVisibleSequenceAndFailure(){
  const h=harness();await h.ready();
  const interval=h.intervals.find(t=>t.delay===500);assert.ok(interval);
  interval.fn();let hb=h.latest('/heartbeat');assert.equal(JSON.parse(hb.options.body).seq,2);
  hb.reject(new Error('network'));await settle();
  assert.match(h.elements.linkStatus.textContent,/연결 끊김/);
  assert.equal(h.elements.start.disabled,true);assert.equal(h.elements.complete.disabled,true);assert.equal(h.elements.stop.disabled,false);
  const count=h.requests.length;await h.elements.start.onclick();assert.equal(h.requests.length,count);
  interval.fn();hb=h.latest('/heartbeat');const requests=h.requests.length;
  h.visibility(true);await settle();interval.fn();assert.equal(h.requests.length,requests,'hidden page sends no heartbeat');
  hb.resolve({success:true,control_epoch:0});await settle();assert.match(h.elements.linkStatus.textContent,/숨겨져/);
  h.visibility(false);const visible=h.latest('/heartbeat');assert.equal(JSON.parse(visible.options.body).seq,4);
  visible.resolve({success:true,control_epoch:0});await settle();assert.equal(h.elements.start.disabled,false);
}
async function testStopTimeoutAndNewestStopWins(){
  const h=harness();await h.ready();
  const old=h.elements.stop.onclick(),oldRequest=h.latest('/stop');
  const latest=h.elements.stop.onclick();h.latest('/stop').resolve({success:true});await latest;
  const notice=h.elements.safetyAlert.textContent;oldRequest.reject(new Error('old fail'));await old;
  assert.equal(h.elements.safetyAlert.textContent,notice);
  const timeout=h.elements.stop.onclick();h.fire(8000);await timeout;
  assert.match(h.elements.safetyAlert.textContent,/정지 요청 실패/);assert.equal(h.elements.stop.disabled,false);
}
async function testDelayedStartCarriesOriginalServerEpoch(){
  const h=harness();await h.ready();
  const start=h.elements.start.onclick(),delayed=h.latest('/start');
  assert.equal(JSON.parse(delayed.options.body).control_epoch,0);
  const stop=h.elements.stop.onclick();
  assert.equal(JSON.parse(h.latest('/stop').options.body).control_epoch,0);
  h.latest('/stop').resolve({success:true});await stop;
  h.intervals[0].fn();h.latest('/heartbeat').resolve({success:true,control_epoch:1});await settle();
  // Body was serialized before Stop. The server must reject this late epoch 0 Start.
  assert.equal(JSON.parse(delayed.options.body).control_epoch,0);
  delayed.resolve({success:false,message:'stale control epoch'},false);await start;
  assert.match(h.elements.safetyAlert.textContent,/전달됨/);
  await h.update({stop_confirmed:true});
  const next=h.elements.restart.onclick();
  assert.equal(JSON.parse(h.latest('/restart-stack').options.body).control_epoch,1);
  h.latest('/restart-stack').resolve({success:true});await next;
  h.intervals[0].fn();h.latest('/heartbeat').resolve({success:true,control_epoch:0});await settle();
  assert.equal(h.elements.start.disabled,true,'regressed heartbeat epoch must not unlock commands');
  assert.equal(h.elements.stop.disabled,false);
}

async function testFailedStatusCannotBeUnlockedByHeartbeat(){
  const h=harness();await h.ready();const poll=h.poll();h.latest('/status').reject(new Error('offline'));await poll;
  h.intervals[0].fn();h.latest('/heartbeat').resolve({success:true,control_epoch:0});await settle();
  assert.equal(h.elements.start.disabled,true);assert.equal(h.elements.stop.disabled,false);
}
(async()=>{
  for(const test of [testBusyStartPriorityStopAndOldResponses,testStopFailureIsProminentAndPersistent,testStopSentIsNotPhysicalConfirmation,testHeartbeatVisibleSequenceAndFailure,testStopTimeoutAndNewestStopWins,testFailedStatusCannotBeUnlockedByHeartbeat,testDelayedStartCarriesOriginalServerEpoch]){await test();console.log('PASS',test.name)}
})().catch(error=>{console.error(error);process.exitCode=1});
