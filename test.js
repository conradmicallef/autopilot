const {createSim,step,evaluate,FT,KT,D2R}=require('./core.js');
for (const [wd,wk,gu] of [[270,0,false],[270,8,false],[300,15,false],[220,12,true],[90,6,false]]){
 const s=createSim({windDir:wd,windKt:wk,gusts:gu,seed:7});
 let lastPh='';let n=0;
 while(s.phase!=='STOPPED'&&s.t<900){step(s,0.02);n++;
  if(n%500===0 && process.argv[2]) console.log(s.t.toFixed(0),s.phase,'x',s.x.toFixed(0),'y',s.y.toFixed(0),'h',(s.h/FT).toFixed(0),'V',(s.V/KT).toFixed(0),'th',(s.theta/D2R).toFixed(1),'phi',(s.phi/D2R).toFixed(0),'thr',s.thr.toFixed(2),s.stalled?'STALL':'');}
 console.log(`wind ${wd}/${wk} g${gu}: t=${s.t.toFixed(0)} phase=${s.phase} maxAlt=${(s.maxAlt/FT).toFixed(0)} maxBank=${(s.maxBank/D2R).toFixed(0)}`);
 console.log(' events:',s.events.map(e=>`${e.phase}@${e.t.toFixed(0)}s ${Math.round(e.h/FT)}ft ${Math.round(e.V/KT)}kt (${Math.round(e.x)},${Math.round(e.y)})`).join(' | '));
 console.log(' ',JSON.stringify(evaluate(s)?.map(r=>r.k+': '+r.v+' '+r.ok)), 'crab', s.td&&(s.td.crab/D2R).toFixed(1));
}
