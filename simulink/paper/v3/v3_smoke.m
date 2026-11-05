function v3_smoke(out)
s=v3_spec;s.id='VP_PID_2';s.group='PRECHECK';s.phase='precheck';s.method='PID_surface_first';s.duration=2;m=v3_run_case(s,out);assert(~isempty(m) && ~m.simulation_aborted && m.actual_end_s>=2-.001,'v3:smoke','PID structure smoke failed');
s.id='VP_PID_60';s.duration=60;s.amplitude=5;s.transition=8;m=v3_run_case(s,out);assert(~isempty(m) && m.complete && ~ismember(m.status,{'MODEL_DOMAIN_FAIL','CONTROLLER_FAIL'}),'v3:smoke','PID 60s valid-domain smoke failed');
for a=[5 20],if a==5,dur=20;else,dur=4;end;for cmg=[false true],s=v3_spec;s.group='PRECHECK';s.phase='precheck';s.amplitude=a;s.transition=dur;s.cmg=cmg;s.id=sprintf('VP_E1_%02d_%d',a,cmg);m=v3_run_case(s,out);assert(~isempty(m) && m.complete && ~ismember(m.status,{'MODEL_DOMAIN_FAIL','CONTROLLER_FAIL'}),'v3:smoke','E1 60s smoke failed');end;end
paper_json(fullfile(out,'config','smoke_passed.json'),struct('passed',true,'cases',6));
end
