function failures=run_torque_margin_feedback_suite(which)
% Sequenced ablation runs with per-run failure isolation.
%
%   smoke        2 s  A/B x B0/M10/F20/MF10   (fast wiring and stability gate)
%   screen10    10 s  A/B x B0/M10/F20/MF10   (safety screening, per the plan)
%   main60      60 s  A/B x B0/M10/F20/MF10   (main experiment; only for groups whose
%                                              10 s screening run exists and passed)
%   sensitivity 60 s  A/B x M05/MF05          (optional 5 % shrink, reported separately)
%
% A failing run is written to failures.json and the next run still executes; nothing is
% ever replaced by zero or by a default value.
root=repo_root;init_paths;
base=fullfile(root,'simulink','results','online_authority','torque_margin_feedback');
if ~exist(base,'dir'),mkdir(base);end
groups={'B0','M10','F20','MF10'};
duration=2;
switch which
 case 'smoke',duration=2;
 case 'screen10',duration=10;
 case 'main60',duration=60;
 case 'sensitivity',groups={'M05','MF05'};duration=60;
 otherwise,error('unknown suite %s (smoke/screen10/main60/sensitivity)',which);
end
failures={};skipped={};
for g=1:numel(groups)
 for arm={'A','B'}
  name=arm{1};tag=sprintf('%s_%s_%gs',groups{g},name,duration);
  if strcmp(which,'main60')
   gate=fullfile(base,sprintf('%s_%s_10s',groups{g},name),'summary.json');
   if ~isfile(gate)
    skipped{end+1}=sprintf('%s: no 10 s screening run',tag); %#ok<AGROW>
    fprintf('TMF_SUITE_SKIPPED %s\n',skipped{end});
    continue;
   end
  end
  try
   run_torque_margin_feedback_ablation(groups{g},name,duration);
  catch err
   failures{end+1}=sprintf('%s: %s',tag,err.message); %#ok<AGROW>
   fprintf('TMF_SUITE_FAILED %s\n',failures{end});
   stack=err.stack;
   detail=struct('tag',tag,'group',groups{g},'arm',name,'duration_s',duration, ...
    'message',err.message,'identifier',err.identifier,'time',datestr(now,31), ...
    'stack',{arrayfun(@(s)sprintf('%s:%d %s',s.file,s.line,s.name),stack, ...
    'UniformOutput',false)});
   fid=fopen(fullfile(base,sprintf('failed_%s.json',tag)),'w');
   fprintf(fid,'%s',jsonencode(detail,'PrettyPrint',true));fclose(fid);
  end
 end
end
fid=fopen(fullfile(base,'failures.json'),'w');
fprintf(fid,'%s',jsonencode(struct('suite',which,'failures',{failures},'skipped',{skipped}), ...
 'PrettyPrint',true));fclose(fid);
fprintf('TMF_SUITE_DONE %s runs=%d failures=%d skipped=%d\n',which,4*numel(groups), ...
 numel(failures),numel(skipped));
end
