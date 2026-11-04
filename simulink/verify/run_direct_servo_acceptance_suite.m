function run_direct_servo_acceptance_suite(which)
% Runs a fixed list of ablation configurations, isolating failures so one bad run cannot
% silently cancel the rest. Never assumes success: failures are printed and counted.
root=repo_root;init_paths;
if nargin<1,which='screen';end
switch which
 case 'screen'
  runs={'hard_stop','fixed','A',2;'hard_stop','fixed','B',2; ...
        'no_descent_gate','fixed','A',2;'no_descent_gate','fixed','B',2; ...
        'no_descent_gate','legacy','B',2};
 case 'main60'
  runs={'hard_stop','fixed','A',60;'hard_stop','fixed','B',60; ...
        'no_descent_gate','fixed','A',60;'no_descent_gate','fixed','B',60};
 case 'legacy60'
  runs={'no_descent_gate','legacy','B',10;'hard_stop','legacy','B',10};
 otherwise
  error('unknown suite %s',which);
end
failures={};
for k=1:size(runs,1)
 try
  run_direct_servo_acceptance_ablation(runs{k,1},runs{k,2},runs{k,3},runs{k,4});
 catch err
  failures{end+1}=sprintf('%s/%s/%s/%gs: %s',runs{k,1},runs{k,2},runs{k,3},runs{k,4},err.message); %#ok<AGROW>
  fprintf('ABLATION_FAILED %s\n',failures{end});
 end
end
out=fullfile(root,'simulink','results','online_authority','direct_servo','acceptance_ablation');
if ~exist(out,'dir'),mkdir(out);end
fid=fopen(fullfile(out,sprintf('suite_%s_failures.json',which)),'w');
fprintf(fid,'%s',jsonencode(failures,'PrettyPrint',true));fclose(fid);
fprintf('ABLATION_SUITE_DONE %s runs=%d failures=%d\n',which,size(runs,1),numel(failures));
end
