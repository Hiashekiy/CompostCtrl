function run_fast_authority_suite(which)
% Sequenced certificate-ablation runs with per-run failure isolation.
root=repo_root;init_paths;
switch which
 case 'smoke'
  runs={'legacy','A',2;'legacy','B',2;'analytic_full','A',2;'analytic_full','B',2; ...
        'analytic_box','B',2;'shadow','B',2};
 case 'main60'
  runs={'legacy','A',60;'legacy','B',60;'analytic_full','A',60;'analytic_full','B',60; ...
        'analytic_box','B',60};
 otherwise
  error('unknown suite %s',which);
end
failures={};
for k=1:size(runs,1)
 try
  run_fast_authority_ablation(runs{k,1},runs{k,2},runs{k,3});
 catch err
  failures{end+1}=sprintf('%s/%s/%gs: %s',runs{k,1},runs{k,2},runs{k,3},err.message); %#ok<AGROW>
  fprintf('FAST_ABLATION_FAILED %s\n',failures{end});
 end
end
out=fullfile(root,'simulink','results','online_authority','fast_certificate','ablation');
if ~exist(out,'dir'),mkdir(out);end
fid=fopen(fullfile(out,sprintf('suite_%s_failures.json',which)),'w');
fprintf(fid,'%s',jsonencode(failures,'PrettyPrint',true));fclose(fid);
fprintf('FAST_SUITE_DONE %s runs=%d failures=%d\n',which,size(runs,1),numel(failures));
end
