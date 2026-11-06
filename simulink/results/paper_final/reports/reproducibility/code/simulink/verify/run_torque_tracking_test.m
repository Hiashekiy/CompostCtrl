function run_torque_tracking_test
% Re-runs the fast-certificate (analytic_full) closed loop once per arm at 60 s and copies
% the full simulation output into results/.../fast_certificate/torque_tracking/.
% Nothing in the controller, allocator, plant or reference is changed.
root=repo_root;init_paths;
base=fullfile(root,'simulink','results','online_authority','fast_certificate');
target=fullfile(base,'torque_tracking');if ~exist(target,'dir'),mkdir(target);end
for arm={'A','B'}
 tag=sprintf('analytic_full_%s_60s',arm{1});
 run_fast_authority_ablation('analytic_full',arm{1},60);
 src=fullfile(base,'ablation',tag);dst=fullfile(target,tag);
 if ~exist(dst,'dir'),mkdir(dst);end
 files=dir(fullfile(src,'*'));
 for k=1:numel(files)
  if files(k).isdir,continue;end
  copyfile(fullfile(src,files(k).name),fullfile(dst,files(k).name));
 end
 fprintf('TORQUE_TRACKING_RUN_COPIED %s\n',dst);
end
fprintf('TORQUE_TRACKING_RUNS_DONE %s\n',target);
end
