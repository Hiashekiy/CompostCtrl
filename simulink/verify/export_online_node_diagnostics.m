function export_online_node_diagnostics
root=repo_root;folder=fullfile(root,'simulink','results','online_authority','stage_C');
names={'iteration','node','node_time_s','height_m','speed_mps','rho_kgpm3','qbar_Pa','alpha_rad','beta_rad', ...
 'natural_roll','natural_pitch','natural_yaw','min_roll','min_pitch','min_yaw','max_roll','max_pitch','max_yaw', ...
 'nominal_roll','nominal_pitch','nominal_yaw','delta_e','delta_a','delta_r','box_scale','vertex_residual_Nm', ...
 'servo_vertex_residual_rad','allocation_roll','allocation_pitch','allocation_yaw','final_HPIPM_status'};
for arm={'A','B'}
 d=load(fullfile(folder,[arm{1} '_single_call.mat']));N=double(d.s.N);rows=[];dt=d.s.dts(:);
 for iteration=1:numel(d.info.iteration_authority)
  a=d.info.iteration_authority{iteration};
  rows=[rows;iteration*ones(N,1) (1:N)' [0;cumsum(dt(1:end-1))] a.air' a.natural' ...
   a.tau_min' a.tau_max' a.nominal_tau' a.delta(:,1:N)' a.scale' max(a.vertex_residual,[],1)' ...
   a.servo_vertex_residual' a.allocation_residual' d.info.status*ones(N,1)]; %#ok<AGROW>
 end
 writetable(array2table(rows,'VariableNames',names),fullfile(folder,[arm{1} '_iteration_nodes.csv']));
end
end
