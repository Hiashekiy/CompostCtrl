function paper_export_additional_tables(out)
f=fullfile(out,'pid_calibration','tuning.json');if isfile(f),d=jsondecode(fileread(f));rows={};for k=1:numel(d),c=d(k).config;rows{end+1}=struct('candidate',d(k).candidate,'score',d(k).score,'kp_roll',c.kp(1),'kp_pitch',c.kp(2),'kp_yaw',c.kp(3),'ki_roll',c.ki(1),'ki_pitch',c.ki(2),'ki_yaw',c.ki(3),'kd_roll',c.kd(1),'kd_pitch',c.kd(2),'kd_yaw',c.kd(3));end;writetable(struct2table([rows{:}],'AsArray',true),fullfile(out,'tables','E2_pid_tuning.csv'));end
files=dir(fullfile(out,'E2*','*','extra_metrics.json'));rows={};for k=1:numel(files),e=jsondecode(fileread(fullfile(files(k).folder,files(k).name)));if ~isfield(e,'unload_active_ticks'),continue;end;rows{end+1}=struct('case_id',e.id,'experiment',e.experiment,'active_ticks',e.unload_active_ticks,'low_qbar_ticks',e.low_qbar_ticks,'compensation_residual_max_Nm',e.unload_compensation_residual_max,'allocation_residual_rms_Nm',e.allocation_residual_rms);end
if ~isempty(rows),writetable(struct2table([rows{:}],'AsArray',true),fullfile(out,'tables','E2_unload_diagnostics.csv'));end
T=readtable(fullfile(out,'metrics.csv'),'TextType','string');
writetable(T(startsWith(T.experiment,"E2"),:),fullfile(out,'tables','E2_pid_vs_mpc.csv'));
S=T(T.experiment=="E3_followup",:);writetable(S(S.masked==0,:),fullfile(out,'tables','E3_horizon.csv'));if ~isempty(S),ablation=S(S.masked~=0 | S.horizon==min(S.horizon) | S.horizon==max(S.horizon),:);writetable(ablation,fullfile(out,'tables','E3_information_ablation.csv'));end
writetable(T(T.experiment=="E3",:),fullfile(out,'tables','E3_initial_grid_failures.csv'));
writetable(T(T.experiment=="E4",:),fullfile(out,'tables','E4_initial_attempts.csv'));
writetable(T(T.experiment=="E4_followup",:),fullfile(out,'tables','E4_frequency_response.csv'));
end
