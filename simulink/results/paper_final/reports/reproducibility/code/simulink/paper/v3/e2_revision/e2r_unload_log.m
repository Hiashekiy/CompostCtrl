function [T,summary]=e2r_unload_log(folder)
p=vehicle_params;d=load(fullfile(folder,'sim_output.mat'),'r');r=d.r;h=load(fullfile(folder,'pid_diagnostics.mat'));rows={};
for k=1:numel(h.times)
 z=h.history{k};t=h.times(k);v=struct('time_s',t,'qbar_Pa',z.qbar,'Hnorm_Nms',norm(z.H),'Hy_Nms',z.H(2),'eta',z.eta,'reason',z.reason,'base_residual_Nm',norm(z.residual),'requested_extra_Nm',0,'selected_extra_Nm',0,'predicted_compensation_error_Nm',NaN,'actual_compensation_error_Nm',NaN,'actual_extra_aero_x_Nm',NaN,'actual_extra_aero_y_Nm',NaN,'actual_extra_aero_z_Nm',NaN,'actual_extra_cmg_x_Nm',NaN,'actual_extra_cmg_y_Nm',NaN,'actual_extra_cmg_z_Nm',NaN,'surface_command_margin_rad',NaN,'H_delta_vs_base_Nms',0,'physical_ray_length_radps',0);
 if isfield(z,'dynamic_choice') && isfield(z.dynamic_choice,'requested_extra_Nm')
  c=z.dynamic_choice;v.requested_extra_Nm=c.requested_extra_Nm;v.selected_extra_Nm=c.selected_extra_Nm;v.predicted_compensation_error_Nm=c.compensation_error_endpoint_Nm;v.surface_command_margin_rad=c.surface_command_margin_rad;v.H_delta_vs_base_Nms=c.H_delta_vs_base_Nms;v.physical_ray_length_radps=c.physical_ray_length_radps;
  if z.eta>0 && t+.02<=r.states.Time(end)+1e-9
   tt=t+.02;i=round(tt*1000)+1;assert(abs(r.states.Time(i)-tt)<1e-8);delta_data=squeeze(r.delta_actual.Data);tc_data=squeeze(r.cmg_torque.Data);air_data=squeeze(r.flight_conditions.Data);x_data=squeeze(r.states.Data);actualdelta=delta_data(i,:)';tc=tc_data(i,:)';air=air_data(i,:)';x=x_data(i,:)';[~,B]=online_aero_moment(p.vehicle,air,x(4:6),actualdelta);
   ea=B*(actualdelta-c.base_surface_endpoint_rad);ec=tc-c.predicted_base_cmg_Nm;v.actual_compensation_error_Nm=norm(ea+ec);v.actual_extra_aero_x_Nm=ea(1);v.actual_extra_aero_y_Nm=ea(2);v.actual_extra_aero_z_Nm=ea(3);v.actual_extra_cmg_x_Nm=ec(1);v.actual_extra_cmg_y_Nm=ec(2);v.actual_extra_cmg_z_Nm=ec(3);
  end
 end
 rows{end+1}=v;
end
T=struct2table([rows{:}]);active=T.eta>0;effective=active & isfinite(T.actual_compensation_error_Nm);summary=struct('active_ticks',sum(active),'max_requested_extra_Nm',max(T.requested_extra_Nm),'max_selected_extra_Nm',max(T.selected_extra_Nm),'predicted_compensation_max_Nm',max(T.predicted_compensation_error_Nm(active),[],'omitnan'),'actual_compensation_max_Nm',max(T.actual_compensation_error_Nm(effective),[],'omitnan'),'actual_compensation_rms_Nm',sqrt(mean(T.actual_compensation_error_Nm(effective).^2)),'strict_predicted_momentum_descent',all(T.H_delta_vs_base_Nms(active)<0),'actual_extra_definition','Measured20ms endpoint increment against a same-instant no-unload conditional endpoint, with natural aero canceled at measured endpoint; not a second counterfactual flight');
writetable(T,fullfile(folder,'dynamic_unload_ticks.csv'));paper_json(fullfile(folder,'dynamic_unload_summary.json'),summary);
end
