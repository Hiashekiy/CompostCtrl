function check_online_flight_prediction
% Stage B prerequisite: replay measured native attitude, compare translation.
root=repo_root;out=fullfile(root,'simulink','results','online_authority','stage_B');
build_prediction_flight_environment;load_system('Prediction_FlightEnvironment');
report=nan(4,4);names={'static','pitch','roll','large_alpha'};
for k=1:4
 si=Simulink.SimulationInput('Prediction_FlightEnvironment');
 moment=zeros(3,1);
 if k==2,moment(2)=.1;elseif k==3,moment(1)=.1;end
 if k==4
  env=load(fullfile(root,'simulink','data','native_environment.mat'));
  vb=eul2rotm([0 .4 0])'*env.V_ned0(:);
  si=si.setBlockParameter('Prediction_FlightEnvironment/Aircraft_6DOF','eul_0','[0 .4 0]');
  si=si.setBlockParameter('Prediction_FlightEnvironment/Aircraft_6DOF','Vm_0',mat2str(vb',17));
 end
 si=si.setExternalInput([0 moment';23.2 moment']);r=sim(si);
 t=r.euler.Time;angles=squeeze(r.euler.Data);if size(angles,1)~=numel(t),angles=angles';end
 q=eul2quat(angles(:,[3 2 1]))';
 pos=squeeze(r.position_NED.Data);vel=squeeze(r.velocity_NED.Data);
 if size(pos,1)~=numel(t),pos=pos';vel=vel';end
 initial=[pos(1,:) vel(1,:)]';
 predicted=online_translation_prediction(initial,t,q,.001);
 finer=online_translation_prediction(initial,t,q,.0005);
 runtime_step=online_translation_prediction(initial,t,q,.1);
 state=[pos vel]';err=predicted.state-state;
 report(k,:)=[max(abs(err(1:3,:)),[],'all') max(abs(err(4:6,:)),[],'all') max(abs(predicted.state-finer.state),[],'all') max(abs(err)./max(abs(state),1),[],'all')];
 air=squeeze(r.air.Data);if size(air,1)~=numel(t),air=air';end
 truth_air=[-pos(:,3) air(:,2) 2*air(:,1)./air(:,2).^2 air(:,1) air(:,3:4)]';
 air_error=predicted.air-truth_air;
 air_max_abs=max(abs(air_error),[],2);
 air_max_relative=max(abs(air_error)./max(abs(truth_air),1e-12),[],2);
 template=load(fullfile(root,'simulink','tests','05_python_matlab_qp_equivalence','production_online_qp.mat'));
 node_time=[0;cumsum(template.dts(:))];
 node_prediction=online_translation_prediction(initial,node_time,interp1(t,q',node_time,'linear')',.001);
 runtime_node=online_translation_prediction(initial,node_time,interp1(t,q',node_time,'linear')',.1);
 runtime_step_error=max(abs(runtime_node.state-node_prediction.state),[],'all');
 assert(runtime_step_error<.001,'Online integration substep mismatch');
 % Full replay uses the recorded attitude at 1 ms; node replay exposes interpolation error.
 node_error=node_prediction.state-interp1(t,state',node_time,'linear')';
 save(fullfile(out,[names{k} '.mat']),'r','predicted','finer','err','air_error','air_max_abs','air_max_relative','node_prediction','node_error','runtime_step','runtime_node','runtime_step_error');
 node_max=[max(abs(node_error(1:3,:)),[],'all') max(abs(node_error(4:6,:)),[],'all')];
 writematrix(node_max,fullfile(out,[names{k} '_node_errors.csv']));
 writematrix(report,fullfile(out,'translation_errors.csv'));
 fprintf('REPLAY_CASE %s position=%g velocity=%g sensitivity=%g\n',names{k},report(k,1:3));
 % Predeclared replay tolerances: 1 cm position, 1 mm/s velocity.
 assert(report(k,1)<.01 && report(k,2)<.001,'Native replay mismatch');
 assert(node_max(1)<.01 && node_max(2)<.001,'Nonuniform attitude interpolation exceeds replay tolerances');
end
writematrix(report,fullfile(out,'translation_errors.csv'));disp(report);
fprintf('ONLINE_FLIGHT_REPLAY_PASS\n');close_system('Prediction_FlightEnvironment',0);
end
