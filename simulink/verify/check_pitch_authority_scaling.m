function check_pitch_authority_scaling
root=repo_root;p=vehicle_params;v=p.vehicle;
folder=fullfile(root,'simulink','results','online_authority','pitch_scale_repair');if ~exist(folder,'dir'),mkdir(folder);end
[natural,B]=online_aero_moment(v,[1;3640;deg2rad(32);0],zeros(3,1),zeros(3,1));
half=abs(B)*v.delta_max(:);physical_hi=natural(2)+half(2);
assert(physical_hi>0,'The supplied airframe does not admit the stated trim example');
Q=.55/physical_hi;
[natural,B]=online_aero_moment(v,[Q;3640;deg2rad(32);0],zeros(3,1),zeros(3,1));
box=online_certified_torque_box(natural,B,-v.delta_max(:),v.delta_max(:),zeros(3,1));
old_uniform_hi=natural(2)+.774*(abs(B(2,:))*v.delta_max(:));
assert(abs(box.tau_max(2)-.55)<1e-9 && box.axis_scale(2)==1);
assert(old_uniform_hi<0 && max(box.vertex_residual)<1e-9);
% Algebraic reproduction of the reported +0.55 -> -1.55 example. Its complete
% rho/V/omega working point was not supplied; do not label it a truth replay.
example_half=(.55+1.55)/(1-.774);
example_B=[0 1 .2;example_half 0 0;0 .1 -1];example_natural=[0;.55-example_half;0];
example=online_certified_torque_box(example_natural,example_B,-ones(3,1),ones(3,1),zeros(3,1));
assert(abs(example.tau_max(2)-.55)<1e-9 && abs(example_natural(2)+.774*example_half+1.55)<1e-9);
% Isolated pitch remains full even when roll/yaw are rank deficient.
singular=online_certified_torque_box(zeros(3,1),[0 1 0;2 0 0;0 1 0],-ones(3,1),ones(3,1),zeros(3,1));
assert(abs(singular.tau_max(2)-2)<1e-9 && max(singular.tau_max([1 3]))<1e-8);
% Fully coupled maps still require the original common-group certificate.
coupled=online_certified_torque_box(zeros(3,1),[1 .3 .2;.2 1 .1;.1 .2 1],-ones(3,1),ones(3,1),zeros(3,1));
assert(max(coupled.vertex_residual)<1e-9 && numel(unique(coupled.axis_scale))==1);
save(fullfile(folder,'regression.mat'),'Q','natural','B','box','old_uniform_hi','singular','coupled','example');
fprintf('PITCH_SCALE_PASS alpha=32 Q=%g old_hi=%g new_hi=%g scales=%s\n',Q,old_uniform_hi,box.tau_max(2),mat2str(box.axis_scale'));
end
