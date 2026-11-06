function reproduce_paper_final(out)
% Read-only reconstruction from archived native outputs. Never calls sim/build.
out=char(out);code=fullfile(out,'reports/reproducibility/code');addpath(fullfile(code,'simulink/paper'),fullfile(code,'simulink/paper/v3'));
index=readtable(fullfile(out,'manifest.csv'),'TextType','string');p=load(fullfile(code,'simulink/data/native_baseline/A4_descent_replay_S00_with_cmg/vehicle_params.mat'));polar=load(fullfile(code,'simulink/data/public_surrogate/force_polar.mat'));
work=fullfile(out,'.work');if ~isfolder(work),mkdir(work);end
dec=jsondecode(fileread(fullfile(out,'reports/selection_decisions.json')));checks={};events={};profiles={};e3=cell(5,2);horizons=[5 8 12 16 23.2];
for k=1:height(index)
 row=index(k,:);sc=fileparts(fullfile(out,char(row.scenario_path)));cfg=jsondecode(fileread(fullfile(sc,'case_config.json')));m=jsondecode(fileread(fullfile(sc,'metrics.json')));b=load(fullfile(out,char(row.bank_path)));d=load(fullfile(out,char(row.raw_path)),'r');r=d.r;t=r.states.Time(:);x=squeeze(r.states.Data);[qr,ref]=paper_reference_series(b,t);q=paper_euler_quaternions(x(:,1:3));err=rad2deg(2*acos(min(1,abs(sum(q.*qr,2)))));actual=rad2deg(x(:,1:3));
 val=struct('rmse_partial_deg',sqrt(mean(err.^2)),'peak_attitude_error_deg',max(err));if m.complete,val.rmse_full_deg=val.rmse_partial_deg;end
 for item={'prepare','maneuver','post'},key=item{1};if strcmp(key,'prepare'),mask=t>=10&t<cfg.onset;elseif strcmp(key,'maneuver'),mask=t>=cfg.onset&t<=cfg.onset+cfg.transition;else,mask=t>cfg.onset+cfg.transition&t<=cfg.onset+cfg.transition+10;end;if any(mask),val.([key '_rmse_deg'])=sqrt(mean(err(mask).^2));if strcmp(key,'maneuver'),val.maneuver_peak_deg=max(err(mask));end;end;end
 passed=true;maxdiff=0;tested=0;keys=fieldnames(val);for j=1:numel(keys),key=keys{j};if isfield(m,key)&&~isempty(m.(key))&&isfinite(m.(key)),diffval=abs(val.(key)-m.(key));maxdiff=max(maxdiff,diffval);tested=tested+1;passed=passed&&diffval<1e-8;end;end
 % Reproduce the original step stability definition, including incomplete post windows.
 stable=true;ends=cfg.onset+cfg.transition;if ~isempty(cfg.events),ends=cfg.events(:,1)+cfg.events(:,2);end
 if cfg.amplitude==0&&isempty(cfg.events),mask=t>=2;stable=any(mask)&&all(err(mask)<=1);else,for finish=ends(:)',mask=t>=finish+2&t<=finish+10;if ~any(mask)||t(end)<finish+10-.001,stable=false;else,stable=stable&&all(err(mask)<=1);end;end;end
 if ~strcmp(cfg.profile,'sine') && ismember(m.status,{'SUCCESS','TRACKING_FAIL'}),passed=passed&&(stable==strcmp(m.status,'SUCCESS'));end
 if strcmp(cfg.profile,'sine')
  mask=t>=8+2/cfg.frequency-1e-9&t<=cfg.duration+1e-9;fit=v3_fit_periodic(t(mask),ref(mask,2),actual(mask,2),cfg.frequency,8);keys=fieldnames(fit);
  for j=1:numel(keys),key=keys{j};if isfield(m,key)&&~isempty(m.(key)),diffval=abs(double(fit.(key))-double(m.(key)));maxdiff=max(maxdiff,diffval);tested=tested+1;passed=passed&&diffval<1e-8;end;end
 end
 air=align(r.flight_conditions,t,'linear');alpha=rad2deg(air(:,3));valid_domain=all(alpha>=rad2deg(min(polar.alpha_rad))-1e-8 & alpha<=rad2deg(max(polar.alpha_rad))+1e-8);
 c=struct('case_id',cfg.id,'samples',numel(t),'metric_checks',tested,'max_abs_difference',maxdiff,'status',m.status,'original_stability_reproduced',stable,'model_domain_observed_valid',valid_domain,'actual_end_s',t(end),'truth_1ms',max(abs(diff(t)-.001))<1e-8,'passed',passed);checks{end+1}=c;assert(passed,'paper:metrics','Stored metrics differ from native output: %s',cfg.id);
 if strcmp(char(row.experiment),'E2_Controller_Comparison')
  ev=cfg.events;if isempty(ev),ev=[cfg.onset cfg.transition 0 cfg.amplitude 0];end
  for j=1:size(ev,1),start=ev(j,1);finish=start+ev(j,2);next=inf;if j<size(ev,1),next=ev(j+1,1);end;stop=min([finish+10,next,t(end)]);during=t>=start&t<=finish;after=t>finish&t<=stop;v=struct('case_id',cfg.id,'event_index',j,'onset_s',start,'finish_s',finish,'post_end_s',stop,'full_post_10s',stop>=finish+10-1e-9,'maneuver_rmse_deg',sqrt(mean(err(during).^2)),'maneuver_peak_deg',max(err(during)),'post_rmse_deg',sqrt(mean(err(after).^2)));events{end+1}=v;end
 end
 needtrace=~strcmp(char(row.experiment),'E4_Frequency_Tracking') && (~strcmp(char(row.experiment),'E1_CMG_Benefit')||ismember(cfg.id,dec.E1_representatives));
 if needtrace
  tau=align(r.tau_actual,t,'linear');delta=align(r.delta_actual,t,'previous');deltarate=align(r.delta_dot,t,'previous');H=zeros(numel(t),3);tc=H;gamma=zeros(numel(t),4);nu=gamma;acc=gamma;sig=zeros(numel(t),1);support=zeros(numel(t),2);
  if cfg.cmg
   H=align(r.cmg_H,t,'linear');tc=align(r.cmg_torque,t,'linear');nu=align(r.nu_actual,t,'previous');acc=align(r.cmg_acceleration,t,'previous');gamma=x(:,7:10);
   if strcmp(char(row.experiment),'E3_Prediction_Horizon')||startsWith(cfg.id,'E2R_')
    for j=1:numel(t),g=gamma(j,:);A=p.cmg.h0*(-p.s0.*sin(g)+p.gxs0.*cos(g));sig(j)=min(svd(A));co=-A(2,:);lo=max(-p.cmg.gdot_max,nu(j,:)-p.cmg.gddot_max*.2);hi=min(p.cmg.gdot_max,nu(j,:)+p.cmg.gddot_max*.2);support(j,:)=[sum(max(co,0).*hi+min(co,0).*lo),-sum(max(co,0).*lo+min(co,0).*hi)];end
   end
  end
  vals=[t actual ref err tau(:,2) tc(:,2) delta deltarate H gamma nu acc sig support air(:,1) alpha];names={'time_s','roll_deg','pitch_deg','yaw_deg','ref_roll_deg','ref_pitch_deg','ref_yaw_deg','attitude_error_deg','aero_pitch_Nm','cmg_pitch_Nm','elevator_rad','aileron_rad','rudder_rad','elevator_rate_radps','aileron_rate_radps','rudder_rate_radps','Hx','Hy','Hz','g1','g2','g3','g4','nu1','nu2','nu3','nu4','acc1','acc2','acc3','acc4','sigma_min','positive_support_Nm','negative_support_Nm','qbar_Pa','alpha_deg'};
  idx=extrema_indices(vals);writetable(array2table(vals(idx,:),'VariableNames',names),fullfile(work,[cfg.id '.csv']));
  profiles{end+1}=struct('case_id',cfg.id,'native_samples',numel(t),'plot_samples',numel(idx),'selection','20ms endpoints plus every column min/max in each20ms bin; no smoothing; all numerical metrics use full1ms data');
  if strcmp(char(row.experiment),'E3_Prediction_Horizon'),h=find(abs(horizons-sum(cfg.dts))<1e-8);mode=1+(cfg.amplitude==0);e3{h,mode}=struct('t',t,'gamma',gamma,'H',H);end
 end
 if mod(k,20)==0,fprintf('PAPER_RAW_REPRODUCE %d/%d\n',k,height(index));end
end
writetable(struct2table([checks{:}]),fullfile(out,'reports/reproduction_checks.csv'));writetable(struct2table([events{:}]),fullfile(out,'E2_Controller_Comparison/tables/events_from_raw.csv'));paper_json(fullfile(out,'reports/plot_sampling.json'),profiles);
pairs={};for h=1:5,M=e3{h,1};Z=e3{h,2};assert(isequal(M.t,Z.t));t=M.t;dg=rad2deg(M.gamma-Z.gamma);dh=M.H-Z.H;movement=max(abs(dg),[],2);v=struct('horizon_s',horizons(h),'prepreview_delta_deg',max(movement(t<35-horizons(h)-1e-9)),'migration_2deg_s',sustained(t,movement,2));v.advance_s=35-v.migration_2deg_s;pairs{end+1}=v;ix=extrema_indices([t dg dh movement]);writetable(array2table([t(ix),dg(ix,:),dh(ix,:),movement(ix)],'VariableNames',{'time_s','dg1_deg','dg2_deg','dg3_deg','dg4_deg','dHx','dHy','dHz','max_dgamma_deg'}),fullfile(work,sprintf('E3_pair_%g.csv',horizons(h))));end
writetable(struct2table([pairs{:}]),fullfile(out,'E3_Prediction_Horizon/tables/pairs_from_raw.csv'));
paper_json(fullfile(out,'reports/reproduction_receipt.json'),struct('passed',all(cellfun(@(c)c.passed,checks)),'raw_cases',height(index),'new_simulations',0,'MATLAB_version',version,'metric_tolerance',1e-8,'physical_alpha_domain_deg',rad2deg([min(polar.alpha_rad),max(polar.alpha_rad)]),'analysis_only',true,'original_metrics_unchanged',true));
fprintf('PAPER_REPRODUCTION_COMPLETE 197 cases, zero simulations\n');
end
function a=align(ts,t,method),a=interp1(ts.Time,squeeze(ts.Data),t,method,'extrap');end
function idx=extrema_indices(V)
n=size(V,1);m=floor(n/20);idx=[1:20:n,n];for j=2:size(V,2),A=reshape(V(1:20*m,j),20,m);[~,lo]=min(A,[],1);[~,hi]=max(A,[],1);idx=[idx,(0:m-1)*20+lo,(0:m-1)*20+hi];end;idx=unique(idx);
end
function onset=sustained(t,x,threshold)
onset=NaN;active=x>threshold&t<35;edges=diff([false;active;false]);starts=find(edges==1);ends=find(edges==-1)-1;for k=1:numel(starts),if t(ends(k))-t(starts(k))>=.6-1e-9,onset=t(starts(k));return;end;end
end
