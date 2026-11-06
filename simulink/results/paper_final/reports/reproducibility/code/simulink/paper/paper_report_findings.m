function paper_report_findings(f,T)
fprintf(f,'## 主要数值结果\n\n');
for cmg=[0 1],S=T(T.experiment=="E1" & T.profile=="step" & T.cmg==cmg,:);fprintf(f,'- E1 CMG=%d：%d 个扫描点，成功 %d，跟踪失败 %d，控制器失败 %d，有效域中断 %d。成功判据见 criteria_v2.md。\n',cmg,height(S),sum(S.status=="SUCCESS"),sum(S.status=="TRACKING_FAIL"),sum(S.status=="CONTROLLER_FAIL"),sum(S.status=="MODEL_DOMAIN_FAIL"));end
S=T(T.experiment=="E2_followup" & T.profile=="step",:);for k=1:height(S),fprintf(f,'- E2 10°单次任务 %s：RMSE %.4f°，峰值 %.4f°，状态 %s。\n',S.method(k),S.rmse_deg(k),S.peak_deg(k),S.status(k));end
S=T(T.experiment=="E3_followup",:);H=max(S.horizon);F=S(~S.masked & abs(S.horizon-H)<1e-8,:);M=S(S.masked~=0,:);if ~isempty(F) && ~isempty(M),fprintf(f,'- E3 相同 %.4g s 时域：完整未来 RMSE %.4f°，屏蔽未来 RMSE %.4f°。这支持未来任务信息对该工况跟踪效果的影响；构型变化起点是否可识别须查看 extra_metrics.json 的安静基线标记。\n',H,F.rmse_deg(1),M.rmse_deg(1));end
S=T(T.experiment=="E4_followup",:);for cmg=[0 1],M=S(S.cmg==cmg & S.status=="SUCCESS" & isfinite(S.gain_db),:);if ~isempty(M),fprintf(f,'- E4 CMG=%d：通过稳态误差判据的最高已测试频率 %.3g Hz；这是本次网格结果，不能外推为系统最高频率。\n',cmg,max(M.frequency));allpoints=S(S.cmg==cmg & isfinite(S.gain_db),:);if all(allpoints.gain_db>-3),fprintf(f,'  本次有效扫描未覆盖 -3 dB 交点，因此不填报 -3 dB 带宽。\n');else,fprintf(f,'  幅频曲线跨越 -3 dB，但因飞行条件时变及参考预瞄，不直接报告为严格 LTI 带宽。\n');end;end;end
fprintf(f,'\n');
end
