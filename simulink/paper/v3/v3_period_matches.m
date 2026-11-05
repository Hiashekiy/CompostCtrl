function ok=v3_period_matches(t,period)
% A terminal output may be a partial interval; internal clock intervals may not.
d=diff(t(:));ok=~isempty(d) && all(abs(d(1:end-1)-period)<1e-8) && d(end)>0 && d(end)<=period+1e-8;
end
