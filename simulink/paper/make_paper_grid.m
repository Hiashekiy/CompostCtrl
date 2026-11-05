function dts=make_paper_grid(original,H)
% Keep six near-term control nodes; quantize all later stages to the 1 ms servo clock.
dts=original;front=min(6,numel(original)-1);dts(1:front)=.2;remaining=round((H-sum(dts(1:front)))*1000);assert(remaining>=numel(original)-front);tail=original(front+1:end);ticks=max(1,round(remaining*tail/sum(tail)));ticks(end)=ticks(end)+remaining-sum(ticks);assert(all(ticks>0));dts(front+1:end)=ticks/1000;assert(abs(sum(dts)-H)<1e-10 && dts(1)==.2 && max(abs(dts*1000-round(dts*1000)))<1e-9);
end
