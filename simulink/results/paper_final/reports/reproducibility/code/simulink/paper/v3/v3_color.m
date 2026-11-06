function color=v3_color(method,cmg)
if nargin<2,cmg=true;end;name=lower(string(method));if contains(name,'unload'),color=[.2 .6 .35];elseif startsWith(name,'pid'),color=[.494 .184 .556];elseif ~cmg || contains(name,'surface'),color=[0 .447 .741];else,color=[.85 .325 .098];end
end
