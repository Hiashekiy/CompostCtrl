function [q,e]=paper_reference_series(bank,t)
q=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',t,'linear','extrap');q=q./vecnorm(q,2,2);w=q(:,1);x=q(:,2);y=q(:,3);z=q(:,4);e=rad2deg([atan2(2*(w.*x+y.*z),1-2*(x.^2+y.^2)),asin(min(1,max(-1,2*(w.*y-z.*x)))),atan2(2*(w.*z+x.*y),1-2*(y.^2+z.^2))]);
end
