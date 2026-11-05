function q=paper_quat(e)
e=e(:)/2;c=cos(e);s=sin(e);q=[c(1)*c(2)*c(3)+s(1)*s(2)*s(3);s(1)*c(2)*c(3)-c(1)*s(2)*s(3);c(1)*s(2)*c(3)+s(1)*c(2)*s(3);c(1)*c(2)*s(3)-s(1)*s(2)*c(3)];
end
