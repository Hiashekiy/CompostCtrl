function R=online_quaternion_rotation(q)
q=q(:)/norm(q);w=q(1);x=q(2);y=q(3);z=q(4);
R=[1-2*(y*y+z*z) 2*(x*y-w*z) 2*(x*z+w*y);2*(x*y+w*z) 1-2*(x*x+z*z) 2*(y*z-w*x);2*(x*z-w*y) 2*(y*z+w*x) 1-2*(x*x+y*y)];
end
