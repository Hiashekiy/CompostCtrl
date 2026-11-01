function q = s08_quat_mul(a, b)
%S08_QUAT_MUL  Scalar-first Hamilton product (nearspace.model6.quat_mul).
aw = a(1); ax = a(2); ay = a(3); az = a(4);
bw = b(1); bx = b(2); by = b(3); bz = b(4);
q = [aw * bw - ax * bx - ay * by - az * bz;
     aw * bx + ax * bw + ay * bz - az * by;
     aw * by - ax * bz + ay * bw + az * bx;
     aw * bz + ax * by - ay * bx + az * bw];
end
