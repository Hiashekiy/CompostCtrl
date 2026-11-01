function c = s08_cross3(a, b)
%S08_CROSS3  Column-wise cross product of two 3xN stacks (nearspace.cmg._cross3).
c = [a(2, :) .* b(3, :) - a(3, :) .* b(2, :);
     a(3, :) .* b(1, :) - a(1, :) .* b(3, :);
     a(1, :) .* b(2, :) - a(2, :) .* b(1, :)];
end
