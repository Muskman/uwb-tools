function [x] = minus_transform(a)
% transforming 3 dof pose (-)a
    theta = a(3);
    x = -[cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1]*a;
end