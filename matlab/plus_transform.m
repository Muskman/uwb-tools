function [x] = plus_transform(a,b)
% transforming 3dof pose a(+)b
theta = a(3);
x = repmat(a,1,size(b,2)) + [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1]*b;
end
