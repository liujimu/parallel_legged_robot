function Rot_matrix = RotX(theta)
%RotX ��X����תtheta�ǵ���ת����
%   Liu Jimu, 2018-11-09
Rot_matrix = [ 1           0           0;
               0  cos(theta) -sin(theta);
               0  sin(theta)  cos(theta)];
end

