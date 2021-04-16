function mat = cross_v3(vec)
%cross_v3 三维向量转叉乘矩阵
%   此处显示详细说明
x = vec(1);
y = vec(2);
z = vec(3);
mat = [ 0 -z  y;
        z  0 -x;
       -y  x  0];
end

