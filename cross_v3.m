function mat = cross_v3(vec)
%cross_v3 ��ά����ת��˾���
%   �˴���ʾ��ϸ˵��
x = vec(1);
y = vec(2);
z = vec(3);
mat = [ 0 -z  y;
        z  0 -x;
       -y  x  0];
end

