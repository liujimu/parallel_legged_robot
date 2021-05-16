function [q] = leg_ik(pos,config)
%leg_ik 单腿反解
%   此处显示详细说明
x = pos(1);
y = pos(2);
z = pos(3);

sf = config.sf;
s2 = config.s2;
u2 = config.u2;
s3 = config.s3;
u3 = config.u3;

Sfx = sf(1);
Sfy = sf(2);
Sfz = sf(3);

l1 = sqrt(x.^2 + y.^2 + z.^2 - Sfy.^2 - Sfz.^2) - Sfx;

beta1 = asin(y ./ sqrt((l1 + Sfx).^2 + Sfy.^2)) - asin(Sfy ./ sqrt((l1 + Sfx).^2 + Sfy.^2));
tmp = (l1 + Sfx) * cos(beta1) - Sfy * sin(beta1);
alpha1 = atan((Sfz * x - tmp * z) / (tmp * x + Sfz * z));
rotm = RotY(alpha1) * RotZ(beta1);

e1 = [1 0 0]';
l2_vec = rotm*(l1*e1 + s2) - u2;
l2 = norm(l2_vec);
l3_vec = rotm*(l1*e1 + s3) - u3;
l3 = norm(l3_vec);

q = [l1 l2 l3]';
end

