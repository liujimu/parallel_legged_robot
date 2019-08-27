function Rot_matrix = RotY(theta)
%RotX ÈÆXÖáÐý×ªtheta½ÇµÄÐý×ª¾ØÕó
%   Liu Jimu, 2018-11-09
Rot_matrix = [ cos(theta)  0  sin(theta);
                        0  1           0;
              -sin(theta)  0  cos(theta)];
end

