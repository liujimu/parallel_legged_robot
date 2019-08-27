function Rot_matrix = RotZ(theta)
%RotX ÈÆXÖáÐý×ªtheta½ÇµÄÐý×ª¾ØÕó
%   Liu Jimu, 2018-11-09
Rot_matrix = [ cos(theta) -sin(theta)  0;
               sin(theta)  cos(theta)  0;
                        0           0  1];
end

