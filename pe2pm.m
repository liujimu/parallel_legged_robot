function pm = pe2pm(pe, sequence)
% pe2pm
%   根据位置和313欧拉角来计算位姿矩阵

    pm = eye(4);
    if nargin < 2
        sequence = 'ZXZ';
        if nargin < 1
            return;
        end
    end
    
    pm(1:3,4) = pe(1:3)';
    switch sequence
        case 'ZXZ'
            rotm = RotZ(pe(4)) * RotX(pe(5)) * RotZ(pe(6));
        case 'XYZ'
            rotm = RotX(pe(4)) * RotY(pe(5)) * RotZ(pe(6));
        otherwise
            warning('Axis rotation sequence should be ZXZ or XYZ');
    end
    pm(1:3,1:3) = rotm;
end

