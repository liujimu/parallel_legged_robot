function pm = pe2pm(pe)
% pe2pm
%   根据位置和313欧拉角来计算位姿矩阵

    pm=eye(4);
    %% nargin==0
    if nargin<1
        return;
    end
    %% nargin>=1
    pm(1:3,4) = pe(1:3)';
    rotm = RotZ(pe(4)) * RotX(pe(5)) * RotZ(pe(6));
    pm(1:3,1:3) = rotm;
end

