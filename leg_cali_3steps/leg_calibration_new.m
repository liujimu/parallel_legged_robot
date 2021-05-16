clear;
close all
addpath('..')
% 标定的腿的序号(1~6)
leg_id = 1;
%% 名义参数
sf = [142 -34 0];
u2 = [0 232 134];
s2 = [0 59 34];
u3 = [0 232 -134];
s3 = [0 59 -34];
% home_pos = [669, 690, 690];  %config(16:18)
% stroke = 410;
leg_base_pe = [ -433.22 , 0 , -199.07,   pi/2,  pi* 2/3,   -pi/2-pi*70/180;
                -483.05 , 0 ,  0,        pi/2,  pi* 3/3,   -pi/2-pi*70/180;
                -433.22 , 0 ,  199.07,   pi/2,  pi* 4/3,   -pi/2-pi*70/180;
                 433.22 , 0 , -199.07,   pi/2,  pi* 1/3,   -pi/2-pi*70/180;
                 483.05 , 0 ,  0,        pi/2,  pi* 0/3,   -pi/2-pi*70/180;
                 433.22 , 0 ,  199.07,   pi/2,  pi* 5/3,   -pi/2-pi*70/180];
             
%% 读取数据，均转成3行n列
% 目标位姿，单位mm
target_pee = dlmread('..\calibration\target_pee.txt')'.*1000;
% 目标（实际）输入，单位mm
target_input = dlmread('..\calibration\target_input.txt')'.*1000 + [669 690 690]';
% 测量位姿，单位mm
filename = ['..\calibration_20190914\l' num2str(leg_id - 1) 'pee.txt'];
measured_pee = dlmread(filename)';
% 实际u1
leg_base_ori = dlmread('..\calibration_20190914\Sphere Centers.txt',',',0,1)';

nPee = length(target_pee);

hm = dlmread('..\calibration_20190914\hm.txt',',',0,1)';
rc = dlmread('..\calibration_20190914\rc.txt',',',0,1)';

%% 变换到辨识u1后的腿坐标系
u1_nom = leg_base_pe(leg_id,1:3)'; %u1名义值
eul = leg_base_pe(leg_id,4:6); %欧拉角名义值
rotm = RotZ(eul(1)) * RotX(eul(2)) * RotZ(eul(3));
u1 = leg_base_ori(:,leg_id);
measured_pee2l = rotm'*measured_pee - rotm'*u1;
pee_error = measured_pee2l - target_pee;
pee_error_norm = sum(pee_error.^2,1).^0.5;

%% 计算身体坐标系下测量位姿与目标位姿的误差
target_pee2b = rotm*target_pee + u1_nom;
pee_error2b = measured_pee - target_pee2b;
pee_error2b_norm = sum(pee_error2b.^2,1).^0.5;

%% 计算实际输入
% config_ori.sf = [142,-34,0]';
% config_ori.s2 = [0,59,34]';
% config_ori.u2 = [0,232,134]';
% config_ori.s3 = [0,59,-34]';
% config_ori.u3 = [0,232,-134]';
% target_input_1 = zeros(3,nPee);
% for i = 1:nPee
%     target_input_1(:,i) = leg_ik(target_pee(:,i),config_ori);
% end

fun1 = @(e)cal_delta_q1(e, sf, measured_pee2l, target_input(1,:));
error1 = zeros(1,3);
% fun1(error1)
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter');
[error1_id,resnorm1,residual1,exitflag1,output1] = lsqnonlin(fun1,error1,[],[],options);

sf_id = sf + error1_id;
param = [sf_id u2 s2];
fun2 = @(e)cal_delta_q2(e, param, measured_pee2l, target_input(2,:));
error2 = zeros(1,6);
% fun2(error2)
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter');
[error2_id,resnorm2,residual2,exitflag2,output2] = lsqnonlin(fun2,error2,[],[],options);
u2_id = u2 + error2_id(1:3);
s2_id = s2 + error2_id(4:6);

param = [sf_id u3 s3];
fun3 = @(e)cal_delta_q2(e, param, measured_pee2l, target_input(3,:));
error3 = zeros(1,6);
% fun3(error3)
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter');
[error3_id,resnorm3,residual3,exitflag3,output3] = lsqnonlin(fun3,error3,[],[],options);
u3_id = u3 + error3_id(1:3);
s3_id = s3 + error3_id(4:6);

error_id = [error1_id error2_id error3_id];


%% 函数定义
% UP支链输入误差
function dq1 = cal_delta_q1(errors, sf, pee, input)
    n = length(pee);
    dq1 = zeros(1,n);
    Sfx = sf(1) + errors(1);
    Sfy = sf(2);
    Sfz = sf(3);
    for i = 1:n
        q1 = input(i); %实际输入
        x = pee(1,i);
        y = pee(2,i);
        z = pee(3,i);
        l1 = sqrt(x.^2 + y.^2 + z.^2 - Sfy.^2 - Sfz.^2) - Sfx; %计算输入
        dq1(i) = l1 - q1;
    end
end

% UPS支链输入误差
function dq2 = cal_delta_q2(errors, param, pee, input)
    % param是行向量
    n = length(pee);
    dq2 = zeros(1,n);
    Sfx = param(1);
    Sfy = param(2);
    Sfz = param(3);
    u2x = param(4) + errors(1);
    u2y = param(5) + errors(2);
    u2z = param(6) + errors(3);
    s2x = param(7) + errors(4);
    s2y = param(8);
    s2z = param(9);
    u2 = [u2x u2y u2z]';
    s2 = [s2x s2y s2z]';
    
    for i = 1:n
        q2 = input(i); %实际输入
        x = pee(1,i);
        y = pee(2,i);
        z = pee(3,i);
        l1 = sqrt(x.^2 + y.^2 + z.^2 - Sfy.^2 - Sfz.^2) - Sfx;
        beta1 = asin(y ./ sqrt((l1 + Sfx).^2 + Sfy.^2)) - asin(Sfy ./ sqrt((l1 + Sfx).^2 + Sfy.^2));
        tmp = (l1 + Sfx) * cos(beta1) - Sfy * sin(beta1);
        alpha1 = atan((Sfz * x - tmp * z) / (tmp * x + Sfz * z));
        rotm = RotY(alpha1) * RotZ(beta1);
        e1 = [1 0 0]';
        l2_vec = rotm*(l1*e1 + s2) - u2;
        l2 = norm(l2_vec);%计算输入
        dq2(i) = l2 - q2;
    end
end