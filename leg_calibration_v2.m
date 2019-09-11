% 单腿运动学标定，将leg_base姿态角也作为标定参数
clear;
u2y = 0.232; %config(1)
u2z = 0.134; %config(2)
u3y = 0.232; %config(3)
u3z = -0.134; %config(4)
s2x = 0; %config(5)
s2y = 0.059; %config(6)
s2z = 0.034; %config(7)
s3x = 0; %config(8)
s3y = 0.059; %config(9)
s3z = -0.034; %config(10)
sfx = 0.142; %config(11)
sfy = -0.034; %config(12)
sfz = 0; %config(13)
home_pos = [0.669, 0.69, 0.69];  %config(14:16)
config_init = [u2y, u2z, u3y, u3z, s2x, s2y, s2z, s3x, s3y, s3z, sfx, sfy, sfz, home_pos];
stroke = 0.41;
leg_base_pe = [ -0.43322 , 0 , -0.19907,   pi/2,  pi* 2/3,   -pi/2-pi*70/180;
                -0.48305 , 0 ,  0,         pi/2,  pi* 3/3,   -pi/2-pi*70/180;
                -0.43322 , 0 ,  0.19907,   pi/2,  pi* 4/3,   -pi/2-pi*70/180;
                 0.43322 , 0 , -0.19907,   pi/2,  pi* 1/3,   -pi/2-pi*70/180;
                 0.48305 , 0 ,  0,         pi/2,  pi* 0/3,   -pi/2-pi*70/180;
                 0.43322 , 0 ,  0.19907,   pi/2,  pi* 5/3,   -pi/2-pi*70/180];
leg = Leg(config_init, stroke, leg_base_pe(2,:));

target_pee = dlmread('calibration\target_pee.txt')';
nPee = length(target_pee);
target_input = zeros(3,nPee);
target_pee2b = zeros(3,nPee);
for i = 1:nPee
    leg.setPee(target_pee(:,i));
    target_input(:,i) = leg.q;
    target_pee2b(:,i) = leg.Pee_b;
end

% 标定1号腿（LM）
measured_pee = dlmread('calibration\l1pee_b.txt');
measured_pee = measured_pee(3:end,:)'./1000;

fun = @(e)cal_input_error(e, stroke, measured_pee_cali, target_input_cali);
% fun = @(e)cal_input_error(e, stroke, measured_pee, target_input);
offset = [0.002, 0.002, 0.002, 0.002, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.002, 0.002, 0.002, 0.005, 0.005, 0.005];
lb = config_init - offset;
ub = config_init + offset;
options_1 = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter');
config_cali_1 = lsqnonlin(fun,config_init,[],[],options_1);
options_2 = optimoptions('lsqnonlin','Display','iter');
[config_cali_2,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(fun,config_init,lb,ub,options_2);

%% 
% R = RotX(config_cali_2(9));
% Sfipe = [0.0282 0.0100 0]';
% Sfipe_calibrated = R*Sfipe;

%% Plot result
% dq_init = cal_input_error(config_init, stroke, measured_pee, target_input);
% dq_compensated = cal_input_error(config_cali_2, stroke, measured_pee, target_input);
dq_init = cal_input_error(config_init, stroke, measured_pee_veri, target_input_veri);
dq_compensated = cal_input_error(config_cali_2, stroke, measured_pee_veri, target_input_veri);
disp('标定前的输入误差（mm）：')
disp(1000*max(abs(dq_init)))
disp('标定后的输入误差（mm）：')
disp(1000*max(abs(dq_compensated)))

% compensated_pee = dlmread('.\RobotEDU6_compensated_20190315\l5pee.txt',',');
% compensated_pee = compensated_pee'./1000;
leg_compensated = Leg(config_cali_2, stroke);
compensated_pee = zeros(size(measured_pee));
measured_pee_error = zeros(1,length(measured_pee));
compensated_pee_error = zeros(1,n);
for i = 1:length(measured_pee)
    measured_pee_error(i) = 1000.*norm(measured_pee(:,i) - target_pee(:,i));
    leg_compensated.setQ(target_input(:,i),target_pee(:,i));
    calculated_pee = leg_compensated.Pee;
    compensated_pee(:,i) = calculated_pee;
    compensated_pee_error(i) = 1000.*norm(measured_pee(:,i) - compensated_pee(:,i));
end

plot(measured_pee_error,'-s')
hold on
plot(compensated_pee_error,'-o')
hold off
xlabel('Calibration configuration')
ylabel('Euclidean error (mm)')
legend('Before compensation','After compensation')

%% 函数定义
function dq = cal_input_error(config, stroke, pee, input, n)
    if nargin < 5
        n = length(pee);
    end
    input_error = zeros(3,n);
    for i = 1:n
        leg = Leg(config, stroke);
        leg.setPee(pee(:,i));
        input_error(:,i) = leg.q - input(:,i);
    end
    dq = reshape(input_error,[],1);
end






