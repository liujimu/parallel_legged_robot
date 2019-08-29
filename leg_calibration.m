% 单腿运动学标定
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
leg = Leg(config_init, stroke);

target_pee = dlmread('calibration\target_pee.txt')';
nPee = length(target_pee);
target_input = zeros(3,nPee);
for i = 1:nPee
    leg.setPee(target_pee(:,i));
    target_input(:,i) = leg.q;
end

measured_pee = dlmread('calibration\measured_pee.txt');
measured_pee = measured_pee'./1000;

%% 把数据分为标定项和验证项
target_input_cali = target_input(:,1:2:end);
target_pee_cali = target_pee(:,1:2:end);
measured_pee_cali = measured_pee(:,1:2:end);
target_input_veri = target_input(:,2:2:end);
target_pee_veri = target_pee(:,2:2:end);
measured_pee_veri = measured_pee(:,2:2:end);

n = length(target_pee_cali);
% target_input = zeros(3,n);
% for i = 1:n
%     leg.setPee(target_pee(:,i));
%     target_input(:,i) = leg.q;
% end


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

plot(measured_pee_error)
hold on
plot(compensated_pee_error)
hold off
ylabel('error(mm)')
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




