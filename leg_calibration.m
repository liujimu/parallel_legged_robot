% 单腿运动学标定
clear;
leg_params = dlmread('leg_cali_params.txt');
config_init = leg_params(1:end-1);
% config_init = [0.2337 0.1339 0.2337 -0.1345 0.0590 0.0340 0 0.0800 -0.0850 0 0.6690 0.6900 0.6900];
stroke = leg_params(end);
leg = Leg(config_init, stroke);

target_input = dlmread('.\target_input.txt',',');
target_input = target_input';
% target_pee = dlmread('.\target_pee_tracker.txt',',');
target_pee = dlmread('.\target_pee.txt',',');
target_pee = target_pee';
measured_pee = dlmread('.\RobotXIII_20190725\l1pee.txt',',');
measured_pee(1:2,:) = [];
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
offset = [0.002, 0.002, 0.002, 0.002, 0.003, 0.003, 0.002, 0.002, 0.002, 0.05, 0.02, 0.02, 0.02];
lb = config_init - offset;
ub = config_init + offset;
options_1 = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter');
config_calibrated_1 = lsqnonlin(fun,config_init,[],[],options_1);
options_2 = optimoptions('lsqnonlin','Display','iter');
[config_calibrated,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(fun,config_init,lb,ub,options_2);

%% 
R = RotX(config_calibrated(9));
Sfipe = [0.0282 0.0100 0]';
Sfipe_calibrated = R*Sfipe;

%% Plot result
dq_init = cal_input_error(config_init, stroke, measured_pee, target_input);
dq_compensated = cal_input_error(config_calibrated, stroke, measured_pee, target_input);
disp('标定前的输入误差（mm）：')
disp(1000*max(abs(dq_init)))
disp('标定后的输入误差（mm）：')
disp(1000*max(abs(dq_compensated)))

% compensated_pee = dlmread('.\RobotEDU6_compensated_20190315\l5pee.txt',',');
% compensated_pee = compensated_pee'./1000;
leg_compensated = Leg(config_calibrated, stroke);
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




