% 单腿运动学标定，忽略leg_base姿态误差，考虑U2、U3的X坐标误差
clear;
close all
u2x = 0; %config(1)
u2y = 232; %config(2)
u2z = 134; %config(3)
u3x = 0; %config(4)
u3y = 232; %config(5)
u3z = -134; %config(6)
s2x = 0; %config(7)
s2y = 59; %config(8)
s2z = 34; %config(9)
s3x = 0; %config(10)
s3y = 59; %config(11)
s3z = -34; %config(12)
sfx = 142; %config(13)
sfy = -34; %config(14)
sfz = 0; %config(15)
home_pos = [669, 690, 690];  %config(16:18)
config = [u2x, u2y, u2z, u3x, u3y, u3z, s2x, s2y, s2z, s3x, s3y, s3z, sfx, sfy, sfz, home_pos];
stroke = 410;
leg_base_pe = [ -433.22 , 0 , -199.07,   pi/2,  pi* 2/3,   -pi/2-pi*70/180;
                -483.05 , 0 ,  0,        pi/2,  pi* 3/3,   -pi/2-pi*70/180;
                -433.22 , 0 ,  199.07,   pi/2,  pi* 4/3,   -pi/2-pi*70/180;
                 433.22 , 0 , -199.07,   pi/2,  pi* 1/3,   -pi/2-pi*70/180;
                 483.05 , 0 ,  0,        pi/2,  pi* 0/3,   -pi/2-pi*70/180;
                 433.22 , 0 ,  199.07,   pi/2,  pi* 5/3,   -pi/2-pi*70/180];
% 标定的腿的序号
leg_id = 5;
leg = Leg3(config, stroke, leg_base_pe(leg_id + 1,:));

target_pee = dlmread('calibration\target_pee.txt')'.*1000;
nPee = length(target_pee);
target_input = zeros(3,nPee);
target_pee2b = zeros(3,nPee);
for i = 1:nPee
    leg.setPee(target_pee(:,i));
    target_input(:,i) = leg.q;
    target_pee2b(:,i) = leg.Pee_b;
end

leg_base_ori = dlmread('calibration_20190914\Sphere Centers.txt',',',0,1);

hm = dlmread('calibration_20190914\hm.txt',',',0,1);
hm = hm';
rc = dlmread('calibration_20190914\rc.txt',',',0,1);
rc = rc';

% 标定1号腿（LM）
base_pe = leg_base_pe(leg_id + 1,:);
base_pe(1:3) = leg_base_ori(leg_id + 1,:);
filename = ['calibration_20190914\l' num2str(leg_id) 'pee.txt'];
measured_pee = dlmread(filename);
measured_pee = measured_pee';

% 测量位姿与指令位姿的差
pee_error = measured_pee - target_pee;
norm_error = sum(pee_error.^2,1).^(1/2);
max(norm_error(:))

fun = @(e)cal_input_error(e, config, base_pe, stroke, measured_pee, target_input);
% fun = @(e)cal_input_error(e, stroke, measured_pee, target_input);

param_errors = zeros(1,9);
% 测试目标函数
dq_test = fun(param_errors);

% ub = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 5, 5, 5];
ub = 3*ones(1,9);
lb = -ub;
options_1 = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter');
error_cali = lsqnonlin(fun,param_errors,[],[],options_1);
% options_2 = optimoptions('lsqnonlin','Display','iter');
% [error_cali,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(fun,param_errors,lb,ub,options_2);

%% 计算辨识矩阵
n1 = length(param_errors);
delta_q0 = fun(param_errors);
n2 = length(delta_q0);
T = zeros(n2,n1);
eps = 1e-9;
for i = 1:n1
    delta_param = zeros(1,n1);
    delta_param(i) = eps;
    T(:,i) = (fun(delta_param) - delta_q0)./eps;
end
rank(T)
cond(T)
[Q,R] = qr(T);
[U,S,V] = svd(T);

%% Plot result
dq_init = cal_input_error(param_errors, config, leg_base_pe(leg_id + 1,:), stroke, measured_pee, target_input);
dq_compensated = cal_input_error(error_cali, config, base_pe, stroke, measured_pee, target_input);
% dq_init = cal_input_error(config, stroke, measured_pee_veri, target_input_veri);
% dq_compensated = cal_input_error(config_cali_2, stroke, measured_pee_veri, target_input_veri);
disp('标定前的输入误差（mm）：')
disp(max(abs(dq_init)))
disp('标定后的输入误差（mm）：')
disp(max(abs(dq_compensated)))

% config_error = [error_cali(1:6) zeros(1,7) error_cali(7) 0 error_cali(8:end)];
% config_updated = config + config_error;
% config_updated = config + [error_cali zeros(1,5)];
config_updated = config + [error_cali(1:6) zeros(1,9) error_cali(7:9)];
base_pe_updated = base_pe;
leg_compensated = Leg3(config_updated, stroke, base_pe_updated);
compensated_pee = zeros(size(measured_pee));
measured_pee_error = zeros(1,length(measured_pee));
compensated_pee_error = zeros(1,length(measured_pee));
for i = 1:length(measured_pee)
    measured_pee_error(i) = norm(measured_pee(:,i) - target_pee2b(:,i));
    leg_compensated.setQ(target_input(:,i),target_pee(:,i));
    calculated_pee = leg_compensated.Pee_b;
    compensated_pee(:,i) = calculated_pee;
    compensated_pee_error(i) = norm(measured_pee(:,i) - compensated_pee(:,i));
end

% 足尖坐标误差
plot(measured_pee_error,'--s')
hold on
plot(compensated_pee_error,'-o')
hold off
xlabel('Calibration configuration')
ylabel('Euclidean error (mm)')
legend('Before compensation','After compensation','Location','east')

% 输入误差
dq_init_norm = zeros(nPee,1);
dq_compensated_norm = zeros(nPee,1);
for i = 1:nPee
    dq_init_norm(i) = max(abs(dq_init(3*i-2:3*i)));
    dq_compensated_norm(i) = max(abs(dq_compensated(3*i-2:3*i)));
end
figure
color1 = [0 0.4470 0.7410];
color2 = [0.8500 0.3250 0.0980];
plot(dq_init_norm,'-o','LineWidth',1,'MarkerSize',3,'MarkerFaceColor',color1)
hold on
plot(dq_compensated_norm,'-s','LineWidth',1,'MarkerSize',4,'MarkerFaceColor',color2)
hold off
fontsz=10;
legend('Before compensation','After compensation','Location','east')
set(gca,'Position',[0.15 0.2 0.8 0.75],'FontSize',fontsz,'FontName','Times New Roman');
set(gcf,'Position',[232 246 320 240]); 
xlabel('Calibration configuration index','FontSize',fontsz,'FontName','Times New Roman');
ylabel('Maximal input errors (mm)','FontSize',fontsz,'FontName','Times New Roman');
xlim([1 nPee]);
xticks(1:2:nPee);
% yticks(0:1:6);
box off

%% 将标定结果写入文本文件
% output = error_cali';
% U2ipe = leg_compensated.U2ipe(1:3)';
% U3ipe = leg_compensated.U3ipe(1:3)';
% S2ipe = leg_compensated.S2ipe(1:3)';
% S3ipe = leg_compensated.S3ipe(1:3)';
% Sfipe = leg_compensated.Sfipe(1:3)';
% home_pos = leg_compensated.home_pos';
% 
% fileID = fopen(['calibration_20190914\leg' num2str(leg_id) '_param.txt'],'w');
% fprintf(fileID,'U2i, \t%2.5f, %2.5f, %2.5f\n',U2ipe);
% fprintf(fileID,'U3i, \t%2.5f, %2.5f, %2.5f\n',U3ipe);
% fprintf(fileID,'S2i, \t%2.5f, %2.5f, %2.5f\n',S2ipe);
% fprintf(fileID,'S3i, \t%2.5f, %2.5f, %2.5f\n',S3ipe);
% fprintf(fileID,'Sfi, \t%2.5f, %2.5f, %2.5f\n',Sfipe);
% fprintf(fileID,'home_pos, \t%2.5f, %2.5f, %2.5f\n',home_pos);
% fprintf(fileID,'leg_base, \t%2.5f, %2.5f, %2.5f, %2.5f, %2.5f, %2.5f\n', base_pe_updated);
% fclose(fileID);
 

%% 函数定义
function dq = cal_input_error(param_errors,config, base_pe, stroke, pee, input, n)
    if nargin < 7
        n = length(pee);
    end
    input_error = zeros(3,n);
%     nc = length(config);
%     config_updated = config + [param_errors(4:15) 0 0 0 param_errors(1:3)];
    config_updated = config + [param_errors(1:6) zeros(1,9) param_errors(7:9)];
    base_pe_updated = base_pe;
    for i = 1:n
        leg = Leg3(config_updated, stroke, base_pe_updated);
        leg.setPee(pee(:,i),'B');
        input_error(:,i) = leg.q - input(:,i);
    end
    dq = reshape(input_error,[],1);
end






