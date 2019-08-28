% 误差参数敏感度分析
clear;
% U2ipe = [0, 0.232,  0.134];
% U3ipe = [0, 0.232, -0.134];
% S2ipe = [0, 0.059,  0.034];
% S3ipe = [0, 0.059, -0.034];
% Sfipe = [0.142, -0.034, 0];
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
% config_init = [U2ipe(2:3), U3ipe(2:3), S2ipe, S3ipe, Sfipe, home_pos];
config_init = [u2y, u2z, u3y, u3z, s2x, s2y, s2z, s3x, s3y, s3z, sfx, sfy, sfz, home_pos];
stroke = 0.41;
leg = Leg(config_init, stroke);

%% 根据目标pee计算初始关节变量
target_pee = dlmread('calibration\target_pee.txt')';
nPee = length(target_pee);
ideal_inputs = zeros(3,nPee);
for i = 1:nPee
    leg.setPee(target_pee(:,i));
    ideal_inputs(:,i) = leg.q;
end

delta = 0.001;
nConfig = length(config_init);
inputs = zeros(3,nConfig);
pee_errors = zeros(nPee,nConfig); %行标代表末端位置序号，列标代表误差参数序号
for j = 1:nConfig
    % 每个运动学参数分别加偏置，模拟几何误差
    errer_param = zeros(1,nConfig);
    errer_param(j) = delta;
    config = config_init + errer_param;
    leg_comp = Leg(config, stroke);
    % 正解，计算几何误差对Pee的影响
    for i = 1:nPee
        leg_comp.setQ(ideal_inputs(:,i),target_pee(:,i));
        actual_pee = leg_comp.Pee;
        pee_errors(i,j) = norm(actual_pee - target_pee(:,i))./delta;
    end
end
pee_errors_mean = mean(pee_errors);
pee_errors_std = std(pee_errors);
pee_errors_max = max(pee_errors);
pee_errors_min = min(pee_errors);

bar(pee_errors_max)
hold on
errorbar(pee_errors_mean,pee_errors_std)
bar(pee_errors_min)
hold off