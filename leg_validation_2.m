%论文使用版本%
clear
close all

rc_v = dlmread('validation_20190920\rc_v.txt',',',0,1);
target_pee_v = [[ 0,    0,     0.1 ];
                [ 0,    0,     0.2 ];
                [ 0,    0,     0.3 ];
                [ 0,    0.05,  0.3 ];
                [ 0,    0.05,  0.2 ];
                [ 0,    0.05,  0.1 ];
                [ 0,    0.05,  0   ];
                [ 0,    0.05, -0.1 ];
                [ 0,    0.05, -0.2 ];
                [ 0,    0.05, -0.3 ];
                [ 0,    0,    -0.3 ];
                [ 0,    0,    -0.2 ];
                [ 0,    0,    -0.1 ];
                [ 0,    0,     0   ];
                [ 0.1,  0,     0   ];
                [ 0.2,  0,     0   ];
                [ 0.3,  0,     0   ];
                [ 0.3,  0.05,  0   ];
                [ 0.2,  0.05,  0   ];
                [ 0.1,  0.05,  0   ];
                [ 0,    0.05,  0   ];
                [-0.1,  0.05,  0   ];
                [-0.2,  0.05,  0   ];
                [-0.3,  0.05,  0   ];
                [-0.3,  0,     0   ];
                [-0.2,  0,     0   ];
                [-0.1,  0,     0   ];
                [ 0,    0,     0   ]];
n = length(target_pee_v);
m = 6;
error = zeros(m,n);
for leg_id = 0:5
    filename = ['validation_20190920\l' num2str(leg_id) 'pee_v.txt'];
    pee_v = dlmread(filename);
    delta = pee_v - 1000.*target_pee_v - repmat(rc_v(leg_id + 1,:),n,1);
    error(leg_id + 1,:) = sum(abs(delta).^2,2).^(1/2);
end
mean_error = mean(error');
std_error = std(error');
AP = mean_error + 3*std_error;
boxplot(error')
fontsz=10;
set(gca,'Position',[.15 .15 .85 .85],'FontSize',fontsz,'FontName','Times New Roman');
set(gcf,'Position',[232 246 320 240]); 
xlabel('Leg index','FontSize',fontsz,'FontName','Times New Roman');
ylabel('Euclidean error (mm)','FontSize',fontsz,'FontName','Times New Roman');
% box off;
% grid off;

%% 标定前的末端误差
leg_base_pe = [ -433.22 , 0 , -199.07,   pi/2,  pi* 2/3,   -pi/2-pi*70/180;
                -483.05 , 0 ,  0,        pi/2,  pi* 3/3,   -pi/2-pi*70/180;
                -433.22 , 0 ,  199.07,   pi/2,  pi* 4/3,   -pi/2-pi*70/180;
                 433.22 , 0 , -199.07,   pi/2,  pi* 1/3,   -pi/2-pi*70/180;
                 483.05 , 0 ,  0,        pi/2,  pi* 0/3,   -pi/2-pi*70/180;
                 433.22 , 0 ,  199.07,   pi/2,  pi* 5/3,   -pi/2-pi*70/180];
% 目标位姿，单位mm
target_pee = dlmread('calibration\target_pee.txt')'.*1000;
% 实际u1
leg_base_ori = dlmread('calibration_20190914\Sphere Centers.txt',',',0,1)';

pee_error2b_norm = [];
for leg_id = 1:6
    % 测量位姿，单位mm
    filename = ['calibration_20190914\l' num2str(leg_id - 1) 'pee.txt'];
    measured_pee = dlmread(filename)';

    u1_nom = leg_base_pe(leg_id,1:3)'; %u1名义值
    eul = leg_base_pe(leg_id,4:6); %欧拉角名义值
    rotm = RotZ(eul(1)) * RotX(eul(2)) * RotZ(eul(3));
    target_pee2b = rotm*target_pee + u1_nom;
    pee_error2b = measured_pee - target_pee2b;
    pee_error2b_norm = [pee_error2b_norm; sum(pee_error2b.^2,1).^0.5];
end

error_bc_mean = mean(pee_error2b_norm');
error_bc_max = max(pee_error2b_norm');
error_bc_min = min(pee_error2b_norm');
error_bc_pos = error_bc_max - error_bc_mean;
error_bc_neg = error_bc_mean - error_bc_min;

error_ac_mean = mean(error');
error_ac_max = max(error');
error_ac_min = min(error');
error_ac_pos = error_ac_max - error_ac_mean;
error_ac_neg = error_ac_mean - error_ac_min;

figure
fontsz=10;
color1 = [0 0.4470 0.7410];
color2 = [0.8500 0.3250 0.0980];
errorbar(1:6,error_bc_mean,error_bc_neg,error_bc_pos,'-o','LineWidth',1,'MarkerSize',3,'MarkerFaceColor',color1);
hold on
errorbar(1:6,error_ac_mean,error_ac_neg,error_ac_pos,'-s','LineWidth',1,'MarkerSize',4,'MarkerFaceColor',color2);
hold off

set(gca,'Position',[.15 .2 .8 .75],'FontSize',fontsz,'FontName','Times New Roman');
set(gcf,'Position',[232 246 320 240]); 
xlabel('Leg index','FontSize',fontsz,'FontName','Times New Roman');
ylabel('Position error (mm)','FontSize',fontsz,'FontName','Times New Roman');
xlim([0.5 6.5])
xticks(1:6)
box off
legend('Before calibration','After calibration','Location','east')
% figure
% target_pee_plot = 1000.*[zeros(1,3);target_pee_v];
% color1 = [0 0.4470 0.7410];
% color2 = [0.8500 0.3250 0.0980];
% plot3(target_pee_plot(:,1),target_pee_plot(:,3),target_pee_plot(:,2),'-o','LineWidth',1,'MarkerSize',3,'MarkerFaceColor',color1)
% grid on
% axis equal
% % fontsz=10;
% set(gca,'Position',[.12 .15 .8 .8],'FontSize',fontsz,'FontName','Times New Roman');
% set(gcf,'Position',[232 246 400 300]); 
% xlabel('\it x','FontSize',fontsz,'FontName','Times New Roman')
% ylabel('\it z','FontSize',fontsz,'FontName','Times New Roman')
% zlabel('\it y','FontSize',fontsz,'FontName','Times New Roman')
% set(gca,'YDir','reverse')
% xticks(-300:100:300)
% yticks(-300:100:300)
% view(-30,18)
