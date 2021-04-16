%% 测试hexapod类
clear
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
config = [u2y, u2z, u3y, u3z, s2x, s2y, s2z, s3x, s3y, s3z, sfx, sfy, sfz, home_pos];
stroke = 0.41;

% base_pe = [0, 0, 0, pi/2, pi*0/3, -pi/2-pi*70/180];
% leg = Leg.empty;
% for i = 1:6
%     leg(i) = Leg(config, stroke, base_pe);
% end

hexa = Hexapod(config,stroke);
peb = [0 0 0 0 0 0];
% 程序上的Pee
pee = [ -0.60,  -0.95,  -0.60;
        -0.80,  -0.95,   0;
        -0.60,  -0.95,   0.60;
        0.60,   -0.95,  -0.60;
        0.80,   -0.95,   0;
        0.60,   -0.95,   0.60 ];
% 令滑块位于行程中间，通过正解算得（经过圆整）的Pee
% pee = [ -0.60,  -0.96,  -0.48;
%         -0.80,  -0.96,   0;
%         -0.60,  -0.96,   0.48;
%         0.60,   -0.96,  -0.48;
%         0.80,   -0.96,   0;
%         0.60,   -0.96,   0.48 ];
hexa.invKin(peb,pee','XYZ');
pin_init = hexa.pin;
hexa.isInWorkspace()
hexa.q
hexa.calcInvJac();
body_inv_jac = hexa.inv_jac;

% q = 0.2*ones(3,6);
% hexa.setQ(zeros(1,6),q,pee);

%% 绘制机器人骨架
leg_base_ori = hexa.leg_base_pe(:,1:3);
hexagon = [leg_base_ori(1:3,:); leg_base_ori(6:-1:4,:); leg_base_ori(1,:)];

fontsz = 11; %label字体大小
color1 = [0 0.4470 0.7410]; %plot线条颜色
color2 = [0.8500 0.3250 0.0980]; %plot线条颜色

plot3(hexagon(:,1),-hexagon(:,3),hexagon(:,2))
hold on
for i = 1:6
    leg_line = [leg_base_ori(i,:); pee(i,:)];
    plot3(leg_line(:,1),-leg_line(:,3),leg_line(:,2),'Color',color1)
end
hold off
axis equal

xlabel('x','FontSize',fontsz,'FontName','Times New Roman')
ylabel('z','FontSize',fontsz,'FontName','Times New Roman')
zlabel('y','FontSize',fontsz,'FontName','Times New Roman')

%% 验算雅可比计算是否正确
numeric_inv_jac = zeros(18,6);
eps1 = 1e-8;
for i = 1:6
    peb = [0 0 0 0 0 0];
    peb(i) = eps1;
    hexa.invKin(peb,pee','XYZ');
    pin_cur = hexa.pin;
    delta_pin = pin_cur - pin_init;
    numeric_inv_jac(:,i) = delta_pin(:)./eps1;
end
cond(numeric_inv_jac)
%比较数值法与推导的雅可比之间的差别，验证雅可比公式
jac_error = norm(body_inv_jac - numeric_inv_jac);
disp('数值雅可比与解析雅可比的二范数的差：')
disp(jac_error)

%% 正解测试
disp('正解测试')
rc_pin = home_pos + stroke/2.*ones(1,3);
for i = 1:6
    hexa.leg(i).forwardKinematics(rc_pin);
    pee_legi = hexa.leg(i).Pee;
    disp(pee_legi')
    hexa.leg(i).setPee(pee_legi,'L');
    disp(hexa.leg(i).Pee_b')
end