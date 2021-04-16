% 生成targetPee和targetPin，靶座安装在脚踝侧面，故Sfipe_tracker与Sfipe不同

clear;
close all;
% l = 0.63:0.05:0.73;
l = 0.9:0.05:1;
a = -pi/18:pi/18:pi/18;
b = -pi/18:pi/18:pi/18;
nl = length(l);
na = length(a);
nb = length(b);
target_pee = zeros(3,nl*na*nb);
target_pee_tracker = zeros(3,nl*na*nb);

for i = 1:nl
    for j = 1:na
        for k = 1:nb
            target_pee(:,9*(i-1)+3*(j-1)+k) = RotY(a(j)) * RotZ(b(k)) * [l(i);0;0];
        end
    end
end
scatter3(target_pee(3,:),target_pee(2,:),target_pee(1,:),8,'b','filled');
hold on
plot3([0 target_pee(3,3)],[0 target_pee(2,3)],[0 target_pee(1,3)])
plot3([0 0.2],[0 0],[0 0])
plot3([0 0],[0 0.2],[0 0])
plot3([0 0],[0 0],[0 0.2])

hold off
axis equal
view(60,45)
fontsz = 10;
set(gca,'FontSize',fontsz,'FontName','Times New Roman');
set(gcf,'Position',[200 200 300 300]); 
xlabel('\it z','FontSize',fontsz,'FontName','Times New Roman')
ylabel('\it y','FontSize',fontsz,'FontName','Times New Roman')
zlabel('\it x','FontSize',fontsz,'FontName','Times New Roman')
grid off
set(gca,'ZDir','reverse') %交换y,z轴数据后，须改变Y轴方向以满足右手坐标系


%% 机构参数初始化
% RobotEDU6参数
% u2y = 0.162;
% u2z = 0.092;
% u3y = 0.162;
% u3z = -0.092;			
% sy = 0.05;
% sz = 0.0275;
% sfx = 0.0504;
% sfy = -0.0086;
% gamma = 0;
% home_pos = [0.5255 0.538 0.538];
% stroke = 0.208;
% RobotXIII参数
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
leg = Leg(config,stroke);
%% 验证目标落脚点是否在工作空间里
inWorkspace = true(1,length(target_pee));
target_input = zeros(size(target_pee));
maxq = 0.1;
minq = 0.1;
for i = 1:length(target_pee)
    leg.setPee(target_pee(:,i));
    target_input(:,i) = leg.q;
    target_pee_tracker(:,i) = leg.Pee_tracker;
    inWorkspace(i) = leg.isInWorkspace;
    if min(leg.q) < minq
        minq = min(leg.q);
    end
    if max(leg.q) > maxq
        maxq = max(leg.q);
    end
end
if sum(inWorkspace) < length(target_pee)
    disp("Not all the target_pee are in workspace.")
end

dlmwrite('target_pee.txt',target_pee','precision','%7.3f');
dlmwrite('target_pee_tracker.txt',target_pee_tracker','precision','%7.3f');
dlmwrite('target_input.txt',target_input','precision','%7.3f');


