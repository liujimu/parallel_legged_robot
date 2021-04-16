% 绘制单腿工作空间（在身体坐标系下）
clear;
close all
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
leg_base_pe = [ -0.43322 , 0 , -0.19907,   pi/2,  pi* 2/3,   -pi/2-pi*70/180;
                -0.48305 , 0 ,  0,         pi/2,  pi* 3/3,   -pi/2-pi*70/180;
                -0.43322 , 0 ,  0.19907,   pi/2,  pi* 4/3,   -pi/2-pi*70/180;
                 0.43322 , 0 , -0.19907,   pi/2,  pi* 1/3,   -pi/2-pi*70/180;
                 0.48305 , 0 ,  0,         pi/2,  pi* 0/3,   -pi/2-pi*70/180;
                 0.43322 , 0 ,  0.19907,   pi/2,  pi* 5/3,   -pi/2-pi*70/180];
hexa = Hexapod(config,stroke);
leg_id = 5;
leg = hexa.leg(leg_id);

rc_pee = [ -0.60,  -0.95,  -0.60;
        -0.80,  -0.95,   0;
        -0.60,  -0.95,   0.60;
        0.60,   -0.95,  -0.60;
        0.80,   -0.95,   0;
        0.60,   -0.95,   0.60 ];
peb = [0 0 0 0 0 0];

% l0min = leg.home_pos(1);
% l0max = l0min + stroke;
% Sfi_pe = leg.Sfipe(1:3);
% rmin = norm(Sfi_pe + [l0min 0 0]');
% rmax = norm(Sfi_pe + [l0max 0 0]');
% leg.setQ(stroke/2);
% pee_b = leg.Pee_b;

%% 查找边界
upborder = [];
lowborder = [];
dw = 0.05;
for x = 0:dw:1.8
    for z = -0.8:dw:0.8
        isInWs = false;
        for y = -1.4:dw/5:-0.4
            pee = [x y z]';
            leg.setPee(pee,'B');
            if leg.isInWorkspace && ~isInWs
                lowborder = [lowborder,pee];
                isInWs = true;
            end
            if ~leg.isInWorkspace && isInWs
                upborder = [upborder,[x y-dw z]'];
                isInWs = false;
                break;
            end
        end
    end
end
border = [lowborder, upborder]


%% 绘图
leg_base_ori = hexa.leg_base_pe(:,1:3);
hexagon = [leg_base_ori(1:3,:); leg_base_ori(6:-1:4,:); leg_base_ori(1,:)];
plot3(hexagon(:,1),hexagon(:,3),hexagon(:,2),'k','LineWidth',1)
hold on
for i = 1:6
    leg_line = [leg_base_ori(i,:); rc_pee(i,:)];
    plot3(leg_line(:,1),leg_line(:,3),leg_line(:,2),'k','LineWidth',1)
end

[k,v] = boundary(border',0.85);
% scatter3(rc_pee(5,1),-rc_pee(5,3),rc_pee(5,2),...
%         'MarkerEdgeColor','k',...
%         'MarkerFaceColor','k');
% hold on
% xmin = min(ws(1,:));
% xmax = max(ws(1,:));
% C = (ws(1,:) - xmin)./(xmax - xmin);
s = trisurf(k,border(1,:),border(3,:),border(2,:),border(2,:));
s.FaceColor = 'interp';
s.FaceAlpha = 0.4;
s.EdgeColor = 'k';
s.EdgeAlpha = 0.1;
axis equal

%% 工作空间边界的散点图
% bound_id = unique(k(:));
% figure
% scatter3(border(1,bound_id),border(2,bound_id),border(3,bound_id))
% axis equal

%% 画出工作空间圆柱
center = rc_pee(leg_id,:);

% 窄圆柱
r1 = 0.4;
cu1 = upborder(:,(upborder(1,:)-center(1)).^2 + (upborder(3,:)-center(3)).^2 < (r1+dw/5).^2); %找到上边界内以足尖为圆心，半径为r1的圆内的点
cl1 = lowborder(:,(lowborder(1,:)-center(1)).^2 + (lowborder(3,:)-center(3)).^2 < (r1+dw/5).^2); %找到下边界内以足尖为圆心，半径为r1的圆内的点
% scatter3(cu1(1,:),cu1(3,:),cu1(2,:)); %测试用
cyu1 = min(cu1(2,:));
cyl1 = max(cl1(2,:));
ch1 = cyu1 - cyl1;
fc = 'r'; %圆柱颜色
fa = 1; %圆柱透明度
[X1,Z1,Y1] = cylinder(r1);
X1 = X1 + center(1);
Z1 = Z1 + center(3);
Y1 = Y1*ch1 + cyl1;
sc = surf(X1,Z1,Y1);
sc.FaceColor = fc;
sc.FaceAlpha = fa;
sc.EdgeColor = 'none';
p1 = patch(X1(1,:),Z1(1,:),Y1(1,:),fc);
p2 = patch(X1(2,:),Z1(2,:),Y1(2,:),fc);
p1.FaceAlpha = fa;
p1.LineWidth = 1;
p2.FaceAlpha = fa;
p2.LineWidth = 1;

% 宽圆柱
% r2 = 0.4;
% cu2 = upborder(:,(upborder(1,:)-center(1)).^2 + (upborder(3,:)-center(3)).^2 < (r2+dw/5).^2); %找到上边界内以足尖为圆心，半径为r1的圆内的点
% cl2 = lowborder(:,(lowborder(1,:)-center(1)).^2 + (lowborder(3,:)-center(3)).^2 < (r2+dw/5).^2); %找到下边界内以足尖为圆心，半径为r1的圆内的点
% % scatter3(cu2(1,:),cu2(3,:),cu2(2,:)); %测试用
% cyu2 = min(cu2(2,:));
% cyl2 = max(cl2(2,:));
% ch2 = cyu2 - cyl2;
% fc = 'r';
% fa = 1;
% [X2,Z2,Y2] = cylinder(r2);
% X2 = X2 + center(1);
% Z2 = Z2 + center(3);
% Y2 = Y2*ch2 + cyl2;
% sc = surf(X2,Z2,Y2);
% sc.FaceColor = fc;
% sc.FaceAlpha = fa;
% sc.EdgeColor = 'none';
% p1 = patch(X2(1,:),Z2(1,:),Y2(1,:),fc);
% p2 = patch(X2(2,:),Z2(2,:),Y2(2,:),fc);
% p1.FaceAlpha = fa;
% p2.FaceAlpha = fa;

hold off

view(36,18)
fontsz = 11;
% xlabel('\it x','FontSize',fontsz,'FontName','Times New Roman')
% ylabel('\it z','FontSize',fontsz,'FontName','Times New Roman')
% zlabel('\it y','FontSize',fontsz,'FontName','Times New Roman')
set(gca,'FontSize',fontsz,'FontName','Times New Roman');
set(gcf,'Position',[200 200 400 300]); 
xlim([-0.8 1.6])
ylim([-0.8 0.8])
zlim([-1.2 0])
xticks(-0.8:0.8:1.6)
yticks(-0.8:0.8:0.8)
zticks(-1.2:0.4:0)
set(gca,'YDir','reverse') %交换y,z轴数据后，须改变Y轴方向以满足右手坐标系

%% 使用正解求工作空间
home_pos = [0.669, 0.69, 0.69];  %config(14:16)
stroke = 0.41;

q1 = linspace(home_pos(1),home_pos(1) + stroke,41);
q2 = linspace(home_pos(2),home_pos(2) + stroke,41);
q3 = linspace(home_pos(3),home_pos(3) + stroke,41);
XX = zeros(length(q2),length(q3));
YY = zeros(length(q2),length(q3));
ZZ = zeros(length(q2),length(q3));

for i = 1:length(q2)
    for j = 1:length(q3)
        pin_ = [q1(i), q2(j), min(q3)];
        leg.forwardKinematics(pin_);
        pee_ = leg.Pee_b;
        XX(i,j) = pee_(1);
        YY(i,j) = pee_(2);
        ZZ(i,j) = pee_(3);
    end
end
hold on
surf(XX,ZZ,YY)