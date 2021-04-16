%% 计算身体工作空间，RobotEDU2
clear;
close all
u2y = 0.162; %config(1)
u2z = 0.092; %config(2)
u3y = 0.162; %config(3)
u3z = -0.092; %config(4)
s2x = 0; %config(5)
s2y = 0.050; %config(6)
s2z = 0.027; %config(7)
s3x = 0; %config(8)
s3y = 0.050; %config(9)
s3z = -0.027; %config(10)
sfx = 0.071; %config(11)
sfy = -0.022; %config(12)
sfz = 0; %config(13)
home_pos = [0.5670, 0.5855, 0.5855];  %config(14:16)
config = [u2y, u2z, u3y, u3z, s2x, s2y, s2z, s3x, s3y, s3z, sfx, sfy, sfz, home_pos];
stroke = 0.209;
leg_base_pe = [ -0.0762, 0, -0.1320,   pi/2,  pi* 2/3,   -pi/2-pi*43.64/180;
                -0.1524, 0,  0,        pi/2,  pi* 3/3,   -pi/2-pi*43.64/180;
                -0.0762, 0,  0.1320,   pi/2,  pi* 4/3,   -pi/2-pi*43.64/180;
                 0.0762, 0, -0.1320,   pi/2,  pi* 1/3,   -pi/2-pi*43.64/180;
                 0.1524, 0,  0,        pi/2,  pi* 0/3,   -pi/2-pi*43.64/180;
                 0.0762, 0,  0.1320,   pi/2,  pi* 5/3,   -pi/2-pi*43.64/180];    
hexa = Hexapod(config,stroke,leg_base_pe);
% 初始足尖坐标
pee = [ -0.30,   -0.52,   -0.52;
        -0.60,   -0.52,    0;
        -0.30,   -0.52,    0.52;
         0.30,   -0.52,   -0.52;
         0.60,   -0.52,    0;
         0.30,   -0.52,    0.52 ]';
peb = [0 0 0 0 0 0];

alpha = 0:10:360;
n = length(alpha);
r_out = zeros(1,n);
dr = 0.02;
y = 0;
ymax = 0.2;
ymin = -0.2;
dy = 0.02;
R = [];
Y = [];
while true
    for i = 1:n
        inWorkspace = true;
        r = 0;
        while inWorkspace
            r = r + dr;
            peb = [r*cosd(alpha(i)), y, r*sind(alpha(i)), 0, 0, 0];
            hexa.invKin(peb,pee);
            inWorkspace = hexa.isInWorkspace();
        end
        r_out(i) = r - dr;
    end
    if y >= 0
        R = [R; r_out];
        Y = [Y; y];
        y = y + dy;
        if y > ymax || norm(r_out) < eps
            y = -dy;
        end
    else
        R = [r_out; R];
        Y = [y; Y];
        y = y - dy;
        if y < ymin ||norm(r_out) < eps
            break;
        end
    end
end
m = size(R,1);
Y = repmat(Y,1,n);
ALPHA = repmat(alpha,m,1);
X = R.*cosd(ALPHA);
Z = R.*sind(ALPHA);
s = surf(X,Z,Y);
s.FaceAlpha = 0.5;
s.EdgeAlpha = 0.2;

fontsz = 10;
set(gca,'Position',[.15 .1 .8 .95],'FontSize',fontsz,'FontName','Times New Roman');
set(gca,'YDir','reverse') %交换y,z轴数据后，须改变Y轴方向以满足右手坐标系
set(gcf,'Position',[200 200 300 200]); 
xlabel('\it x','FontSize',fontsz,'FontName','Times New Roman')
ylabel('\it z','FontSize',fontsz,'FontName','Times New Roman')
zlabel('\it y','FontSize',fontsz,'FontName','Times New Roman')
xticks(-0.6:0.3:0.6)
yticks(-0.6:0.3:0.6)
zticks(-0.2:0.2:0.2)
xlim([-0.6 0.6])
ylim([-0.6 0.6])
zlim([-0.2 0.2])
grid off
axis equal
view([-45,25])

%% 画出工作空间立方体
% hold on
% plotcube([0.25 0.25 0.25],[-0.125 -0.125 -0.15],1,[1 0 0]);

% center = zeros(1,3);
% r1 = 0.4;
% cu1 = upborder(:,(upborder(1,:)-center(1)).^2 + (upborder(3,:)-center(3)).^2 < (r1+dw/5).^2); %找到上边界内以足尖为圆心，半径为r1的圆内的点
% cl1 = lowborder(:,(lowborder(1,:)-center(1)).^2 + (lowborder(3,:)-center(3)).^2 < (r1+dw/5).^2); %找到下边界内以足尖为圆心，半径为r1的圆内的点
% % scatter3(cu1(1,:),cu1(3,:),cu1(2,:)); %测试用
% cyu1 = min(cu1(2,:));
% cyl1 = max(cl1(2,:));
% ch1 = cyu1 - cyl1;
% fc = 'r'; %圆柱颜色
% fa = 1; %圆柱透明度
% [X1,Z1,Y1] = cylinder(r1);
% X1 = X1 + center(1);
% Z1 = Z1 + center(3);
% Y1 = Y1*ch1 + cyl1;
% sc = surf(X1,Z1,Y1);
% sc.FaceColor = fc;
% sc.FaceAlpha = fa;
% sc.EdgeColor = 'none';
% p1 = patch(X1(1,:),Z1(1,:),Y1(1,:),fc);
% p2 = patch(X1(2,:),Z1(2,:),Y1(2,:),fc);
% p1.FaceAlpha = fa;
% p1.LineWidth = 1;
% p2.FaceAlpha = fa;
% p2.LineWidth = 1;

% 画出最小半径与高度的关系
figure
rm = min(R');
ym = Y(:,1);
plot(ym,rm);