%% 计算身体工作空间
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
% 初始足尖坐标
pee = [ -0.60,  -0.95,  -0.60;
        -0.80,  -0.95,   0;
        -0.60,  -0.95,   0.60;
        0.60,   -0.95,  -0.60;
        0.80,   -0.95,   0;
        0.60,   -0.95,   0.60 ]';
peb = [0 0 0 0 0 0];

alpha = 0:10:360;
n = length(alpha);
r_out = zeros(1,n);
dr = 0.01;
y = 0;
ymax = 0.25;
ymin = -0.25;
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
R = R.*1000; %单位变为mm
Y = repmat(Y,1,n).*1000;
ALPHA = repmat(alpha,m,1);
X = R.*cosd(ALPHA);
Z = R.*sind(ALPHA);
s = surf(X,Z,Y);
s.FaceAlpha = 0.6;
s.EdgeAlpha = 0.6;

hold on
%找到最小半径及对应的方位角
[MinR,id] = min(R,[],2);
MinAlpha = (id - 1).*10;
MinX = MinR.*cosd(MinAlpha);
MinZ = MinR.*sind(MinAlpha);
MinY = Y(:,1);
plot3(MinX,MinZ,MinY,'r','LineWidth',2);

%修改样式
fontsz = 10;
% set(gca,'Position',[.15 .1 .8 .95],'FontSize',fontsz,'FontName','Times New Roman');
set(gca,'FontSize',fontsz,'FontName','Times New Roman');
set(gca,'YDir','reverse') %交换y,z轴数据后，须改变Y轴方向以满足右手坐标系
set(gcf,'Position',[200 200 400 300]); 
xlabel('{\it x} (mm)','FontSize',fontsz,'FontName','Times New Roman')
ylabel('{\it z} (mm)','FontSize',fontsz,'FontName','Times New Roman')
zlabel('{\it y} (mm)','FontSize',fontsz,'FontName','Times New Roman')
% xticks(-600:200:600)
% yticks(-600:200:600)
% zticks(-200:100:200)
% xlim([-600 600])
% ylim([-500 500])
% zlim([-200 200])
grid off
axis equal
view([45,25])

%% 画出工作空间立方体
% hold on
% plotcube([0.25 0.25 0.25],[-0.125 -0.125 -0.15],1,[1 0 0]);

%% 画出工作空间圆柱
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

%% 最小半径拟合
figure
[Rmax,id] = max(MinR);
p1 = polyfit(MinR(1:id),MinY(1:id),3);
Rfit1 = polyval(p1,MinR(1:id));
p2 = polyfit(MinR(id:end),MinY(id:end),3);
Rfit2 = polyval(p2,MinR(id:end));
scatter(MinR,MinY,'r');
axis equal
hold on 
plot(MinR(1:id), Rfit1, 'r')
plot(MinR(id:end), Rfit2, 'r')
% plot(Rfit1, MinY(1:id), 'r')
% plot(Rfit2, MinY(id:end), 'r')
%画矩形
Rc = 200; %圆柱半径
%多项式求解
% rlim = zeros(size(p1));
% rlim(end) = Rc;
% rt1 = roots(p1 - rlim); 
% rt2 = roots(p2 - rlim);
% %在多个解中找到合理的Z值范围内的解
% z1 = rt1(rt1>min(MinY) & rt1<MinY(id));
% z2 = rt2(rt2>MinY(id) & rt2<max(MinY));
% p1 = [0 z1];
% p2 = [Rc z2];
y1 = polyval(p1,Rc);
y2 = polyval(p2,Rc);
p1 = [0 y1];
p2 = [Rc y2];
%画边界线
plot([0 0],[min(MinY) max(MinY)],'-.');
plot([Rc Rc],[min(MinY) max(MinY)],'--');
rectangle('Position',[p1 p2-p1],'FaceColor','b')
xlabel('{\it r}_{min} (mm)','FontSize',fontsz,'FontName','Times New Roman')
ylabel('{\it y} (mm)','FontSize',fontsz,'FontName','Times New Roman');
set(gca,'FontSize',fontsz,'FontName','Times New Roman');
set(gcf,'Position',[200 200 400 300]); 
set(gca,'Position',[.15 .2 .8 .75],'FontSize',fontsz,'FontName','Times New Roman');
axis equal


%% 绘制纺锤体
figure
[X,Y,Z] = cylinder(MinR);
h = max(MinY) - min(MinY);
Z = h.*Z + min(MinY);
surf(X,Y,Z,Z)
set(gca,'FontSize',fontsz,'FontName','Times New Roman');
set(gca,'YDir','reverse') %交换y,z轴数据后，须改变Y轴方向以满足右手坐标系
set(gcf,'Position',[200 200 400 300]); 
xlabel('{\it x} (mm)','FontSize',fontsz,'FontName','Times New Roman')
ylabel('{\it z} (mm)','FontSize',fontsz,'FontName','Times New Roman')
zlabel('{\it y} (mm)','FontSize',fontsz,'FontName','Times New Roman')
% xticks(-100:50:100)
% yticks(-100:50:100)
% zticks(-100:50:100)
% xlim([-100 100])
% ylim([-105 100])
% zlim([-100 100])
grid off
axis equal
view([45,25])

%% 绘制支撑多边形
hold on
poly_support = [ -0.60,  0, -0.60;
                 -0.80,  0,  0;
                 -0.60,  0,  0.60;
                  0.60,  0,  0.60;
                  0.80,  0,  0;
                  0.60,  0, -0.60 ]'.*1000;
pgon = polyshape(poly_support(1,:),poly_support(3,:));
plot(pgon)
pgon1 = polyshape(poly_support(1,1:2:end),poly_support(3,1:2:end));
pgon2 = polyshape(poly_support(1,2:2:end),poly_support(3,2:2:end));
plot(pgon1)
plot(pgon2)