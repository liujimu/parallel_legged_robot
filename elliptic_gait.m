%椭圆步态
clear;
close all
dt = 0.01;
% hs = 0;%台阶高度
d = 0.6;
h = 0.05;
T = 1;
t0 = dt:dt:T; %从dt开始，轨迹开头需要补0
n0 = length(t0);
s = (1-cos(pi*t0/T))/2;
%% 加速段
xfl1 = d/2*(1-cos(pi*s))/2; %步长
yfl1 = h*sin(pi*s); %步高
xfr1 = 0*t0;
yfr1 = 0*t0;
xb1 = d/4*(t0/T).^2;
yb1 = 0*t0;

%% 匀速段
xfl2 = d/2*ones(size(t0));
yfl2 = 0*t0;
xfr2 = 2*xfl1;
yfr2 = yfl1;
xb2 = d/4 + d/2*t0/T;
yb2 = 0*t0;

xfl3 = d/2 + xfr2;
yfl3 = yfl1;
xfr3 = d*ones(size(t0));
yfr3 = 0*t0;
xb3 = d*3/4 + d/2*t0/T;
yb3 = 0*t0;

%% 减速段
xfl4 = 1.5*d*ones(size(t0));
yfl4 = 0*t0;
xfr4 = d + xfl1;
yfr4 = yfl1;
xb4 = 1.5*d - fliplr([0,xb1(1:end - 1)]);
yb4 = 0*t0;

%% 画图
t = 0:dt:4*T;
xfl = [0,xfl1,xfl2,xfl3,xfl4];
yfl = [0,yfl1,yfl2,yfl3,yfl4];
xfr = [0,xfr1,xfr2,xfr3,xfr4];
yfr = [0,yfr1,yfr2,yfr3,yfr4];
xb = [0,xb1,xb2,xb3,xb4];
yb = [0,yb1,yb2,yb3,yb4];

fontsz = 10; %字体大小
lw = 1; %线宽

subplot(3,1,1);
plot(t,xfl,'--',t,xfr,'-.',t,xb,'LineWidth',lw);
title('a')
xlabel('Time')
ylabel({'Horizontal'; 'displacement'})
lgd = legend('foot 1','foot 2','body');
legend('boxoff')
xticks([1 2 3 4])
xticklabels({'0.5\itT','\itT','1.5\itT','2\itT'})
yticks([0 0.5*d d 1.5*d])
yticklabels({'0','0.5\itd','\itd','1.5\itd'})
box off
set(gca,'FontSize',fontsz,'FontName','Times New Roman');

subplot(3,1,2);
plot(t,yfl,'--',t,yfr,'-.',t,yb,'LineWidth',lw);
title('b')
xlabel('Time')
ylabel({'Vertical'; 'displacement'})
% legend('foot 1','foot 2','body','Location','eastoutside')
xticks([1 2 3 4])
xticklabels({'0.5\itT','\itT','1.5\itT','2\itT'})
% ylim([0 h])
yticks([0 h])
yticklabels({'0','\ith'})
box off
set(gca,'FontSize',fontsz,'FontName','Times New Roman');

subplot(3,1,3)
plot((xfl-xb),(yfl-yb),'--',(xfr-xb),(yfr-yb),'-.','LineWidth',lw);
% legend('foot 1','foot 2','Location','eastoutside')
title('c')
xlabel('Horizontal displacement')
ylabel({'Vertical'; 'displacement'})
xlim([-0.35*d 0.35*d])
xticks([-0.25*d 0 0.25*d])
xticklabels({'-0.25\itd','0','0.25\itd'})
yticks([0 h])
yticklabels({'0','\ith'})
box off
% axis equal
set(gca,'FontSize',fontsz,'FontName','Times New Roman');
set(gcf,'Position',[200 200 360 400]); 

% figure
% subplot(2,1,1);
% plot(xf,yf);
% subplot(2,1,2);
% plot(xb,yb);

% figure
% axis equal
% hold on
% 
% x=[min(xf2-xb) max(xf1-xb) max(xf1-xb) min(xf2-xb)];
% y=[x(1:2)/d*hs x(3:4)/d*hs+h];
% patch(x,y,[0.75 0.75 0.75]);
% plot((xf1-xb),(yf1-yb),'r',(xf2-xb),(yf2-yb),'b','LineWidth',2)
% axis off
