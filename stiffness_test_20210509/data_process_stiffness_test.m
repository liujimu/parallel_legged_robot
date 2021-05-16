clear
close all

load('ZForceDisp.mat')
dispz = ZForceDisp(8001:14000,9);
dispz = dispz - dispz(1);
%ft = ZForceDisp(8001:14000,1:3);
fz = ZForceDisp(8001:14000,3);
fz = fz - fz(1);
fz = -fz;
t = 0.002*[0:5999];

%直线拟合
p = polyfit(dispz(1100:5000),fz(1100:5000),1);
f1 = polyval(p,dispz);

disp('刚度')
disp(p(1))

%% 画图
fontsz = 10;
plot(t,dispz,'LineWidth',1)
set(gca,'FontSize',fontsz,'FontName','Times New Roman');
set(gcf,'Position',[200 100 300 240]); 
xlabel('{\it t} (s)','FontSize',fontsz,'FontName','Times New Roman')
ylabel('{\it z} (mm)','FontSize',fontsz,'FontName','Times New Roman')

figure
plot(t,fz,'LineWidth',1)
set(gca,'FontSize',fontsz,'FontName','Times New Roman');
set(gcf,'Position',[200 400 300 240]); 
xlabel('{\it t} (s)','FontSize',fontsz,'FontName','Times New Roman')
ylabel('{\it F_z} (N)','FontSize',fontsz,'FontName','Times New Roman')

figure
plot(dispz,fz,'LineWidth',1)
hold on
plot(dispz,f1,'--r','LineWidth',1)
legend('Original data','Fitting line','Location','northwest')
% scatter(dispz([1100 5000]),fz([1100 5000]))

set(gca,'FontSize',fontsz,'FontName','Times New Roman');
set(gcf,'Position',[200 700 300 240]); 
xlabel('{\it z} (mm)','FontSize',fontsz,'FontName','Times New Roman')
ylabel('{\it F_z} (N)','FontSize',fontsz,'FontName','Times New Roman')

