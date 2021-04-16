%% 计算身体最大俯仰角
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

da = pi/180;
dy = 0.01;
dz = 0.01;
y = -0.2:dy:0.2;
z = -0.52:dz:0.52;
n1 = length(y);
n2 = length(z);
alpha_max = zeros(n1,n2);
alpha_min = zeros(n1,n2);
tic
for i = 1:n1
    for j = 1:n2
        alpha = 0;
        inWorkspace = true;
        while inWorkspace
            peb = [0 y(i) z(j) 0 alpha 0];
            hexa.invKin(peb,pee);
            inWorkspace = hexa.isInWorkspace();
            if inWorkspace
                alpha = alpha + da;
%                 alpha = alpha - da;
            else
                break;
            end
        end
        alpha_max(i,j) = (alpha - da)/pi*180;
%         alpha_max(i,j) = (alpha + da)/pi*180;
        if alpha_max(i,j) < 0
            alpha_max(i,j) = nan;
        end
    end
end
toc

[X,Y] = meshgrid(z,y);
s = surf(X,Y,alpha_max);
s.EdgeColor = 'none';
% s.FaceColor = 'interp';
fontsz = 10;
set(gca,'FontSize',fontsz,'FontName','Times New Roman');
xlabel('\it z','FontSize',fontsz,'FontName','Times New Roman')
ylabel('\it y','FontSize',fontsz,'FontName','Times New Roman')
zlabel('\it \alpha','FontSize',fontsz,'FontName','Times New Roman')
colorbar

%找最大值
[alpha_mm, id] = max(alpha_max(:));
[id_row, id_col] = ind2sub(size(alpha_max),id);
hold on
scatter3(z(id_col),y(id_row),alpha_mm,'filled')
cursor_text = ['(' num2str(z(id_col)) ',' num2str(y(id_row)) ',' num2str(alpha_mm) ')'];
text(z(id_col)+0.02,y(id_row)+0.02,alpha_mm,cursor_text,'FontSize',fontsz,'FontName','Times New Roman');
% zlim([0 30])
% 
% m = size(R,1);
% Y = repmat(Y,1,n);
% ALPHA = repmat(alpha,m,1);
% X = R.*cosd(ALPHA);
% Z = R.*sind(ALPHA);
% s = surf(X,Z,Y);

% 
% set(gca,'YDir','reverse') %交换y,z轴数据后，须改变Y轴方向以满足右手坐标系
% set(gcf,'Position',[200 200 300 200]);
% xticks(-0.6:0.3:0.6)
% yticks(-0.6:0.3:0.6)
% zticks(-0.2:0.2:0.2)
% xlim([-0.6 0.6])
% ylim([-0.6 0.6])
% zlim([-0.2 0.2])
% grid off
% axis equal
% view([-45,25])


