% 绘制工作空间
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
% 标定的腿的序号
leg_id = 4;
leg = Leg(config, stroke, leg_base_pe(leg_id + 1,:));

l0min = leg.home_pos(1);
l0max = l0min + stroke;
Sfi_pe = leg.Sfipe(1:3);
rmin = norm(Sfi_pe + [l0min 0 0]');
rmax = norm(Sfi_pe + [l0max 0 0]');
leg.setQ(stroke/2);
pee_b = leg.Pee_b;

%% 查找边界
border = [];
dw = 0.05;
for y = -1:dw:1
    for z = -1:dw:1
        isInWs = false;
        for x = 0.5:dw/5:1.5
            pee = [x y z]';
            leg.setPee(pee,'L');
            if leg.isInWorkspace && ~isInWs
                border = [border,pee];
                isInWs = true;
            end
            if ~leg.isInWorkspace && isInWs
                border = [border,[x y-dw z]'];
                isInWs = false;
                break;
            end
        end
    end
end
[k,v] = boundary(border',0.85);
% scatter3(border(1,:),border(2,:),border(3,:));
scatter3(pee_b(1),-pee_b(3),pee_b(2),...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor','k');
% hold on
% xmin = min(ws(1,:));
% xmax = max(ws(1,:));
% C = (ws(1,:) - xmin)./(xmax - xmin);
s = trisurf(k,1000.*border(3,:),1000.*border(2,:),-1000.*border(1,:),-border(1,:));
s.FaceColor = 'interp';
s.FaceAlpha = 0.5;
s.EdgeColor = 'k';
s.EdgeAlpha = 0.2;
% hold off
% shading interp
fontsz = 11;
xlabel('\it z','FontSize',fontsz,'FontName','Times New Roman')
ylabel('\it y','FontSize',fontsz,'FontName','Times New Roman')
zlabel('\it x','FontSize',fontsz,'FontName','Times New Roman')
xticks(-600:200:600)
yticks(-600:200:600)
zticks(-1200:200:-800)
zticklabels({'1200','1000','800'})

%% 画出标定pee 
target_pee = dlmread('calibration\target_pee.txt')';
hold on
scatter3(1000.*target_pee(3,:),1000.*target_pee(2,:),-1000.*target_pee(1,:),20,'r','filled');
hold off
axis equal
view([-112,18])
