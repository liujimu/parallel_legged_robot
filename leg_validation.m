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
boxplot(error')
fontsz=11;
set(gca,'Position',[.12 .15 .85 .8],'FontSize',fontsz,'FontName','Times New Roman');
set(gcf,'Position',[232 246 500 330]); 
xlabel('Leg index','FontSize',fontsz,'FontName','Times New Roman');
ylabel('Euclidean error (mm)','FontSize',fontsz,'FontName','Times New Roman');
% box off;
% grid off;

figure
target_pee_plot = 1000.*[zeros(1,3);target_pee_v];
color1 = [0 0.4470 0.7410];
color2 = [0.8500 0.3250 0.0980];
plot3(target_pee_plot(:,1),target_pee_plot(:,3),target_pee_plot(:,2),'-o','LineWidth',1,'MarkerSize',3,'MarkerFaceColor',color1)
grid on
axis equal
fontsz=10;
xlabel('\it x','FontSize',fontsz,'FontName','Times New Roman')
ylabel('\it z','FontSize',fontsz,'FontName','Times New Roman')
zlabel('\it y','FontSize',fontsz,'FontName','Times New Roman')
xticks(-300:100:300)
yticks(-300:100:300)
view(-60,18)
