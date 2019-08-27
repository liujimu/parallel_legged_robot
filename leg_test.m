base_pe = [0, 0, 0, pi/2, pi*0/3, -pi/2-pi*70/180];
u2y = 0.162;
u2z = 0.092;
u3y = 0.162;
u3z = -0.092;
sy = 0.05;
sz = 0.0275;
sfx = 0.037;
sfy = 0.006;
sfz = 0;
gamma = 0;
home_pos = [0.5055 0.518 0.518];
stroke = 0.208;
config = [u2y,u2z,u3y,u3z,sy,sz,sfx,sfy,sfz,gamma,home_pos];
leg = Leg(config,stroke,base_pe);
pee = [0.316282 -0.576598 0.200000];
leg.setPee(pee,'B');
% pee = [0.65 0.1 0.2];
% leg.setPee(pee);
disp('pin')
leg.Pin
% 1000*leg.q
disp('alpha')
leg.alpha./pi.*180
disp('\beta')
leg.beta./pi.*180
leg.calcVelocityJacobian();
disp('calculated_jacobian')
disp(leg.jac)

%% 数值法求雅可比
%孙乔的方法，计算有误
jac = cal_numerical_jacobian(@invKin,pee);
disp('numerical_jacobian(by sunqiao)')
disp(jac)
%直接差分，结果与C++结果一致
pin = invKin(pee)
h = 1e-8;
jaci = zeros(3);
for i = 1:3
    pee1 = pee;
    pee1(i) = pee(i) + h;
    pin1 = invKin(pee1);
    jaci(:,i) = (pin1 - pin) ./ h;
end
disp('my numerical_jacobian:')
jaci
jaci*leg.jac

%% 验证运动学正解
leg.forwardKinematics([0.6002 0.6037 0.5759]',[0.64 0.15 -0.15]');
%计算收敛
leg.Pee

%% 函数形式的运动学反解
function y = invKin(x)
    u2y = 0.162;
    u2z = 0.092;
    u3y = 0.162;
    u3z = -0.092;
    sy = 0.05;
    sz = 0.027;
    sfx = 0.065;
    sfy = 0;
    sfz = 0;
    gamma = 0;
    home_pos = [0.5055 0.518 0.518];
    stroke = 0.208;
    config = [u2y,u2z,u3y,u3z,sy,sz,sfx,sfy,sfz,gamma,home_pos];
    leg1 = Leg(config,stroke);
    leg1.setPee(x);
    y = leg1.Pin;
end
