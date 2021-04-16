% 关节刚度对末端刚度的影响
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
hexa = Hexapod(config,stroke,leg_base_pe);
rc_pee = [ -0.60,  -0.95,  -0.60;
           -0.80,  -0.95,   0;
           -0.60,  -0.95,   0.60;
            0.60,  -0.95,  -0.60;
            0.80,  -0.95,   0;
            0.60,  -0.95,   0.60 ];
peb = [0 0 0 0 0 0];
hexa.invKin(peb,rc_pee');
hexa.calcInvJac();
body_inv_jac = hexa.inv_jac;

G_vB = zeros(18,6);
for leg_id = 1:6
    legi = hexa.leg(leg_id);
    peei = rc_pee(leg_id,:)';
    legi.setPee(peei,'B');
    legi.calcVelocityJacobian();
    jacv = legi.jac;
    R_b2h = legi.base_pm(1:3,1:3);
    pf_cross = cross_v3(legi.Pee_b);
    G_Bi = jacv\R_b2h'*[-eye(3) pf_cross];

    G_vB(3*leg_id - 2:3*leg_id,:) = G_Bi;
end


Kq = eye(18);
K_x = G_vB'*Kq*G_vB;

G_vB1 = G_vB([1:3,7:9,13:15],:);
Kq1 = eye(9);
K_x1 = G_vB1'*Kq1*G_vB1;

G_vB2 = G_vB([4:6,10:12,16:18],:);
Kq2 = eye(9);
K_x2 = G_vB2'*Kq2*G_vB2;

%计算关节力
F_body = [1 0 0 0 0 0]';
J_B = pinv(G_vB);
F_in = J_B'*F_body;

%F_in1 = (G_vB*G_vB')\G_vB*F_body;%奇异
F_in1 = G_vB*inv(G_vB'*G_vB)*F_body;
F_in11 = G_vB/(G_vB'*G_vB)*F_body;

%验证
norm(G_vB'*F_in - F_body)
norm(G_vB'*F_in1 - F_body)

%% 由身体力算足尖力(计算结果与上面不一致，可以不考虑)
T = zeros(6,18);
for i = 1:6
    T(:,3*i-2:3*i) = [eye(3);cross_v3(rc_pee(i,:))];
end

F_foot = pinv(T)*F_body;
F_in2 = zeros(18,1);
for i = 1:6
    Fee = F_foot(3*i-2:3*i);
    F_in2(3*i-2:3*i) = hexa.leg(i).calcJointForces(Fee);
end
