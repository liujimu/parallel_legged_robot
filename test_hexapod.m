%% ≤‚ ‘hexapod¿‡
clear
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

% base_pe = [0, 0, 0, pi/2, pi*0/3, -pi/2-pi*70/180];
% leg = Leg.empty;
% for i = 1:6
%     leg(i) = Leg(config, stroke, base_pe);
% end

hexa = Hexapod(config,stroke);
peb = [0.5 -0.14 0.2 0 0 0];
pee = [ -0.60,  -0.95,  -0.60;
        -0.80,  -0.95,   0;
        -0.60,  -0.95,   0.60;
        0.60,   -0.95,  -0.60;
        0.80,   -0.95,   0;
        0.60,   -0.95,   0.60 ]';
hexa.invKin(peb,pee);
hexa.pin
hexa.isInWorkspace()