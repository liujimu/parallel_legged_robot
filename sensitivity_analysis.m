% 误差参数敏感度分析
clear;
U2ipe = [0, 0.232,  0.134];
U3ipe = [0, 0.232, -0.134];
S2ipe = [0, 0.059,  0.034];
S3ipe = [0, 0.059, -0.034];
Sfipe = [0.142, -0.034, 0];
home_pos = [0.669, 0.69, 0.69];
config_init = [U2ipe(2:3), U3ipe(2:3), S2ipe, S3ipe, Sfipe, home_pos];
stroke = 0.41;
leg = Leg(config_init, stroke);

pee = [0.9, 0, 0]';
leg.setPee(pee);
ideal_inputs = leg.q;

n = length(config_init);
inputs = zeros(3,n);
for i = 1:n
    errer_param = zeros(1,n);
    errer_param(i) = 0.001;
    config = config_init + errer_param;
    leg_comp = Leg(config, stroke);
    leg_comp.setPee(pee);
    inputs(:,i) = leg_comp.q;
end
input_errors = inputs - ideal_inputs;