% Ä¬ÈÏ²ÎÊý

leg_id = 1;
leg_base_pe = [ -433.22 , 0 , -199.07,   pi/2,  pi* 2/3,   -pi/2-pi*70/180;
                -483.05 , 0 ,  0,        pi/2,  pi* 3/3,   -pi/2-pi*70/180;
                -433.22 , 0 ,  199.07,   pi/2,  pi* 4/3,   -pi/2-pi*70/180;
                 433.22 , 0 , -199.07,   pi/2,  pi* 1/3,   -pi/2-pi*70/180;
                 483.05 , 0 ,  0,        pi/2,  pi* 0/3,   -pi/2-pi*70/180;
                 433.22 , 0 ,  199.07,   pi/2,  pi* 5/3,   -pi/2-pi*70/180];
config.eul = leg_base_pe(leg_id, 4:6);
config.u1 = leg_base_pe(leg_id, 1:3);
config.u2 = [0,232,134];
config.u3 = [0,232,-134];
config.sf = [142,-34,0];
config.s2 = [0,59,34];
config.s3 = [0,59,-34];
config.hm = [669,690,690];