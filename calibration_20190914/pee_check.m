l0pee = dlmread('l0pee.txt');
l0pee_check = dlmread('l0pee_check.txt');
delta0 = l0pee - l0pee_check;
max(max(abs(delta0)))
l1pee = dlmread('l1pee.txt');
l1pee_check = dlmread('l1pee_check.txt');
delta1 = l1pee - l1pee_check;
max(max(abs(delta1)))
l2pee = dlmread('l2pee.txt');
l2pee_check = dlmread('l2pee_check.txt');
delta2 = l2pee - l2pee_check;
max(max(abs(delta2)))
l3pee = dlmread('l3pee.txt');
l3pee_check = dlmread('l3pee_check.txt');
delta3 = l3pee - l3pee_check;
max(max(abs(delta3)))
l4pee = dlmread('l4pee.txt');
l4pee_check = dlmread('l4pee_check.txt');
delta4 = l4pee - l4pee_check;
max(max(abs(delta4)))
l5pee = dlmread('l5pee.txt');
l5pee_check = dlmread('l5pee_check.txt');
delta5 = l5pee - l5pee_check;
max(max(abs(delta5)))