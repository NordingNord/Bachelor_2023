x = [0.5029296875, 0.4150390625, 0.46875, 0.4443359375, 0.48828125, 0.458984375, 0.478515625, 0.4736328125, 0.4833984375, 0.4638671875, 0.498046875, 0.4541015625, 0.44921875, 0.4931640625, 0.419921875, 0.4296875, 0.4345703125,0.439453125];
xp = [-13.427734375, -12.20703125, -9.765625, -8.544921875, -7.32421875, -6.103515625, -4.8828125, -3.662109375, -2.44140625, -1.220703125, 0.0, 1.220703125, 2.44140625, 3.662109375, 4.8828125, 6.103515625, 7.32421875, 8.544921875];
B = sort(x);
Bp = sort(xp);
y1 = [0,0,0,0,0,0,0,5,2,1,201,1622,1052,79,1,3,1,0];
y2 = [0,0,0,0,0,0,0,0,0,0,0,1,0,0,7,67,44,2];
y3 = [ 0,0,0,0,0,0,0,6,10,23,76,473,1836,462,45,12,4,1];
y4 = [1,0,0,0,0,1,1,5,12,18,129,964,1533,309,34,8,2,0];
y5 = [0,1,1,0,0,0,1,4,8,19,107,626,1686,468,37,3,3,0];
y6 = [0,1,1,1,1,0,1,0,1,7,21,101,839,1626,362,36,10,0];

figure('Name','Voltage V')
tiledlayout(3,3)
nexttile
bar(B,y1)
title('5 min, without water, no tube')

nexttile
bar(B,y2)
title('Blow test, without water, no tube')

nexttile
bar(B,y3)
title('5 min, without water, in tube')

nexttile
bar(B,y4)
title('5 min, movement water, in tube')

nexttile
bar(B,y5)
title('5 min, still water, in tube')

nexttile
bar(B,y6)
title('5 min, Moving water, tube sideways on boat')


figure("Name",'Pressure kPa')
tiledlayout(3,3)
nexttile
bar(Bp,y1)
title('5 min, without water, no tube')

nexttile
bar(Bp,y2)
title('Blow test, without water, no tube')

nexttile
bar(Bp,y3)
title('5 min, without water, in tube')

nexttile
bar(Bp,y4)
title('5 min, movement water, in tube')

nexttile
bar(Bp,y5)
title('5 min, still water, in tube')

nexttile
bar(Bp,y6)
title('5 min, Moving water, tube sideways on boat')