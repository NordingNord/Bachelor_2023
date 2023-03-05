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

%%
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



%%
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

%%
x2 = [0.49199218750000007, 0.49414062500000006, 0.49521484375, 0.49091796875000004, 0.49843750000000003, 0.49736328125000007, 0.49628906250000004, 0.49306640625000003, 0.5005859375, 0.49951171875000006, 0.50166015625, 0.48984375, 0.48876953125000006, 0.5027343750000001, 0.5038085937500001, 0.5081054687500001, 0.5048828125, 0.50703125, 0.48447265625, 0.50595703125, 0.5145507812500001, 0.48339843750000006, 0.48769531250000003, 0.48554687500000004, 0.511328125, 0.511328125, 0.5145507812500001, 0.515625, 0.5134765625000001, 0.51669921875, 0.5188476562500001, 0.5177734375, 0.5263671875, 0.5199218750000001, 0.51025390625, 0.51240234375, 0.52314453125, 0.5091796875000001, 0.50703125, 0.49843750000000003, 0.5081054687500001, 0.49199218750000007, 0.49306640625000003, 0.49843750000000003, 0.49414062500000006, 0.49628906250000004, 0.48876953125000006, 0.49521484375, 0.49736328125000007, 0.49951171875000006, 0.49091796875000004, 0.48984375, 0.5038085937500001, 0.50703125, 0.48125000000000007, 0.5048828125, 0.5134765625000001, 0.5005859375, 0.5145507812500001, 0.5027343750000001, 0.48769531250000003, 0.48232421875000003, 0.50166015625, 0.51025390625, 0.48554687500000004, 0.515625, 0.47802734375000006, 0.511328125, 0.50595703125, 0.5177734375, 0.48339843750000006, 0.49521484375, 0.49306640625000003, 0.49951171875000006, 0.49736328125000007, 0.48984375, 0.49091796875000004, 0.49414062500000006, 0.49199218750000007, 0.49843750000000003, 0.49628906250000004, 0.48876953125000006, 0.5005859375, 0.5048828125, 0.48662109375000007, 0.48769531250000003, 0.50166015625, 0.5027343750000001, 0.48447265625, 0.48554687500000004, 0.5091796875000001, 0.50595703125, 0.48232421875000003, 0.49736328125000007, 0.49306640625000003, 0.49091796875000004, 0.49414062500000006, 0.49199218750000007, 0.49521484375, 0.49628906250000004, 0.49951171875000006, 0.48662109375000007, 0.49843750000000003, 0.5005859375, 0.48984375, 0.5081054687500001, 0.48876953125000006, 0.48554687500000004, 0.48769531250000003, 0.50703125, 0.50166015625, 0.5027343750000001, 0.48017578125000004, 0.48447265625, 0.5038085937500001];
B2 = sort(x2);
BD2 = unique(B2(:).')
y1 = [0,0,0,0,1,1,1,0,1,5,29,103,222,452,650,612,422,234,124,43,36,20,10,6,3,1,2,3,0,0,1,0,0,1,0,0,0,0,0,0,0];
y2 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,1,9,9,20,19,35,44,25,18,11,4,5,2,1];
y3 = [1,0,2,1,1,0,2,0,5,24,89,203,397,606,617,477,246,144,69,33,14,5,11,5,4,4,7,0,0,1,1,0,1,1,2,0,1,0,0,0,0];
y4 = [0,0,0,1,0,1,1,9,16,67,169,374,645,617,550,286,149,66,43,11,4,1,1,0,2,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0];
y5 = [0,1,0,0,0,1,3,7,13,56,135,342,515,687,582,342,156,83,36,21,8,1,2,1,0,0,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0];

figure("Name",'Voltage with ref 1.1')
tiledlayout(3,3)
nexttile
bar(BD2,y1)
title('5 min, without water, no tube')

nexttile
bar(BD2,y2)
title('Blow test, without water, no tube')

nexttile
bar(BD2,y3)
title('5 min, without water, in tube')

nexttile
bar(BD2,y4)
title('5 min, movement water, in tube')

nexttile
bar(BD2,y5)
title('5 min, still water, in tube')
