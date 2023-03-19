%x = [0.5029296875, 0.4150390625, 0.46875, 0.4443359375, 0.48828125, 0.458984375, 0.478515625, 0.4736328125, 0.4833984375, 0.4638671875, 0.498046875, 0.4541015625, 0.44921875, 0.4931640625, 0.419921875, 0.4296875, 0.4345703125,0.439453125];
%xp = [-13.427734375, -12.20703125, -9.765625, -8.544921875, -7.32421875, -6.103515625, -4.8828125, -3.662109375, -2.44140625, -1.220703125, 0.0, 1.220703125, 2.44140625, 3.662109375, 4.8828125, 6.103515625, 7.32421875, 8.544921875];
%B = sort(x);
%Bp = sort(xp);
%y1 = [0,0,0,0,0,0,0,5,2,1,201,1622,1052,79,1,3,1,0];
%y2 = [0,0,0,0,0,0,0,0,0,0,0,1,0,0,7,67,44,2];
%y3 = [ 0,0,0,0,0,0,0,6,10,23,76,473,1836,462,45,12,4,1];
%y4 = [1,0,0,0,0,1,1,5,12,18,129,964,1533,309,34,8,2,0];
%y5 = [0,1,1,0,0,0,1,4,8,19,107,626,1686,468,37,3,3,0];
%y6 = [0,1,1,1,1,0,1,0,1,7,21,101,839,1626,362,36,10,0];

%%
%figure('Name','Voltage V')
%tiledlayout(3,3)
%nexttile
%bar(B,y1)
%ylabel('Number of readings:')
%xlabel('Voltage [V]:')
%title('5 min, without water, no tube')

%nexttile
%bar(B,y2)
%ylabel('Number of readings:')
%xlabel('Voltage [V]:')
%title('Blow test, without water, no tube')

%nexttile
%bar(B,y3)
%ylabel('Number of readings:')
%xlabel('Voltage [V]:')
%title('5 min, without water, in tube')

%nexttile
%bar(B,y4)
%ylabel('Number of readings:')
%xlabel('Voltage [V]:')
%title('5 min, movement water, in tube')

%nexttile
%bar(B,y5)
%ylabel('Number of readings:')
%xlabel('Voltage [V]:')
%title('5 min, still water, in tube')

%nexttile
%bar(B,y6)
%ylabel('Number of readings:')
%xlabel('Voltage [V]:')
%title('5 min, Moving water, tube sideways on boat')

%%
%figure("Name",'Pressure kPa')
%tiledlayout(3,3)
%nexttile
%bar(Bp,y1)
%ylabel('Number of readings:')
%xlabel('Pressure [kPa]:')
%title('5 min, without water, no tube')

%nexttile
%bar(Bp,y2)
%ylabel('Number of readings:')
%xlabel('Pressure [kPa]:')
%title('Blow test, without water, no tube')

%nexttile
%bar(Bp,y3)
%ylabel('Number of readings:')
%xlabel('Pressure [kPa]:')
%title('5 min, without water, in tube')

%nexttile
%bar(Bp,y4)
%ylabel('Number of readings:')
%xlabel('Pressure [kPa]:')
%title('5 min, movement water, in tube')

%nexttile
%bar(Bp,y5)
%ylabel('Number of readings:')
%xlabel('Pressure [kPa]:')
%title('5 min, still water, in tube')

%nexttile
%bar(Bp,y6)
%ylabel('Number of readings:')
%xlabel('Pressure [kPa]:')
%title('5 min, Moving water, tube sideways on boat')

%%
%x2 = [0.49199218750000007, 0.49414062500000006, 0.49521484375, 0.49091796875000004, 0.49843750000000003, 0.49736328125000007, 0.49628906250000004, 0.49306640625000003, 0.5005859375, 0.49951171875000006, 0.50166015625, 0.48984375, 0.48876953125000006, 0.5027343750000001, 0.5038085937500001, 0.5081054687500001, 0.5048828125, 0.50703125, 0.48447265625, 0.50595703125, 0.5145507812500001, 0.48339843750000006, 0.48769531250000003, 0.48554687500000004, 0.511328125, 0.511328125, 0.5145507812500001, 0.515625, 0.5134765625000001, 0.51669921875, 0.5188476562500001, 0.5177734375, 0.5263671875, 0.5199218750000001, 0.51025390625, 0.51240234375, 0.52314453125, 0.5091796875000001, 0.50703125, 0.49843750000000003, 0.5081054687500001, 0.49199218750000007, 0.49306640625000003, 0.49843750000000003, 0.49414062500000006, 0.49628906250000004, 0.48876953125000006, 0.49521484375, 0.49736328125000007, 0.49951171875000006, 0.49091796875000004, 0.48984375, 0.5038085937500001, 0.50703125, 0.48125000000000007, 0.5048828125, 0.5134765625000001, 0.5005859375, 0.5145507812500001, 0.5027343750000001, 0.48769531250000003, 0.48232421875000003, 0.50166015625, 0.51025390625, 0.48554687500000004, 0.515625, 0.47802734375000006, 0.511328125, 0.50595703125, 0.5177734375, 0.48339843750000006, 0.49521484375, 0.49306640625000003, 0.49951171875000006, 0.49736328125000007, 0.48984375, 0.49091796875000004, 0.49414062500000006, 0.49199218750000007, 0.49843750000000003, 0.49628906250000004, 0.48876953125000006, 0.5005859375, 0.5048828125, 0.48662109375000007, 0.48769531250000003, 0.50166015625, 0.5027343750000001, 0.48447265625, 0.48554687500000004, 0.5091796875000001, 0.50595703125, 0.48232421875000003, 0.49736328125000007, 0.49306640625000003, 0.49091796875000004, 0.49414062500000006, 0.49199218750000007, 0.49521484375, 0.49628906250000004, 0.49951171875000006, 0.48662109375000007, 0.49843750000000003, 0.5005859375, 0.48984375, 0.5081054687500001, 0.48876953125000006, 0.48554687500000004, 0.48769531250000003, 0.50703125, 0.50166015625, 0.5027343750000001, 0.48017578125000004, 0.48447265625, 0.5038085937500001];
%B2 = sort(x2);
%BD2 = unique(B2(:).')
%y1 = [0,0,0,0,1,1,1,0,1,5,29,103,222,452,650,612,422,234,124,43,36,20,10,6,3,1,2,3,0,0,1,0,0,1,0,0,0,0,0,0,0];
%y2 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,1,9,9,20,19,35,44,25,18,11,4,5,2,1];
%y3 = [1,0,2,1,1,0,2,0,5,24,89,203,397,606,617,477,246,144,69,33,14,5,11,5,4,4,7,0,0,1,1,0,1,1,2,0,1,0,0,0,0];
%y4 = [0,0,0,1,0,1,1,9,16,67,169,374,645,617,550,286,149,66,43,11,4,1,1,0,2,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0];
%y5 = [0,1,0,0,0,1,3,7,13,56,135,342,515,687,582,342,156,83,36,21,8,1,2,1,0,0,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0];

%figure("Name",'Voltage with ref 1.1')
%tiledlayout(3,3)
%nexttile
%bar(BD2,y1)
%ylabel('Number of readings:')
%xlabel('Voltage [V]:')
%title('5 min, without water, no tube')

%nexttile
%bar(BD2,y2)
%ylabel('Number of readings:')
%xlabel('Voltage [V]:')
%title('Blow test, without water, no tube')

%nexttile
%bar(BD2,y3)
%ylabel('Number of readings:')
%xlabel('Voltage [V]:')
%title('5 min, without water, in tube')

%nexttile
%bar(BD2,y4)
%ylabel('Number of readings:')
%xlabel('Voltage [V]:')
%title('5 min, movement water, in tube')

%nexttile
%bar(BD2,y5)
%ylabel('Number of readings:')
%xlabel('Voltage [V]:')
%title('5 min, still water, in tube')

%% Sensor Pressure test
clear all
% Import test results
%Data = importdata('Data.txt');
% Remove earlier tests
%Data(1:307,:) = [];

% Extract important data
%Data_10cm = Data(3:39,:);
%Data_15cm = Data(42:75,:);
%Data_20cm = Data(78:112,:);
%Data_25cm = Data(115:150,:);
%Data_30cm = Data(153:189,:);

% Extract pressures


% Different pressure readings
P_Manual = [0.5371093749999972, 0.8056640625000028, 1.0742187500000084, 1.3427734375, 1.6113281250000056, 1.8798828124999973, 2.1484375000000027, 2.4169921875000084, 2.685546875, 2.9541015625000053, 3.2226562499999973, 3.4912109375000027, 3.7597656250000084, 4.0283203125, 4.296875000000005, 4.565429687499997, 4.833984375000003, 5.102539062500008, 5.37109375, 5.639648437500005, 5.908203124999997, 6.176757812500003, 6.445312500000008, 6.713867187500014, 6.982421874999992, 7.250976562499997, 7.519531250000003, 7.788085937500008, 8.056640625000014, 8.325195312499991, 8.593749999999996, 8.862304687500004, 9.130859375000009, 9.399414062500014, 9.667968749999991, 9.936523437499996, 10.205078125000004, 10.473632812500009, 11.010742187499991, 11.279296874999996, 11.816406250000009, 12.353515624999991];

% 10 cm readings
S1_10cm = [0, 0, 1, 1, 1, 1, 4, 1, 1, 5, 9, 15, 57, 111, 366, 749, 1145, 1102, 786, 296, 153, 72, 42, 28, 13, 7, 8, 4, 2, 2, 2, 3, 3, 2, 1, 4, 1, 2, 0, 0, 0, 0];
S2_10cm = [0, 0, 1, 0, 1, 0, 2, 0, 2, 10, 17, 27, 126, 285, 693, 1098, 1141, 798, 415, 165, 78, 49, 31, 21, 9, 8, 4, 6, 3, 3, 0, 1, 2, 0, 1, 2, 1, 0, 0, 0, 0, 0];
S3_10cm = [0,0,2,2,1,0,1,2,3,4,6,16,36,123,287,684,1032,1008,870,452,220,100,55,34,21,9,5,6,4,5,2,1,1,2,4,1,0,0,1,0,0,0];

% 15 cm readings
S1_15cm = [0,0,0,0,0,0,3,1,4,2,7,14,25,84,244,533,890,1017,940,496,321,194,99,37,29,18,15,5,5,5,3,3,1,3,1,0,1,0,0,0,0,0];
S2_15cm = [0,0,0,1,0,1,3,0,3,5,6,15,36,89,262,533,913,985,933,532,314,131,79,53,18,20,21,6,12,9,3,6,2,3,3,1,1,1,0,0,0,0];
S3_15cm = [0,0,0,1,0,1,1,1,1,2,2,11,19,41,169,441,820,1041,1010,644,377,167,94,48,35,28,8,6,8,6,2,2,1,2,5,1,4,1,0,0,0,0];

% 20 cm readings
S1_20cm = [0,0,0,0,1,1,3,4,4,6,9,26,68,224,592,957,1133,886,549,263,107,62,39,23,15,15,6,1,1,2,1,0,0,1,0,1,0,0,0,0,0,0];
S2_20cm = [0,0,0,0,0,0,2,3,1,1,2,5,5,10,41,88,260,559,1119,1077,784,437,238,125,69,44,29,17,13,14,18,11,11,5,4,3,3,1,0,1,0,0];
S3_20cm = [0,0,0,0,1,2,3,1,1,4,6,11,22,67,176,521,935,1118,993,516,277,126,81,39,23,22,16,8,7,2,7,2,5,2,2,2,2,0,0,0,0,0];

% 25 cm readings
S1_25cm = [0,0,1,0,2,4,2,4,1,5,14,40,124,321,699,1076,1152,741,438,163,92,44,27,17,10,7,6,5,2,1,0,1,1,0,0,0,0,0,0,0,0,0];
S2_25cm = [0,0,0,1,0,2,1,1,3,6,10,12,28,92,274,566,775,916,881,605,381,190,104,49,39,23,12,6,10,4,0,1,2,1,1,2,0,0,0,0,1,1];
S3_25cm = [1,0,0,0,3,2,2,5,3,13,9,44,106,291,660,975,1060,845,506,208,122,52,28,22,12,10,3,9,3,3,0,1,1,0,0,1,0,0,0,0,0,0];

% 30 cm readings
S1_30cm = [0,0,2,0,1,2,1,2,5,6,19,45,135,345,742,1034,1061,724,445,180,95,53,32,15,14,12,9,7,4,3,3,1,0,0,2,0,1,0,0,0,0,0];
S2_30cm = [0,0,1,3,2,3,5,1,6,7,8,32,80,211,540,954,1151,892,561,249,99,58,36,25,20,12,14,6,6,7,2,3,1,1,2,0,1,1,0,0,0,0];
S3_30cm = [0,2,2,0,0,0,4,5,10,11,21,70,221,527,1050,1169,872,532,238,100,62,27,17,16,12,7,5,5,2,5,0,3,0,1,3,1,0,0,0,0,0,0];

% Define depht array
Depht = [10, 15, 20, 25, 30];

S1_Data = [S1_10cm(:), S1_15cm(:), S1_20cm(:), S1_25cm(:), S1_30cm(:)];
S2_Data = [S2_10cm(:), S2_15cm(:), S2_20cm(:), S2_25cm(:), S2_30cm(:)];
S3_Data = [S3_10cm(:), S3_15cm(:), S3_20cm(:), S3_25cm(:), S3_30cm(:)];



% ---Plots---
figure("Name",'Pressure with ref 1.1V 2D');
Tiles = tiledlayout(3,1,'TileSpacing','Compact','Padding','Compact');

% S1 plots
nexttile;
S1_Bar = bar(P_Manual,S1_Data, 'stacked');
% Set colour
set(S1_Bar(1), 'FaceColor',[0.6 0.8 0.9]);
set(S1_Bar(2), 'FaceColor',[0.4 0.6 0.9]);
set(S1_Bar(3), 'FaceColor',[0 0 0.7]);
set(S1_Bar(4), 'FaceColor',[0.1 0.4 0.6]);
set(S1_Bar(5), 'FaceColor',[0.5 0.55 0.7]);
legend('Depht: 10cm','Depht: 15cm','Depht: 20cm','Depht: 25cm','Depht: 30cm');
ylabel('Number of readings:');
xlabel('Pressure [Kpa]:');
title('5000 samples: Sensor 1 in 2D');

% S2 plots
nexttile;
S2_Bar = bar(P_Manual,S2_Data, 'stacked');
% Set colour
set(S2_Bar(1), 'FaceColor',[0.6 0.8 0.9]);
set(S2_Bar(2), 'FaceColor',[0.4 0.6 0.9]);
set(S2_Bar(3), 'FaceColor',[0 0 0.7]);
set(S2_Bar(4), 'FaceColor',[0.1 0.4 0.6]);
set(S2_Bar(5), 'FaceColor',[0.5 0.55 0.7]);
legend('Depht: 10cm','Depht: 15cm','Depht: 20cm','Depht: 25cm','Depht: 30cm');
ylabel('Number of readings:');
xlabel('Pressure [Kpa]:');
title('5000 samples: Sensor 2 in 2D');

% S3 plots
nexttile;
S3_Bar = bar(P_Manual,S3_Data, 'stacked');
% Set colour
set(S3_Bar(1), 'FaceColor',[0.6 0.8 0.9]);
set(S3_Bar(2), 'FaceColor',[0.4 0.6 0.9]);
set(S3_Bar(3), 'FaceColor',[0 0 0.7]);
set(S3_Bar(4), 'FaceColor',[0.1 0.4 0.6]);
set(S3_Bar(5), 'FaceColor',[0.5 0.55 0.7]);
legend('Depht: 10cm','Depht: 15cm','Depht: 20cm','Depht: 25cm','Depht: 30cm');
ylabel('Number of readings:');
xlabel('Pressure [Kpa]:');
title('5000 samples: Sensor 3 in 2D');

t.TileSpacing = 'compact';
t.Padding = 'compact';

figure("Name",'Pressure with ref 1.1V 3D');
Tiles = tiledlayout(1,3,'TileSpacing','Compact','Padding','Compact');

nexttile;
S1_Bar3 = bar3(P_Manual,S1_Data);
% Set colour
set(gca,'XTickLabel',[10 15 20 25 30]);
set(S1_Bar3(1), 'FaceColor',[0.6 0.8 0.9]);
set(S1_Bar3(2), 'FaceColor',[0.4 0.6 0.9]);
set(S1_Bar3(3), 'FaceColor',[0 0 0.7]);
set(S1_Bar3(4), 'FaceColor',[0.1 0.4 0.6]);
set(S1_Bar3(5), 'FaceColor',[0.5 0.55 0.7]);
zlabel('Number of readings:');
ylabel('Pressure [Kpa]:');
xlabel('Depht [cm]:');
legend('Depht: 10cm','Depht: 15cm','Depht: 20cm','Depht: 25cm','Depht: 30cm');
title('5000 samples: Sensor 1 in 3D');

nexttile;
S2_Bar3 = bar3(P_Manual,S2_Data);
% Set colour
set(gca,'XTickLabel',[10 15 20 25 30]);
set(S2_Bar3(1), 'FaceColor',[0.6 0.8 0.9]);
set(S2_Bar3(2), 'FaceColor',[0.4 0.6 0.9]);
set(S2_Bar3(3), 'FaceColor',[0 0 0.7]);
set(S2_Bar3(4), 'FaceColor',[0.1 0.4 0.6]);
set(S2_Bar3(5), 'FaceColor',[0.5 0.55 0.7]);
zlabel('Number of readings:');
ylabel('Pressure [Kpa]:');
xlabel('Depht [cm]:');
title('5000 samples: Sensor 2 in 3D');

nexttile;
S3_Bar3 = bar3(P_Manual,S3_Data);
% Set colour
set(gca,'XTickLabel',[10 15 20 25 30]);
set(S3_Bar3(1), 'FaceColor',[0.6 0.8 0.9]);
set(S3_Bar3(2), 'FaceColor',[0.4 0.6 0.9]);
set(S3_Bar3(3), 'FaceColor',[0 0 0.7]);
set(S3_Bar3(4), 'FaceColor',[0.1 0.4 0.6]);
set(S3_Bar3(5), 'FaceColor',[0.5 0.55 0.7]);
zlabel('Number of readings:');
ylabel('Pressure [Kpa]:');
xlabel('Depht [cm]:');
title('5000 samples: Sensor 3 in 3D');