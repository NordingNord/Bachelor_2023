clear all 
%% Get Data ready
% Total Pressure data:
P = [0.09375000000000355, 0.14062500000000533, 0.18749999999999323, 0.234374999999995, 0.2812499999999968, 0.32812499999999856, 0.37500000000000033, 0.42187499999998823, 0.46874999999999, 0.5156249999999918, 0.5624999999999936, 0.6093749999999953, 0.6562499999999971, 0.7031249999999989, 0.7500000000000007, 0.7968750000000024, 0.8437500000000042, 0.890625000000006, 0.9375000000000078, 0.9843750000000095, 1.0312500000000113, 1.078125000000013, 1.1249999999999871, 1.171874999999989, 1.2187499999999907, 0.234374999999995, 0.2812499999999968, 0.32812499999999856, 0.37500000000000033, 0.42187499999998823, 0.46874999999999, 0.5156249999999918, 0.5624999999999936, 0.6093749999999953, 0.6562499999999971, 0.7031249999999989, 0.7500000000000007, 0.7968750000000024, 0.8437500000000042, 0.890625000000006, 0.9375000000000078, 0.9843750000000095, 1.0312500000000113, 1.078125000000013, 1.1249999999999871, 1.171874999999989, 1.2187499999999907, 1.2656249999999925, 1.3124999999999942, 1.359374999999996, 1.4062499999999978, 1.4531249999999996, 1.5000000000000013, 0.32812499999999856, 0.37500000000000033, 0.42187499999998823, 0.46874999999999, 0.5156249999999918, 0.5624999999999936, 0.6093749999999953, 0.6562499999999971, 0.7031249999999989, 0.7500000000000007, 0.7968750000000024, 0.8437500000000042, 0.890625000000006, 0.9375000000000078, 0.9843750000000095, 1.0312500000000113, 1.078125000000013, 1.1249999999999871, 1.171874999999989, 1.2187499999999907, 1.2656249999999925, 1.3124999999999942, 1.359374999999996, 1.4062499999999978, 1.4531249999999996, 1.5000000000000013, 1.546875000000003, 0.2812499999999968, 0.32812499999999856, 0.37500000000000033, 0.42187499999998823, 0.46874999999999, 0.5156249999999918, 0.5624999999999936, 0.6093749999999953, 0.6562499999999971, 0.7031249999999989, 0.7500000000000007, 0.7968750000000024, 0.8437500000000042, 0.890625000000006, 0.9375000000000078, 0.9843750000000095, 1.0312500000000113, 1.078125000000013, 1.1249999999999871, 1.171874999999989, 1.2187499999999907, 1.2656249999999925, 1.3124999999999942, 1.359374999999996, 1.4062499999999978, 1.4531249999999996, 1.5000000000000013, 1.546875000000003, 0.234374999999995, 0.2812499999999968, 0.32812499999999856, 0.37500000000000033, 0.42187499999998823, 0.46874999999999, 0.5156249999999918, 0.5624999999999936, 0.6093749999999953, 0.6562499999999971, 0.7031249999999989, 0.7500000000000007, 0.7968750000000024, 0.8437500000000042, 0.890625000000006, 0.9375000000000078, 0.9843750000000095, 1.0312500000000113, 1.078125000000013, 1.1249999999999871, 1.171874999999989, 1.2187499999999907, 1.2656249999999925, 1.3124999999999942, 1.359374999999996, 1.4062499999999978, 1.4531249999999996, 1.5000000000000013];
P = sort(P);
P = unique(P(:).');

% Individual test pressure data
P_10cm = [0.09375000000000355, 0.14062500000000533, 0.18749999999999323, 0.234374999999995, 0.2812499999999968, 0.32812499999999856, 0.37500000000000033, 0.42187499999998823, 0.46874999999999, 0.5156249999999918, 0.5624999999999936, 0.6093749999999953, 0.6562499999999971, 0.7031249999999989, 0.7500000000000007, 0.7968750000000024, 0.8437500000000042, 0.890625000000006, 0.9375000000000078, 0.9843750000000095, 1.0312500000000113, 1.078125000000013, 1.1249999999999871, 1.171874999999989, 1.2187499999999907];
P_15cm = [0.234374999999995, 0.2812499999999968, 0.32812499999999856, 0.37500000000000033, 0.42187499999998823, 0.46874999999999, 0.5156249999999918, 0.5624999999999936, 0.6093749999999953, 0.6562499999999971, 0.7031249999999989, 0.7500000000000007, 0.7968750000000024, 0.8437500000000042, 0.890625000000006, 0.9375000000000078, 0.9843750000000095, 1.0312500000000113, 1.078125000000013, 1.1249999999999871, 1.171874999999989, 1.2187499999999907, 1.2656249999999925, 1.3124999999999942, 1.359374999999996, 1.4062499999999978, 1.4531249999999996, 1.5000000000000013];
P_20cm = [0.32812499999999856, 0.37500000000000033, 0.42187499999998823, 0.46874999999999, 0.5156249999999918, 0.5624999999999936, 0.6093749999999953, 0.6562499999999971, 0.7031249999999989, 0.7500000000000007, 0.7968750000000024, 0.8437500000000042, 0.890625000000006, 0.9375000000000078, 0.9843750000000095, 1.0312500000000113, 1.078125000000013, 1.1249999999999871, 1.171874999999989, 1.2187499999999907, 1.2656249999999925, 1.3124999999999942, 1.359374999999996, 1.4062499999999978, 1.4531249999999996, 1.5000000000000013, 1.546875000000003];
P_25cm = [0.2812499999999968, 0.32812499999999856, 0.37500000000000033, 0.42187499999998823, 0.46874999999999, 0.5156249999999918, 0.5624999999999936, 0.6093749999999953, 0.6562499999999971, 0.7031249999999989, 0.7500000000000007, 0.7968750000000024, 0.8437500000000042, 0.890625000000006, 0.9375000000000078, 0.9843750000000095, 1.0312500000000113, 1.078125000000013, 1.1249999999999871, 1.171874999999989, 1.2187499999999907, 1.2656249999999925, 1.3124999999999942, 1.359374999999996, 1.4062499999999978, 1.4531249999999996, 1.5000000000000013, 1.546875000000003];
P_30cm = [0.234374999999995, 0.2812499999999968, 0.32812499999999856, 0.37500000000000033, 0.42187499999998823, 0.46874999999999, 0.5156249999999918, 0.5624999999999936, 0.6093749999999953, 0.6562499999999971, 0.7031249999999989, 0.7500000000000007, 0.7968750000000024, 0.8437500000000042, 0.890625000000006, 0.9375000000000078, 0.9843750000000095, 1.0312500000000113, 1.078125000000013, 1.1249999999999871, 1.171874999999989, 1.2187499999999907, 1.2656249999999925, 1.3124999999999942, 1.359374999999996, 1.4062499999999978, 1.4531249999999996, 1.5000000000000013];

% S1 Data:
S1_10cm = [0,0,0,0,2,1,3,5,13,26,41,69,138,184,253,356,361,243,159,85,33,17,2,8,1];
S1_15cm = [0,0,0,0,0,0,1,4,3,13,22,22,60,84,118,144,219,212,257,248,185,154,110,83,34,18,8,1];
S1_20cm = [0,0,1,0,1,2,4,5,11,14,32,52,82,114,182,158,239,256,212,210,158,131,67,40,19,7,3];
S1_25cm = [0,0,0,0,0,1,0,0,4,9,7,14,32,55,75,139,158,215,224,255,245,195,147,96,80,37,6,6];
S1_30cm = [0,0,0,0,0,1,2,6,9,15,32,53,86,116,129,199,233,221,245,199,158,119,71,54,25,15,6,6];

% S2 Data:
S2_10cm = [1,1,1,4,5,14,20,37,78,137,205,212,286,308,239,187,125,77,37,16,8,2,0,0,0];
S2_15cm = [0,1,3,2,4,11,28,47,67,112,173,200,270,291,245,244,161,78,41,18,2,2,0,0,0,0,0,0];
S2_20cm = [1,3,9,11,26,47,82,111,160,184,279,269,265,225,170,94,48,9,4,2,1,0,0,0,0,0,0];
S2_25cm = [0,1,3,2,9,20,18,44,69,134,145,220,301,293,272,207,124,77,42,9,9,0,1,0,0,0,0,0];
S2_30cm = [0,2,2,5,7,20,43,73,76,134,209,215,245,290,230,215,121,69,33,6,5,0,0,0,0,0,0,0];

% S3 Data:
S3_10cm = [0,0,1,7,10,13,37,60,131,176,232,289,318,258,174,124,65,51,38,8,3,4,1,0,0];
S3_15cm = [1,0,2,2,1,9,28,52,81,97,180,192,236,250,225,212,174,110,78,35,29,3,2,1,0,0,0,0];
S3_20cm = [3,5,6,18,27,54,82,122,179,212,246,230,221,197,178,104,61,34,13,7,0,1,0,0,0,0,0];
S3_25cm = [2,3,2,11,13,33,53,83,130,180,193,244,265,233,192,160,100,54,35,9,2,3,0,0,0,0,0,0];
S3_30cm = [1,0,5,7,10,21,42,67,87,165,206,208,248,249,233,178,123,77,47,14,5,4,3,0,0,0,0,0];

% Update Data 10cm:
i = 1;
while ~isequal(P(i),P_10cm(1))
    S1_10cm = [0, S1_10cm(1:end)];
    S2_10cm = [0, S2_10cm(1:end)];
    S3_10cm = [0, S3_10cm(1:end)];
    i = i+1;
end
P_10cm = [P(1:i-1),P_10cm(1:end)];
i = length(P);
while ~isequal(P(i),P_10cm(end))
    S1_10cm = [S1_10cm(1:end),0];
    S2_10cm = [S2_10cm(1:end),0];
    S3_10cm = [S3_10cm(1:end),0];
    i = i-1;
end
P_10cm = [P_10cm(1:end),P(i+1:end)];
while ~isequal(length(P_10cm),length(P))
    for j = 1:length(P_10cm)
        if ~isequal(P_10cm(j),P(j))
            S1_10cm = [S1_10cm(1:j-1),0,S1_10cm(j:end)];
            S2_10cm = [S2_10cm(1:j-1),0,S2_10cm(j:end)];
            S3_10cm = [S3_10cm(1:j-1),0,S3_10cm(j:end)];
            P_10cm = [P_10cm(1:j-1),P(j),P_10cm(j:end)];
            break;
        end
    end
end

% Update Data 15cm:
i = 1;
while ~isequal(P(i),P_15cm(1))
    S1_15cm = [0, S1_15cm(1:end)];
    S2_15cm = [0, S2_15cm(1:end)];
    S3_15cm = [0, S3_15cm(1:end)];
    i = i+1;
end
P_15cm = [P(1:i-1),P_15cm(1:end)];
i = length(P);
while ~isequal(P(i),P_15cm(end))
    S1_15cm = [S1_15cm(1:end),0];
    S2_15cm = [S2_15cm(1:end),0];
    S3_15cm = [S3_15cm(1:end),0];
    i = i-1;
end
P_15cm = [P_15cm(1:end),P(i+1:end)];
while ~isequal(length(P_15cm),length(P))
    for j = 1:length(P_15cm)
        if ~isequal(P_15cm(j),P(j))
            S1_15cm = [S1_15cm(1:j-1),0,S1_15cm(j:end)];
            S2_15cm = [S2_15cm(1:j-1),0,S2_15cm(j:end)];
            S3_15cm = [S3_15cm(1:j-1),0,S3_15cm(j:end)];
            P_15cm = [P_15cm(1:j-1),P(j),P_15cm(j:end)];
            break;
        end
    end
end

% Update Data 20cm:
i = 1;
while ~isequal(P(i),P_20cm(1))
    S1_20cm = [0, S1_20cm(1:end)];
    S2_20cm = [0, S2_20cm(1:end)];
    S3_20cm = [0, S3_20cm(1:end)];
    i = i+1;
end
P_20cm = [P(1:i-1),P_20cm(1:end)];
i = length(P);
while ~isequal(P(i),P_20cm(end))
    S1_20cm = [S1_20cm(1:end),0];
    S2_20cm = [S2_20cm(1:end),0];
    S3_20cm = [S3_20cm(1:end),0];
    i = i-1;
end
P_20cm = [P_20cm(1:end),P(i+1:end)];
while ~isequal(length(P_20cm),length(P))
    for j = 1:length(P_20cm)
        if ~isequal(P_20cm(j),P(j))
            S1_20cm = [S1_20cm(1:j-1),0,S1_20cm(j:end)];
            S2_20cm = [S2_20cm(1:j-1),0,S2_20cm(j:end)];
            S3_20cm = [S3_20cm(1:j-1),0,S3_20cm(j:end)];
            P_20cm = [P_20cm(1:j-1),P(j),P_20cm(j:end)];
            break;
        end
    end
end

% Update Data 25cm:
i = 1;
while ~isequal(P(i),P_25cm(1))
    S1_25cm = [0, S1_25cm(1:end)];
    S2_25cm = [0, S2_25cm(1:end)];
    S3_25cm = [0, S3_25cm(1:end)];
    i = i+1;
end
P_25cm = [P(1:i-1),P_25cm(1:end)];
i = length(P);
while ~isequal(P(i),P_25cm(end))
    S1_25cm = [S1_25cm(1:end),0];
    S2_25cm = [S2_25cm(1:end),0];
    S3_25cm = [S3_25cm(1:end),0];
    i = i-1;
end
P_25cm = [P_25cm(1:end),P(i+1:end)];
while ~isequal(length(P_25cm),length(P))
    for j = 1:length(P_25cm)
        if ~isequal(P_25cm(j),P(j))
            S1_25cm = [S1_25cm(1:j-1),0,S1_25cm(j:end)];
            S2_25cm = [S2_25cm(1:j-1),0,S2_25cm(j:end)];
            S3_25cm = [S3_25cm(1:j-1),0,S3_25cm(j:end)];
            P_25cm = [P_25cm(1:j-1),P(j),P_25cm(j:end)];
            break;
        end
    end
end

% Update Data 30cm:
i = 1;
while ~isequal(P(i),P_30cm(1))
    S1_30cm = [0, S1_30cm(1:end)];
    S2_30cm = [0, S2_30cm(1:end)];
    S3_30cm = [0, S3_30cm(1:end)];
    i = i+1;
end
P_30cm = [P(1:i-1),P_30cm(1:end)];
i = length(P);
while ~isequal(P(i),P_30cm(end))
    S1_30cm = [S1_30cm(1:end),0];
    S2_30cm = [S2_30cm(1:end),0];
    S3_30cm = [S3_30cm(1:end),0];
    i = i-1;
end
P_30cm = [P_30cm(1:end),P(i+1:end)];
while ~isequal(length(P_30cm),length(P))
    for j = 1:length(P_30cm)
        if ~isequal(P_30cm(j),P(j))
            S1_30cm = [S1_30cm(1:j-1),0,S1_30cm(j:end)];
            S2_30cm = [S2_30cm(1:j-1),0,S2_30cm(j:end)];
            S3_30cm = [S3_30cm(1:j-1),0,S3_30cm(j:end)];
            P_30cm = [P_30cm(1:j-1),P(j),P_30cm(j:end)];
            break;
        end
    end
end

% Initialise the vectors i will be working with
Median_Vector_S1_10cm = [];
Median_Vector_S1_15cm = [];
Median_Vector_S1_20cm = [];
Median_Vector_S1_25cm = [];
Median_Vector_S1_30cm = [];

Median_Vector_S2_10cm = [];
Median_Vector_S2_15cm = [];
Median_Vector_S2_20cm = [];
Median_Vector_S2_25cm = [];
Median_Vector_S2_30cm = [];

Median_Vector_S3_10cm = [];
Median_Vector_S3_15cm = [];
Median_Vector_S3_20cm = [];
Median_Vector_S3_25cm = [];
Median_Vector_S3_30cm = [];

Median_Vector_S1_10cm_Cut = [];
Median_Vector_S1_15cm_Cut = [];
Median_Vector_S1_20cm_Cut = [];
Median_Vector_S1_25cm_Cut = [];
Median_Vector_S1_30cm_Cut = [];

Median_Vector_S2_10cm_Cut = [];
Median_Vector_S2_15cm_Cut = [];
Median_Vector_S2_20cm_Cut = [];
Median_Vector_S2_25cm_Cut = [];
Median_Vector_S2_30cm_Cut = [];

Median_Vector_S3_10cm_Cut = [];
Median_Vector_S3_15cm_Cut = [];
Median_Vector_S3_20cm_Cut = [];
Median_Vector_S3_25cm_Cut = [];
Median_Vector_S3_30cm_Cut = [];

for i = 1:length(P)
    Median_Vector_S1_10cm = [Median_Vector_S1_10cm(1:end), zeros(1,S1_10cm(i))+P(i)];
    Median_Vector_S1_15cm = [Median_Vector_S1_15cm(1:end), zeros(1,S1_15cm(i))+P(i)];
    Median_Vector_S1_20cm = [Median_Vector_S1_20cm(1:end), zeros(1,S1_20cm(i))+P(i)];
    Median_Vector_S1_25cm = [Median_Vector_S1_25cm(1:end), zeros(1,S1_25cm(i))+P(i)];
    Median_Vector_S1_30cm = [Median_Vector_S1_30cm(1:end), zeros(1,S1_30cm(i))+P(i)];

    Median_Vector_S2_10cm = [Median_Vector_S2_10cm(1:end), zeros(1,S2_10cm(i))+P(i)];
    Median_Vector_S2_15cm = [Median_Vector_S2_15cm(1:end), zeros(1,S2_15cm(i))+P(i)];
    Median_Vector_S2_20cm = [Median_Vector_S2_20cm(1:end), zeros(1,S2_20cm(i))+P(i)];
    Median_Vector_S2_25cm = [Median_Vector_S2_25cm(1:end), zeros(1,S2_25cm(i))+P(i)];
    Median_Vector_S2_30cm = [Median_Vector_S2_30cm(1:end), zeros(1,S2_30cm(i))+P(i)];

    Median_Vector_S3_10cm = [Median_Vector_S3_10cm(1:end), zeros(1,S3_10cm(i))+P(i)];
    Median_Vector_S3_15cm = [Median_Vector_S3_15cm(1:end), zeros(1,S3_15cm(i))+P(i)];
    Median_Vector_S3_20cm = [Median_Vector_S3_20cm(1:end), zeros(1,S3_20cm(i))+P(i)];
    Median_Vector_S3_25cm = [Median_Vector_S3_25cm(1:end), zeros(1,S3_25cm(i))+P(i)];
    Median_Vector_S3_30cm = [Median_Vector_S3_30cm(1:end), zeros(1,S3_30cm(i))+P(i)];
    
    if S1_10cm(i) >= 10
        Median_Vector_S1_10cm_Cut = [Median_Vector_S1_10cm_Cut(1:end), zeros(1,S1_10cm(i))+P(i)];
    else
        Median_Vector_S1_10cm_Cut = [Median_Vector_S1_10cm_Cut(1:end), zeros(1,S1_10cm(i))+NaN];
    end
    if S1_15cm(i) >= 10
        Median_Vector_S1_15cm_Cut = [Median_Vector_S1_15cm_Cut(1:end), zeros(1,S1_15cm(i))+P(i)];
    else
        Median_Vector_S1_15cm_Cut = [Median_Vector_S1_15cm_Cut(1:end), zeros(1,S1_15cm(i))+NaN];
    end
    if S1_20cm(i) >= 10
        Median_Vector_S1_20cm_Cut = [Median_Vector_S1_20cm_Cut(1:end), zeros(1,S1_20cm(i))+P(i)];
    else
        Median_Vector_S1_20cm_Cut = [Median_Vector_S1_20cm_Cut(1:end), zeros(1,S1_20cm(i))+NaN];
    end
    if S1_25cm(i) >= 10
        Median_Vector_S1_25cm_Cut = [Median_Vector_S1_25cm_Cut(1:end), zeros(1,S1_25cm(i))+P(i)];
    else
        Median_Vector_S1_25cm_Cut = [Median_Vector_S1_25cm_Cut(1:end), zeros(1,S1_25cm(i))+NaN];
    end
    if S1_30cm(i) >= 10
        Median_Vector_S1_30cm_Cut = [Median_Vector_S1_30cm_Cut(1:end), zeros(1,S1_30cm(i))+P(i)];
    else
        Median_Vector_S1_30cm_Cut = [Median_Vector_S1_30cm_Cut(1:end), zeros(1,S1_30cm(i))+NaN];
    end
    
    if S2_10cm(i) >= 10
        Median_Vector_S2_10cm_Cut = [Median_Vector_S2_10cm_Cut(1:end), zeros(1,S2_10cm(i))+P(i)];
    else
        Median_Vector_S2_10cm_Cut = [Median_Vector_S2_10cm_Cut(1:end), zeros(1,S2_10cm(i))+NaN];
    end
    if S2_15cm(i) >= 10
        Median_Vector_S2_15cm_Cut = [Median_Vector_S2_15cm_Cut(1:end), zeros(1,S2_15cm(i))+P(i)];
    else
        Median_Vector_S2_15cm_Cut = [Median_Vector_S2_15cm_Cut(1:end), zeros(1,S2_15cm(i))+NaN];
    end
    if S2_20cm(i) >= 10
        Median_Vector_S2_20cm_Cut = [Median_Vector_S2_20cm_Cut(1:end), zeros(1,S2_20cm(i))+P(i)];
    else
        Median_Vector_S2_20cm_Cut = [Median_Vector_S2_20cm_Cut(1:end), zeros(1,S2_20cm(i))+NaN];
    end
    if S2_25cm(i) >= 10
        Median_Vector_S2_25cm_Cut = [Median_Vector_S2_25cm_Cut(1:end), zeros(1,S2_25cm(i))+P(i)];
    else
        Median_Vector_S2_25cm_Cut = [Median_Vector_S2_25cm_Cut(1:end), zeros(1,S2_25cm(i))+NaN]; 
    end
    if S2_30cm(i) >= 10
        Median_Vector_S2_30cm_Cut = [Median_Vector_S2_30cm_Cut(1:end), zeros(1,S2_30cm(i))+P(i)];
    else
        Median_Vector_S2_30cm_Cut = [Median_Vector_S2_30cm_Cut(1:end), zeros(1,S2_30cm(i))+NaN];
    end
    
    if S3_10cm(i) >= 10
        Median_Vector_S3_10cm_Cut = [Median_Vector_S3_10cm_Cut(1:end), zeros(1,S3_10cm(i))+P(i)];
    else
        Median_Vector_S3_10cm_Cut = [Median_Vector_S3_10cm_Cut(1:end), zeros(1,S3_10cm(i))+NaN];
    end
    if S3_15cm(i) >= 10
        Median_Vector_S3_15cm_Cut = [Median_Vector_S3_15cm_Cut(1:end), zeros(1,S3_15cm(i))+P(i)];
    else
        Median_Vector_S3_15cm_Cut = [Median_Vector_S3_15cm_Cut(1:end), zeros(1,S3_15cm(i))+NaN];
    end
    if S3_20cm(i) >= 10
        Median_Vector_S3_20cm_Cut = [Median_Vector_S3_20cm_Cut(1:end), zeros(1,S3_20cm(i))+P(i)];
    else
        Median_Vector_S3_20cm_Cut = [Median_Vector_S3_20cm_Cut(1:end), zeros(1,S3_20cm(i))+NaN];
    end
    if S3_25cm(i) >= 10
        Median_Vector_S3_25cm_Cut = [Median_Vector_S3_25cm_Cut(1:end), zeros(1,S3_25cm(i))+P(i)];
    else
        Median_Vector_S3_25cm_Cut = [Median_Vector_S3_25cm_Cut(1:end), zeros(1,S3_25cm(i))+NaN];
    end
    if S3_30cm(i) >= 10
        Median_Vector_S3_30cm_Cut = [Median_Vector_S3_30cm_Cut(1:end), zeros(1,S3_30cm(i))+P(i)];
    else
       Median_Vector_S3_30cm_Cut = [Median_Vector_S3_30cm_Cut(1:end), zeros(1,S3_30cm(i))+NaN]; 
    end   
end

% Boxplot fun
S1_Box_Data = [Median_Vector_S1_10cm.' Median_Vector_S1_15cm.' Median_Vector_S1_20cm.' Median_Vector_S1_25cm.' Median_Vector_S1_30cm.'];
S2_Box_Data = [Median_Vector_S2_10cm.' Median_Vector_S2_15cm.' Median_Vector_S2_20cm.' Median_Vector_S2_25cm.' Median_Vector_S2_30cm.'];
S3_Box_Data = [Median_Vector_S3_10cm.' Median_Vector_S3_15cm.' Median_Vector_S3_20cm.' Median_Vector_S3_25cm.' Median_Vector_S3_30cm.'];

S1_Box_Data_Cut = [Median_Vector_S1_10cm_Cut.' Median_Vector_S1_15cm_Cut.' Median_Vector_S1_20cm_Cut.' Median_Vector_S1_25cm_Cut.' Median_Vector_S1_30cm_Cut.'];
S2_Box_Data_Cut = [Median_Vector_S2_10cm_Cut.' Median_Vector_S2_15cm_Cut.' Median_Vector_S2_20cm_Cut.' Median_Vector_S2_25cm_Cut.' Median_Vector_S2_30cm_Cut.'];
S3_Box_Data_Cut = [Median_Vector_S3_10cm_Cut.' Median_Vector_S3_15cm_Cut.' Median_Vector_S3_20cm_Cut.' Median_Vector_S3_25cm_Cut.' Median_Vector_S3_30cm_Cut.'];

%% Boxplots

figure("Name",'Pressure with ref 1.1V Boxplot');
tiledlayout(3,1,'TileSpacing','Compact','Padding','Compact');

nexttile
S1_B = boxplot(S1_Box_Data,'Notch','on','Labels',{'10cm','15cm','20cm','25cm','30cm'});
title("Pressure by depth S1")
xlabel("Depth [cm]:")
ylabel("Pressure [Kpa]:")

nexttile
S2_B = boxplot(S2_Box_Data,'Notch','on','Labels',{'10cm','15cm','20cm','25cm','30cm'});
title("Pressure by depth S2")
xlabel("Depth [cm]:")
ylabel("Pressure [Kpa]:")

nexttile
S3_B = boxplot(S3_Box_Data,'Notch','on','Labels',{'10cm','15cm','20cm','25cm','30cm'});
title("Pressure by depth S3")
xlabel("Depth [cm]:")
ylabel("Pressure [Kpa]:")

print -deps New_16_test


% Cut version
figure("Name",'Pressure with ref 1.1V Boxplot Cut');
tiledlayout(3,1,'TileSpacing','Compact','Padding','Compact');

nexttile
S1_BC = boxplot(S1_Box_Data_Cut,'Notch','on','Labels',{'10cm','15cm','20cm','25cm','30cm'});
title("Pressure by depth S1")
xlabel("Depth [cm]:")
ylabel("Pressure [Kpa]:")

nexttile
S2_BC = boxplot(S2_Box_Data_Cut,'Notch','on','Labels',{'10cm','15cm','20cm','25cm','30cm'});
title("Pressure by depth S2")
xlabel("Depth [cm]:")
ylabel("Pressure [Kpa]:")

nexttile
S3_BC = boxplot(S3_Box_Data_Cut,'Notch','on','Labels',{'10cm','15cm','20cm','25cm','30cm'});
title("Pressure by depth S3")
xlabel("Depth [cm]:")
ylabel("Pressure [Kpa]:")