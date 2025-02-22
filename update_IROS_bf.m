clear, clc 
close all
load finalESTIMATION.mat
load NodatML.mat
%% organize index
% INDEX
exp1_claib1_i = length(data_Sync1);
exp1_claib2_i = length(data_Sync2) + exp1_claib1_i;
exp1_claib3_i = length(data_Sync3) + exp1_claib2_i;     

exp1_claib5_i = length(data_Sync5) + exp1_claib3_i;     % gentle mood
exp1_claib6_i = length(data_Sync6) + exp1_claib5_i;     % mormal hand manipulation (bar) --- **** exp1_claib5_i - exp1_claib6_i

exp1_claib8_i = length(data_Sync8) + exp1_claib6_i;     % well done by manipulation 
exp1_claib9_i = length(data_Sync9) + exp1_claib8_i;     % moving actuator no loading -------**** exp1_claib8_i - exp1_claib9_i

exp1_claib10_i = length(data_Sync10) + exp1_claib9_i;   % M hand manipulation 
exp1_claib11_i = length(data_Sync11) + exp1_claib10_i;   % M actuator 1 ------- loading **** exp1_claib10_i - exp1_claib11_i
exp1_claib12_i = length(data_Sync12) + exp1_claib11_i;   % M actuator 2

%% RMSE at an exprm. DATA SET
%cI = exp1_claib5_i:exp1_claib6_i; % current index
%cI = exp1_claib8_i:exp1_claib9_i; % current index
cI = 1:exp1_claib5_i; % current index % 가장 베스트 가장 정확. 로딩 없을때
%cI = 1:exp1_claib12_i; % ALL DATA 
%cI = exp1_claib3_i:exp1_claib5_i; %  +++++++++
%cI = exp1_claib6_i:exp1_claib8_i; %  +++++++++
%cI = exp1_claib8_i:exp1_claib9_i; 
%cI = exp1_claib9_i:exp1_claib10_i;
%cI = exp1_claib11_i:exp1_claib12_i; % +++++++++++++
%cI = exp1_claib10_i:exp1_claib11_i; % ++++++++++
%cI = [1:exp1_claib3_i, exp1_claib5_i:exp1_claib6_i];
% PCC+ML+KF Data (with cI index)

%% ANGLE RMSE
% 2 
ang2rmse = sqrt(mean((ang_2_Vjnt(cI, :) - fused_angle2jnt(cI, :)).^2)); % fusioned
ang2errors = (ang_2_Vjnt(cI, :) - fused_angle2jnt(cI, :));

ang2rmse_IC = sqrt(mean((ang_2_Vjnt(cI, :) - ang_2_I_Cjnt(cI, :)).^2));
ang2errors_IC = (ang_2_Vjnt(cI, :) - ang_2_I_Cjnt(cI, :));

ang2rmse_F = sqrt(mean((ang_2_Vjnt(cI, :) - ang_2_Fjnt(cI, :)).^2));
ang2errors_F = (ang_2_Vjnt(cI, :) - ang_2_Fjnt(cI, :));

%3
ang3rmse = sqrt(mean((ang_3_Vjnt(cI, :) - fused_angle3jnt(cI, :)).^2)); % fusioned
ang3errors = (ang_3_Vjnt(cI, :) - fused_angle3jnt(cI, :));

ang3rmse_IC = sqrt(mean((ang_3_Vjnt(cI, :) - ang_3_I_Cjnt(cI, :)).^2));
ang3errors_IC = (ang_3_Vjnt(cI, :) - ang_3_I_Cjnt(cI, :));

ang3rmse_F = sqrt(mean((ang_3_Vjnt(cI, :) - ang_3_Fjnt(cI, :)).^2));
ang3errors_F = (ang_3_Vjnt(cI, :) - ang_3_Fjnt(cI, :));

%4
ang4rmse = sqrt(mean((ang_4_Vjnt(cI, :) - fused_angle4jnt(cI, :)).^2)); % fusioned
ang4errors = (ang_4_Vjnt(cI, :) - fused_angle4jnt(cI, :));

ang4rmse_IC = sqrt(mean((ang_4_Vjnt(cI, :) - ang_4_I_Cjnt(cI, :)).^2));
ang4errors_IC = (ang_4_Vjnt(cI, :) - ang_4_I_Cjnt(cI, :));

ang4rmse_F = sqrt(mean((ang_4_Vjnt(cI, :) - ang_4_Fjnt(cI, :)).^2));
ang4errors_F = (ang_4_Vjnt(cI, :) - ang_4_Fjnt(cI, :));

%5
ang5rmse = sqrt(mean((ang_5_Vjnt(cI, :) - fused_angle5jnt(cI, :)).^2)); % fusioned
ang5errors = ang_5_Vjnt(cI, :) - fused_angle5jnt(cI, :); % fusioned

ang5rmse_IC = sqrt(mean((ang_5_Vjnt(cI, :) - ang_5_I_Cjnt(cI, :)).^2));
ang5errors_IC = (ang_5_Vjnt(cI, :) - ang_5_I_Cjnt(cI, :));

ang5rmse_F = sqrt(mean((ang_5_Vjnt(cI, :) - ang_5_Fjnt(cI, :)).^2));
ang5errors_F = (ang_5_Vjnt(cI, :) - ang_5_Fjnt(cI, :));

%6
ang6rmse = sqrt(mean((ang_6_Vjnt(cI, :) - fused_angle6jnt(cI, :)).^2)); % fusioned
ang6errors = ang_6_Vjnt(cI, :) - fused_angle6jnt(cI, :); % fusioned

ang6rmse_IC = sqrt(mean((ang_6_Vjnt(cI, :) - ang_6_I_Cjnt(cI, :)).^2));
ang6errors_IC = (ang_6_Vjnt(cI, :) - ang_6_I_Cjnt(cI, :));

ang6rmse_F = sqrt(mean((ang_6_Vjnt(cI, :) - ang_6_Fjnt(cI, :)).^2));
ang6errors_F = (ang_6_Vjnt(cI, :) - ang_6_Fjnt(cI, :));

%7
ang7rmse = sqrt(mean((ang_7_Vjnt(cI, :) - fused_angle7jnt(cI, :)).^2)); % fusioned
ang7errors = ang_7_Vjnt(cI, :) - fused_angle7jnt(cI, :); % fusioned

ang7rmse_IC = sqrt(mean((ang_7_Vjnt(cI, :) - ang_7_I_Cjnt(cI, :)).^2));
ang7errors_IC = (ang_7_Vjnt(cI, :) - ang_7_I_Cjnt(cI, :));

ang7rmse_F = sqrt(mean((ang_7_Vjnt(cI, :) - ang_7_Fjnt(cI, :)).^2));
ang7errors_F = (ang_7_Vjnt(cI, :) - ang_7_Fjnt(cI, :));


%% RMSE AT MANIPULATOR - error propagation
% IMU + Flex Sensor + KF
[cord_rmseXKF, cord_rmseYKF, rmsePxKF, rmsePyKF, errpP2xKF, errpP2yKF, ...
    errpP3xKF, errpP3yKF, errpP4xKF, errpP4yKF, errpP5xKF, errpP5yKF, ...
    errpP6xKF, errpP6yKF, errpP7xKF, errpP7yKF] = calculate_rmse_values_MANIPUL( ...
    ang2rmse, ang3rmse, ang4rmse, ang5rmse, ang6rmse, ...
    finalCord2jntNO(cI, :), finalCord3jntNO(cI, :), finalCord4jntNO(cI, :), ...
    finalCord5jntNO(cI, :), finalCord6jntNO(cI, :), finalCord7jntNO(cI, :), ...
    cord2_Vjnt(cI, :), cord3_Vjnt(cI, :), cord4_Vjnt(cI, :), cord5_Vjnt(cI, :), ...
    cord6_Vjnt(cI, :), cord7_Vjnt(cI, :), ang2errors, ang3errors, ang4errors, ang5errors, ang6errors);

% Flex sensor + PCC
[cord_rmseXF, cord_rmseYF, rmsePxF, rmsePyF, errpP2xF, errpP2yF, errpP3xF, ...
    errpP3yF, errpP4xF, errpP4yF, errpP5xF, errpP5yF, errpP6xF, errpP6yF, ....
    errpP7xF, errpP7yF] = calculate_rmse_values_MANIPUL( ...
    ang2rmse_F, ang3rmse_F, ang4rmse_F, ang5rmse_F, ang6rmse_F, ...
    cord_2Fjnt(cI, :), cord_3Fjnt(cI, :), cord_4Fjnt(cI, :), cord_5Fjnt(cI, :), ...
    cord_6Fjnt(cI, :), cord_7Fjnt(cI, :), ...
    cord2_Vjnt(cI, :), cord3_Vjnt(cI, :), cord4_Vjnt(cI, :), cord5_Vjnt(cI, :), ...
    cord6_Vjnt(cI, :), cord7_Vjnt(cI, :), ang2errors_F, ang3errors_F, ang4errors_F, ang5errors_F, ang6errors_F);

% IMU + PCC
[cord_rmseXIC, cord_rmseYIC, rmsePxIC, rmsePyIC, errpP2xIC, errpP2yIC, ...
    errpP3xIC, errpP3yIC, errpP4xIC, errpP4yIC, errpP5xIC, errpP5yIC, ...
    errpP6xIC, errpP6yIC, errpP7xIC, errpP7yIC] = calculate_rmse_values_MANIPUL( ...
    ang2rmse_IC, ang3rmse_IC, ang4rmse_IC, ang5rmse_IC, ang6rmse_IC, ...
    cord_2I_Cjnt(cI, :), cord_3I_Cjnt(cI, :), cord_4I_Cjnt(cI, :), cord_5I_Cjnt(cI, :), ...
    cord_6I_Cjnt(cI, :), cord_7I_Cjnt(cI, :), ...
    cord2_Vjnt(cI, :), cord3_Vjnt(cI, :), cord4_Vjnt(cI, :), cord5_Vjnt(cI, :), ...
    cord6_Vjnt(cI, :), cord7_Vjnt(cI, :), ang2errors_IC, ang3errors_IC, ang4errors_IC, ang5errors_IC, ang6errors_IC);

%% 2nd - Add KF Fusioned Only
fusionedF = sqrt(abs(errpP2xF).^2 + abs(errpP2yF).^2);
fusionedIC = sqrt(abs(errpP2xIC).^2 + abs(errpP2yIC).^2);
fusionedKF = sqrt(abs(errpP2xKF).^2 + abs(errpP2yKF).^2); % KF Only

rmseFINALF2 = sqrt(mean(fusionedF.^2));
rmseFINALIC2 = sqrt(mean(fusionedIC.^2));
rmseFINALKF2 = sqrt(mean(fusionedKF.^2)); % KF Only

%% 3rd
fusionedF = sqrt(abs(errpP3xF).^2 + abs(errpP3yF).^2);
fusionedIC = sqrt(abs(errpP3xIC).^2 + abs(errpP3yIC).^2);
fusionedKF = sqrt(abs(errpP3xKF).^2 + abs(errpP3yKF).^2); % KF Only

rmseFINALF3 = sqrt(mean(fusionedF.^2));
rmseFINALIC3 = sqrt(mean(fusionedIC.^2));
rmseFINALKF3 = sqrt(mean(fusionedKF.^2)); % KF Only

%% 4th
fusionedF = sqrt(abs(errpP4xF).^2 + abs(errpP4yF).^2);
fusionedIC = sqrt(abs(errpP4xIC).^2 + abs(errpP4yIC).^2);
fusionedKF = sqrt(abs(errpP4xKF).^2 + abs(errpP4yKF).^2); % KF Only

rmseFINALF4 = sqrt(mean(fusionedF.^2));
rmseFINALIC4 = sqrt(mean(fusionedIC.^2));
rmseFINALKF4 = sqrt(mean(fusionedKF.^2)); % KF Only

%% 5th
fusionedF = sqrt(abs(errpP5xF).^2 + abs(errpP5yF).^2);
fusionedIC = sqrt(abs(errpP5xIC).^2 + abs(errpP5yIC).^2);
fusionedKF = sqrt(abs(errpP5xKF).^2 + abs(errpP5yKF).^2); % KF Only

rmseFINALF5 = sqrt(mean(fusionedF.^2));
rmseFINALIC5 = sqrt(mean(fusionedIC.^2));
rmseFINALKF5 = sqrt(mean(fusionedKF.^2)); % KF Only

%% 6th
fusionedF = sqrt(abs(errpP6xF).^2 + abs(errpP6yF).^2);
fusionedIC = sqrt(abs(errpP6xIC).^2 + abs(errpP6yIC).^2);
fusionedKF = sqrt(abs(errpP6xKF).^2 + abs(errpP6yKF).^2); % KF Only

rmseFINALF6 = sqrt(mean(fusionedF.^2));
rmseFINALIC6 = sqrt(mean(fusionedIC.^2));
rmseFINALKF6 = sqrt(mean(fusionedKF.^2)); % KF Only

%% 7th
fusionedF = sqrt(abs(errpP7xF).^2 + abs(errpP7yF).^2);
fusionedIC = sqrt(abs(errpP7xIC).^2 + abs(errpP7yIC).^2);
fusionedKF = sqrt(abs(errpP7xKF).^2 + abs(errpP7yKF).^2); % KF Only

rmseFINALF = sqrt(mean(fusionedF.^2)); % Corrected Flex only
rmseFINALIC = sqrt(mean(fusionedIC.^2)); % Corrected IMU only
rmseFINALKF = sqrt(mean(fusionedKF.^2)); % KF Only

%% BOX PLOT - we use this 
rmseValues = [rmseFINALKF, rmseFINALF, rmseFINALIC];
allData = [fusionedKF, fusionedF, fusionedIC];

labels = {'KF', 'Bend Sensor', 'IMU'};

figure;
boxplot(allData, 'Labels', labels, 'Symbol', '');

hold on;
for i = 1:length(rmseValues)
    plot(i, rmseValues(i), 'r*', 'MarkerSize', 8); 
    text(i, rmseValues(i), sprintf('%.2f', rmseValues(i)), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontSize', 8, 'FontName', 'Times New Roman'); 
end

hold off;

title('End Effector Error');
ylabel('Error (mm)');
legend('RMSE', 'Location', 'Best');
set(gca, 'FontName', 'Times New Roman');

%% CALCULATE RMSE AT the index - JOINT FRAME EACH (NOT PROPAGATED) - we can make a error table

% PCC  + Flex Data
rmse2F = calculateRMSE_xy(cord_2Fjnt(cI, :), cord2_Vjnt(cI, :));
rmse3F = calculateRMSE_xy(cord_3Fjnt(cI, :), cord3_Vjnt(cI, :));
rmse4F = calculateRMSE_xy(cord_4Fjnt(cI, :), cord4_Vjnt(cI, :));
rmse5F = calculateRMSE_xy(cord_5Fjnt(cI, :), cord5_Vjnt(cI, :));
rmse6F = calculateRMSE_xy(cord_6Fjnt(cI, :), cord6_Vjnt(cI, :));
rmse7F = calculateRMSE_xy(cord_7Fjnt(cI, :), cord7_Vjnt(cI, :));

% PCC IMU (c) Data (with cI index) 
rmse2IC = calculateRMSE_xy(cord_2I_Cjnt(cI, :), cord2_Vjnt(cI, :));
rmse3IC = calculateRMSE_xy(cord_3I_Cjnt(cI, :), cord3_Vjnt(cI, :));
rmse4IC = calculateRMSE_xy(cord_4I_Cjnt(cI, :), cord4_Vjnt(cI, :));
rmse5IC = calculateRMSE_xy(cord_5I_Cjnt(cI, :), cord5_Vjnt(cI, :));
rmse6IC = calculateRMSE_xy(cord_6I_Cjnt(cI, :), cord6_Vjnt(cI, :));
rmse7IC = calculateRMSE_xy(cord_7I_Cjnt(cI, :), cord7_Vjnt(cI, :));

% KF only 
rmse2KFonly = calculateRMSE_xy(finalCord2jntNO(cI, :), cord2_Vjnt(cI, :));
rmse3KFonly = calculateRMSE_xy(finalCord3jntNO(cI, :), cord3_Vjnt(cI, :));
rmse4KFonly = calculateRMSE_xy(finalCord4jntNO(cI, :), cord4_Vjnt(cI, :));
rmse5KFonly = calculateRMSE_xy(finalCord5jntNO(cI, :), cord5_Vjnt(cI, :));
rmse6KFonly = calculateRMSE_xy(finalCord6jntNO(cI, :), cord6_Vjnt(cI, :));
rmse7KFonly = calculateRMSE_xy(finalCord7jntNO(cI, :), cord7_Vjnt(cI, :));

save update_iros.mat 
%% Functions 
function [cord_rmseX, cord_rmseY, rmsePx, rmsePy, errp2x, errp2y, errp3x, ...
    errp3y,errp4x, errp4y,errp5x, errp5y,errp6x, errp6y,errp7x, errp7y] = ...
    calculate_rmse_values_MANIPUL(ang2rmse, ang3rmse, ang4rmse, ang5rmse, ang6rmse, ...
    cord2jnt, cord3jnt, cord4jnt, cord5jnt, cord6jnt, cord7jnt, ...                 %estimation 
    cord2_Vjnt, cord3_Vjnt, cord4_Vjnt, cord5_Vjnt, cord6_Vjnt, cord7_Vjnt, ...     %GT
    ang2errors, ang3errors, ang4errors, ang5errors, ang6errors)                     %angle error

    % Calculate RMSE for each cord
    
    cord2errorsX = cord2jnt(:,1) - cord2_Vjnt(:,1);
    cord2errorsY = cord2jnt(:,2) - cord2_Vjnt(:,2);

    cord2rmseX = sqrt(mean((cord2jnt(:,1) - cord2_Vjnt(:,1)).^2));
    cord2rmseY = sqrt(mean((cord2jnt(:,2) - cord2_Vjnt(:,2)).^2));
    
    cord3errorsX = cord3jnt(:,1) - cord3_Vjnt(:,1);
    cord3errorsY = cord3jnt(:,2) - cord3_Vjnt(:,2);

    cord3rmseX = sqrt(mean((cord3jnt(:,1) - cord3_Vjnt(:,1)).^2));
    cord3rmseY = sqrt(mean((cord3jnt(:,2) - cord3_Vjnt(:,2)).^2));

    cord4errorsX = cord4jnt(:,1) - cord4_Vjnt(:,1);
    cord4errorsY = cord4jnt(:,2) - cord4_Vjnt(:,2);

    cord4rmseX = sqrt(mean((cord4jnt(:,1) - cord4_Vjnt(:,1)).^2));
    cord4rmseY = sqrt(mean((cord4jnt(:,2) - cord4_Vjnt(:,2)).^2));

    cord5errorsX = cord5jnt(:,1) - cord5_Vjnt(:,1);
    cord5errorsY = cord5jnt(:,2) - cord5_Vjnt(:,2);

    cord5rmseX = sqrt(mean((cord5jnt(:,1) - cord5_Vjnt(:,1)).^2));
    cord5rmseY = sqrt(mean((cord5jnt(:,2) - cord5_Vjnt(:,2)).^2));

    cord6errorsX = cord6jnt(:,1) - cord6_Vjnt(:,1);
    cord6errorsY = cord6jnt(:,2) - cord6_Vjnt(:,2);

    cord6rmseX = sqrt(mean((cord6jnt(:,1) - cord6_Vjnt(:,1)).^2));
    cord6rmseY = sqrt(mean((cord6jnt(:,2) - cord6_Vjnt(:,2)).^2));

    cord7errorsX = cord7jnt(:,1) - cord7_Vjnt(:,1);
    cord7errorsY = cord7jnt(:,2) - cord7_Vjnt(:,2);

    cord7rmseX = sqrt(mean((cord7jnt(:,1) - cord7_Vjnt(:,1)).^2));
    cord7rmseY = sqrt(mean((cord7jnt(:,2) - cord7_Vjnt(:,2)).^2));

    errp2x = cord2errorsX;
    errp2y = cord2errorsY;

    ang_3errors = ang2errors;
    errp3x = cord2errorsX + cord3errorsX + sin(deg2rad(ang_3errors));
    errp3y = cord2errorsY + cord3errorsY + cos(deg2rad(ang_3errors));
    
    ang_4errors = ang2errors + ang3errors;
    errp4x = cord2errorsX + cord3errorsX + cord4errorsX + sin(deg2rad(ang_4errors));
    errp4y = cord2errorsY + cord3errorsY + cord4errorsY + cos(deg2rad(ang_4errors));

    ang_5errors = ang2errors + ang3errors + ang4errors;
    errp5x = cord2errorsX + cord3errorsX + cord4errorsX + cord5errorsX + sin(deg2rad(ang_5errors));
    errp5y = cord2errorsY + cord3errorsY + cord4errorsY + cord5errorsY + cos(deg2rad(ang_5errors));
    
    ang_6errors = ang2errors + ang3errors + ang4errors + ang5errors;
    errp6x = cord2errorsX + cord3errorsX + cord4errorsX + cord5errorsX + cord6errorsX + sin(deg2rad(ang_6errors));
    errp6y = cord2errorsY + cord3errorsY + cord4errorsY + cord5errorsY + cord6errorsY + cos(deg2rad(ang_6errors));

    ang_7errors = ang2errors + ang3errors + ang4errors + ang5errors + ang6errors;
    errp7x = cord2errorsX + cord3errorsX + cord4errorsX + cord5errorsX + cord6errorsX + cord7errorsX + sin(deg2rad(ang_7errors));
    errp7y = cord2errorsY + cord3errorsY + cord4errorsY + cord5errorsY + cord6errorsY + cord7errorsY + cos(deg2rad(ang_7errors));

    % Calculate cumulative RMSE for each point
    rmseP2x = cord2rmseX;
    rmseP2y = cord2rmseY;
    total_ang_rmse3 = ang2rmse;

    rmseP3x = cord2rmseX + cord3rmseX + sin(deg2rad(total_ang_rmse3));
    rmseP3y = cord2rmseY + cord3rmseY + cos(deg2rad(total_ang_rmse3));

    total_ang_rmse4 = ang2rmse + ang3rmse;
    rmseP4x = cord2rmseX + cord3rmseX + cord4rmseX + sin(deg2rad(total_ang_rmse4));
    rmseP4y = cord2rmseY + cord3rmseY + cord4rmseY + cos(deg2rad(total_ang_rmse4));

    total_ang_rmse5 = ang2rmse + ang3rmse + ang4rmse;
    rmseP5x = cord2rmseX + cord3rmseX + cord4rmseX + cord5rmseX + sin(deg2rad(total_ang_rmse5));
    rmseP5y = cord2rmseY + cord3rmseY + cord4rmseY + cord5rmseY + cos(deg2rad(total_ang_rmse5));

    total_ang_rmse6 = ang2rmse + ang3rmse + ang4rmse + ang5rmse;
    rmseP6x = cord2rmseX + cord3rmseX + cord4rmseX + cord5rmseX + cord6rmseX + sin(deg2rad(total_ang_rmse6));
    rmseP6y = cord2rmseY + cord3rmseY + cord4rmseY + cord5rmseY + cord6rmseY + cos(deg2rad(total_ang_rmse6));

    total_ang_rmse7 = ang2rmse + ang3rmse + ang4rmse + ang5rmse + ang6rmse;
    rmseP7x = cord2rmseX + cord3rmseX + cord4rmseX + cord5rmseX + cord6rmseX + cord7rmseX + sin(deg2rad(total_ang_rmse7));
    rmseP7y = cord2rmseY + cord3rmseY + cord4rmseY + cord5rmseY + cord6rmseY + cord7rmseY + cos(deg2rad(total_ang_rmse7));

    % Output all RMSE values
    cord_rmseX = [cord2rmseX, cord3rmseX, cord4rmseX, cord5rmseX, cord6rmseX, cord7rmseX];
    cord_rmseY = [cord2rmseY, cord3rmseY, cord4rmseY, cord5rmseY, cord6rmseY, cord7rmseY];
    rmsePx = [rmseP2x, rmseP3x, rmseP4x, rmseP5x, rmseP6x, rmseP7x];
    rmsePy = [rmseP2y, rmseP3y, rmseP4y, rmseP5y, rmseP6y, rmseP7y];
end