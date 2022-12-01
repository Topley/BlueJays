%% Load csv, extract data, and save data as a .mat for processing
clear all; close all; clc;

[num,txt,raw] = xlsread('C:\Users\tuc43377\Downloads\[External]_Blue_Jays_Biomechanist_Role\biomechanics_data.csv');

allTrials = raw(2:end, :);
type = unique(allTrials(2:end, 6));
discreteRows = contains(allTrials(:,6), type{1});
discreteData = allTrials(discreteRows, :);
disEvent = unique(discreteData(:, 7));
seriesData = allTrials;
seriesData(discreteRows, :) = [];
signal = unique(seriesData(:, 7));
allP = unique(allTrials(:,1));
series = unique(seriesData(:,1));

for i = 1:size(allP,1)
    playerN = allP{i};
    pDisRows = contains(discreteData(:,1), playerN);
    pDisData = discreteData(pDisRows, :);
    for j = 1:size(disEvent,1)
        eventRows = contains(pDisData(:,7), disEvent{j});
        eventData = pDisData(eventRows, [7,9]);
        domArm = pDisData(eventRows, 2);
        Pitch = pDisData(eventRows, 3);
        pVelo = pDisData(eventRows,4);
        spinRate = pDisData(eventRows,5);
        trialStruct(i).hand = domArm{:};
        trialStruct(i).Pitch = Pitch{:};
        trialStruct(i).Velo = pVelo{:};
        trialStruct(i).SpinRate = spinRate{:};
        trialStruct(i).Events.(disEvent{j}) = eventData{2};
    end
end

for ii = 1:size(allP,1)
    
    try
        playerN = allP{ii};
        subjectRows = contains(seriesData(:,1), playerN);
        subjectData = seriesData(subjectRows, :);
        
        for j = 1:size(signal,1)
            sigRows = contains(subjectData(:,7), signal{j});
            sigData = subjectData(sigRows, [8,9]);
            sigMat = cell2mat(sigData);
            [~,idx] = sort(sigMat(:,1));
            timeAligned = sigMat(idx,:);
            trialStruct(ii).(signal{j}) = timeAligned(:,2);
            trialStruct(ii).Time = timeAligned(:,1);
        end
    end
end
save('C:\Users\tuc43377\Desktop\BlueJaysDataStruct.mat', 'trialStruct');

%% open .mat so matlab runs faster
clear all; clc; close all;
load('C:\Users\tuc43377\Desktop\BlueJaysDataStruct.mat')

%% plot Leg Kinematics
figure(1);
t = tiledlayout(2,1, 'TileSpacing', 'Compact');
c = 1;
for j = 1:size(trialStruct,2)
    name = ['P', num2str(j)];
    time = [0:length(trialStruct(j).Time)-1] ./70;
    layback = trialStruct(j).Events.Time_to_MAX_EXT;
    release = trialStruct(j).Events.Time_to_Release;
    dKneR = trialStruct(j).Events.DriveKnee_ANGLE_Flexion_At_RELEASE;
    lKneR = trialStruct(j).Events.LeadKnee_ANGLE_Flexion_At_RELEASE;
    dKneX = trialStruct(j).Events.DriveKnee_ANGLE_Flexion_At_FtStrike;
    lKneX = trialStruct(j).Events.LeadKnee_ANGLE_Flexion_At_FtStrike;
    
    ax1 = nexttile(1);
    dKNE =  trialStruct(j).DriveKnee_ANGLE_X;
    if ~isempty(dKNE)
        ax1.FontSize = 20;
        plot(ax1, time, dKNE, 'LineWidth', 2, 'SeriesIndex', c, 'DisplayName', [name, ': FtStr ', sprintf('%0.1f', dKneX), ', Rel ', sprintf('%0.1f',dKneR)])
        title('Drive Knee', 'FontSize', 10)
        hold on
    end
    
    ax2 = nexttile(2);
    lKNE =  trialStruct(j).LeadKnee_ANGLE_X;
    if ~isempty(lKNE)
        ax2.FontSize = 20;
        plot(ax2, time, lKNE, 'LineWidth', 2,'SeriesIndex', c, 'DisplayName', ['FtStr ', sprintf('%0.1f', lKneX), ', Rel ', sprintf('%0.1f',lKneR)])
        title('Lead Knee Angle', 'FontSize', 10)
        hold on
        c = c + 2;
    end
end

xlabel(t, 'Time', 'FontSize', 25)
ylabel(t, 'Angle in Degrees', 'FontSize', 25)

figAxes = findall(gcf,'type','axes');
for jj = 1: length(figAxes)
    yLim = get(figAxes(jj), 'YLim');
    ylim(figAxes(jj), [yLim(1) - 20 yLim(2) + 20]);
    xlim(figAxes(jj), [min(time) max(time)]);
    legend(figAxes(jj),  'Location', 'northwest');
    if jj > 1
        figAxes(jj).XTickLabel = [];
    end
    
    figAxes(jj).TickLength = [0 0];
    box(figAxes(jj), 'off');
end

title(t,'Knee Angles', 'FontSize', 25);

%% Save figure to insert into report document
savename = ['C:\Users\tuc43377\Desktop\', 'LegKinematics.png'];
set(gcf, 'PaperPosition', [0 0 20 15]);
set(gcf, 'PaperSize', [20 15]);
saveas(gcf,savename);

%% plot Leg Kinematics
figure(2);
t = tiledlayout(2,1, 'TileSpacing', 'Compact');
c = 1;
for j = 1:size(trialStruct,2)
    name = ['P', num2str(j)];
    time = [0:length(trialStruct(j).Time)-1] ./70;
    layback = trialStruct(j).Events.Time_to_MAX_EXT;
    release = trialStruct(j).Events.Time_to_Release;
    lTiltR = trialStruct(j).Events.Trunk_Lateral_Tilt_At_RELEASE;
    lTiltX = trialStruct(j).Events.Trunk_Lateral_Tilt_At_FtStrike;
    tiltR = trialStruct(j).Events.Trunk_Tilt_At_RELEASE;
    tiltX = trialStruct(j).Events.Trunk_Tilt_At_FtStrike;
    
     ax1 = nexttile(1);
    lTilt =  trialStruct(j).Trunk_Lateral_Tilt;
    if ~isempty(lTilt)
        ax1.FontSize = 20;
        plot(ax1, time,  lTilt, 'LineWidth', 2, 'SeriesIndex', c, 'DisplayName', ['FtStr ', sprintf('%0.1f', lTiltX), ', Rel ', sprintf('%0.1f', lTiltR)])
        title('Trunk Lateral Tilt', 'FontSize', 10)
        hold on
    end
    
    ax2 = nexttile(2);
    tilt =  trialStruct(j).Trunk_Tilt;
    if ~isempty(tilt)
        ax2.FontSize = 20;
        plot(ax2, time,  tilt, 'LineWidth', 2, 'SeriesIndex', c, 'DisplayName', ['FtStr ', sprintf('%0.1f', tiltX), ', Rel ', sprintf('%0.1f',tiltR)])
        title('Trunk Foward Tilt', 'FontSize', 10)
        hold on
        c = c + 2;
    end
end

xlabel(t, 'Time', 'FontSize', 25)
ylabel(t, 'Angle in Degrees', 'FontSize', 25)

figAxes = findall(gcf,'type','axes');
for jj = 1: length(figAxes)
    yLim = get(figAxes(jj), 'YLim');
    ylim(figAxes(jj), [yLim(1) - 20 yLim(2) + 20]);
    xlim(figAxes(jj), [min(time) max(time)]);
    legend(figAxes(jj), 'Location', 'northwest');
    if jj > 1
        figAxes(jj).XTickLabel = [];
    end
    
    figAxes(jj).TickLength = [0 0];
    box(figAxes(jj), 'off');
end

title(t,'Trunk tilt', 'FontSize', 25);

%% Save figure to insert into report document
savename = ['C:\Users\tuc43377\Desktop\', 'TrunkTilt.png'];
set(gcf, 'PaperPosition', [0 0 20 15]);
set(gcf, 'PaperSize', [20 15]);
saveas(gcf,savename);

%% plot Hip and Trunk kinematics
figure(3);
t = tiledlayout(3,1, 'TileSpacing', 'Compact');
c = 1;
for j = 1:size(trialStruct,2)
    name = ['P', num2str(j)];
    time = [0:length(trialStruct(j).Time)-1] ./70;
    layback = trialStruct(j).Events.Time_to_MAX_EXT;
    release = trialStruct(j).Events.Time_to_Release;
    PRotR = trialStruct(j).Events.Pelvis_Rotation_At_RELEASE;
    PRotX = trialStruct(j).Events.Pelvis_Rotation_At_FtStrike;
    TRotR = trialStruct(j).Events.Trunk_Rotation_At_RELEASE;
    TRotX = trialStruct(j).Events.Trunk_Rotation_At_FtStrike;
    htsR = trialStruct(j).Events.HipShoulderSeparation_At_RELEASE;
    htsX = trialStruct(j).Events.HipShoulderSeparation_At_FtStrike;
    
    ax1 = nexttile(1);
    pRot =  trialStruct(j).Pelvis_Rotation;
    if ~isempty(pRot)
        ax1.FontSize = 20;
        plot(ax1, time, pRot, 'LineWidth', 2, 'SeriesIndex', c, 'DisplayName', [name, ': FtStr ', sprintf('%0.1f', PRotX), ', Rel ', sprintf('%0.1f',PRotR)])
        title('Hip Rotation', 'FontSize', 10)
        hold on
    end
    
    ax2 = nexttile(2);
    tRot =  trialStruct(j).Trunk_Rotation;
    if ~isempty(tRot)
        ax2.FontSize = 20;
        plot(ax2, time, tRot, 'LineWidth', 2,'SeriesIndex', c, 'DisplayName', ['FtStr ', sprintf('%0.1f', TRotX), ', Rel ', sprintf('%0.1f',TRotR)])
        title('Trunk Rotation', 'FontSize', 10)
        hold on
    end
    
    ax3 = nexttile(3);
    hsep =  trialStruct(j).HipShoulderSeparation;
    if ~isempty(hsep)
        ax3.FontSize = 20;
        plot(ax3, time,  hsep, 'LineWidth', 2, 'SeriesIndex', c, 'DisplayName', ['FtStr ', sprintf('%0.1f', htsX), ', Rel ', sprintf('%0.1f',htsR)])
        title('Trunk Forward Tilt', 'FontSize', 10)
        hold on
        c = c + 2;
    end
end

xlabel(t, 'Time', 'FontSize', 25)
ylabel(t, 'Angle in degrees', 'FontSize', 25)

figAxes = findall(gcf,'type','axes');
for jj = 1: length(figAxes)
    yLim = get(figAxes(jj), 'YLim');
    ylim(figAxes(jj), [yLim(1) - 20 yLim(2) + 20]);
    xlim(figAxes(jj), [min(time) max(time)]);
    figAxes(jj).TickLength = [0 0];
    box(figAxes(jj), 'off');
    legend(figAxes(jj), 'Location', 'northwest');
    if jj > 1
        figAxes(jj).XTickLabel = [];
    end
    
end
title(t,'Hip through Trunk Rotation', 'FontSize', 25)

%% Save figure to insert into report document
savename = ['C:\Users\tuc43377\Desktop\', 'HipTrunk.png'];
set(gcf, 'PaperPosition', [0 0 20 15]); %Position plot at left hand corner with width 5 and height 5.
set(gcf, 'PaperSize', [20 15]); %Set the paper to have width 5 and height 5
saveas(gcf,savename);

%% plot Trunk, Shoulder, and Elbow Kinematics
figure(5)
t = tiledlayout(4,1, 'TileSpacing', 'Compact');
c = 1;
for j = 1:size(trialStruct,2)
    
    name = ['Pitcher', num2str(j)];
    time = [0:length(trialStruct(j).Time)-1] ./70;
    layback = trialStruct(j).Events.Time_to_MAX_EXT;
    release = trialStruct(j).Events.Time_to_Release;
    shXR = trialStruct(j).Events.ShoulderHorizAbd_At_RELEASE;
    shXX = trialStruct(j).Events.ShoulderHorizAbd_At_FtStrike;
    shYR = trialStruct(j).Events.ShoulderRotation_At_RELEASE;
    shYX = trialStruct(j).Events.ShoulderRotation_At_FtStrike;
    shZR = trialStruct(j).Events.ShoulderAbduction_At_RELEASE;
    shZX = trialStruct(j).Events.ShoulderAbduction_At_FtStrike;
    eAngR = trialStruct(j).Events.Elbow_Flexion_At_RELEASE;
    eAngX = trialStruct(j).Events.Elbow_Flexion_At_FtStrike;
    
    ax1 = nexttile(1);
    shX =  trialStruct(j).Shoulder_Angle_ZYZ_X;
    if ~isempty(pRot)
        ax1.FontSize = 20;
        plot(ax1, time, shX, 'LineWidth', 2, 'SeriesIndex', c, 'DisplayName', [name, ': FtStr ', sprintf('%0.1f', shXX), ', Rel ', sprintf('%0.1f',shXR)])
        title('Shoulder Horizontal ABD', 'FontSize', 10)
        hold on
    end
    
    ax2 = nexttile(2);
    shY =  trialStruct(j).Shoulder_Angle_ZYZ_Y;
    if ~isempty(tRot)
        ax2.FontSize = 20;
        plot(ax2, time, shY, 'LineWidth', 2,'SeriesIndex', c, 'DisplayName', ['FtStr ', sprintf('%0.1f', shYX), ', Rel ', sprintf('%0.1f',shYR)])
        title('Shoulder ABD', 'FontSize', 10)
        hold on
    end
    
    ax3 = nexttile(3);
    shZ =  trialStruct(j).Shoulder_Angle_ZYZ_Z;
    if ~isempty(hsep)
        plot(ax3, time,  shZ, 'LineWidth', 2, 'SeriesIndex', c, 'DisplayName', ['FtStr ', sprintf('%0.1f', shZX), ', Rel ', sprintf('%0.1f',shZR)])
        ax3.FontSize = 20;
        title('Shoulder INT/EXT Rotation', 'FontSize', 10)
        hold on
    end
    
    ax4 = nexttile(4);
    eAng =  trialStruct(j).Elbow_Angle_X;
    if ~isempty(eAng)
        plot(ax4, time, eAng, 'LineWidth', 2, 'SeriesIndex', c, 'DisplayName', ['FtStr ', sprintf('%0.1f', eAngX), ', Rel ', sprintf('%0.1f',eAngR)])
        ax4.FontSize = 20;
        title('Elbow Flexion', 'FontSize', 10)
        hold on
        c = c + 2;
    end
    
end
xlabel(t, 'Time', 'FontSize', 25)
ylabel(t, 'Angle in degrees', 'FontSize', 25)

figAxes = findall(gcf,'type','axes');
for jj = 1: length(figAxes)
    yLim = get(figAxes(jj), 'YLim');
    ylim(figAxes(jj), [yLim(1) - 10 yLim(2) + 10]);
    xlim(figAxes(jj), [min(time) max(time)]);
    figAxes(jj).TickLength = [0 0];
    box(figAxes(jj), 'off');
     legend(figAxes(jj), 'Location', 'northwest');
    if jj > 1
        figAxes(jj).XTickLabel = [];
    end
    
end
title(t,'Shoulder and Elbow Rotations', 'FontSize', 25)

%% Save figure to insert into report document
savename = ['C:\Users\tuc43377\Desktop\', 'Arm_Rotations.png'];
set(gcf, 'PaperPosition', [0 0 20 20]); %Position plot at left hand corner with width 5 and height 5.
set(gcf, 'PaperSize', [20 20]); %Set the paper to have width 5 and height 5
saveas(gcf,savename);

%% plot Shoulder and Elbow Rotations
figure(4)
t = tiledlayout(5,1, 'TileSpacing', 'Compact');
c = 1;
for j = 1:size(trialStruct,2)
    
    name = ['P', num2str(j)];
    time = [0:length(trialStruct(j).Time)-1] ./70;
    pMagX = trialStruct(j).Events.MAX_RPV_ANGVEL_MAG;
    tMagX = trialStruct(j).Events.MAX_RTA_ANGVEL_MAG;
    aMagX = trialStruct(j).Events.MAX_AR_ANGVEL_MAG;
    fMagX = trialStruct(j).Events.MAX_FA_ANGVEL_MAG;
    hMagX = trialStruct(j).Events.MAX_HA_ANGVEL_MAG;
    
    ax1 = nexttile(1);
    pMag =  trialStruct(j).RPV_ANGVEL_MAG;
    if ~isempty(pMag)
        ax1.FontSize = 20;
        plot(ax1, time, pMag, 'LineWidth', 2, 'SeriesIndex', c, 'DisplayName',[name, ': Max Velo ', sprintf('%0.1f', pMagX)])
        title('Pelvis Ang Velocity', 'FontSize', 10)
        hold on
    end
    
    ax2 = nexttile(2);
    tMag =  trialStruct(j).RTA_ANGVEL_MAG;
    if ~isempty(tMag)
        ax2.FontSize = 20;
        plot(ax2, time, tMag, 'LineWidth', 2,'SeriesIndex', c, 'DisplayName', ['Max Velo ', sprintf('%0.1f', tMagX)])
        title('Trunk Ang Velocity', 'FontSize', 10)
        hold on
    end
    
    ax3 = nexttile(3);
    aMag =  trialStruct(j).AR_ANGVEL_MAG;
    if ~isempty(aMag)
        plot(ax3, time,  aMag, 'LineWidth', 2, 'SeriesIndex', c, 'DisplayName', ['Max Velo ', sprintf('%0.1f', aMagX)])
        ax3.FontSize = 20;
        title('Shoulder Ang Velocity', 'FontSize', 10)
        hold on
    end
    
    ax4 = nexttile(4);
    fMag =  trialStruct(j).FA_ANGVEL_MAG;
    if ~isempty(fMag)
        plot(ax4, time, fMag, 'LineWidth', 2, 'SeriesIndex', c, 'DisplayName', ['Max Velo ', sprintf('%0.1f', fMagX)])
        ax4.FontSize = 20;
        title('Elbow Ang Velocity', 'FontSize', 10)
        hold on
    end
    
    ax5 = nexttile(5);
    hMag =  trialStruct(j).HA_ANGVEL_MAG;
    if ~isempty(hMag)
        plot(ax5, time, hMag, 'LineWidth', 2, 'SeriesIndex', c, 'DisplayName', ['Max Velo ', sprintf('%0.1f', hMagX)])
        ax5.FontSize = 20;
        title('Hand Ang Velocity', 'FontSize', 10)
        hold on
        c = c + 2;
    end
    
end
xlabel(t, 'Time', 'FontSize', 25)
ylabel(t, 'Velocity', 'FontSize', 25)

figAxes = findall(gcf,'type','axes');
for jj = 1: length(figAxes)
    yLim = get(figAxes(jj), 'YLim');
    ylim(figAxes(jj), [yLim(1) - 20 yLim(2) + 20]);
    xlim(figAxes(jj), [min(time) max(time)]);
    figAxes(jj).TickLength = [0 0];
    box(figAxes(jj), 'off');
    legend(figAxes(jj),  'Location', 'northwest');
    
    if jj > 1
        figAxes(jj).XTickLabel = [];
    end
    
end
title(t,'Kinetic Chain', 'FontSize', 25)

%% Save figure to insert into report document
savename = ['C:\Users\tuc43377\Desktop\', 'ArmKinematics.png'];
set(gcf, 'PaperPosition', [0 0 20 20]); %Position plot at left hand corner with width 5 and height 5.
set(gcf, 'PaperSize', [20 20]); %Set the paper to have width 5 and height 5
saveas(gcf,savename);
