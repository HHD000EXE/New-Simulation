%% ReadMe

%This file is only to analyze data for each individual gait. To analyze
%data from "demo" folder, use file named "anal_traj.m".

%To ease in data visualization, relevant matlab variables are saved in
%their respective folders in the directory "matlab data". Import them to
%quickly plot basin plots and/or robot motion trajectories.

%% Initialize values
% Initializes file directory paths to import data 
clear; clc;

speed_param = '0p4'; %Options: 0p4, 2p0
gaitline = 'Pace-Trot'; %Options: 'Bound-Pace', 'Bound-Trot', 'Pace-Trot'

% vicondir = sprintf('/home/anmolk/Desktop/kodlab/Experiment_Data/%s/Speed%s/',gaitline,speed_param); %directory for vicon data
% notefile = sprintf('/home/anmolk/Desktop/kodlab/Experiment_Data/%s/',gaitline); %directory for experiment notes excel sheet

vicondir = sprintf('C:/Users/qianf/Dropbox/USC/Research projects/Obstacle-Gait Coupling project/experiment/data/Anmol/Experiment_Data/%s/Speed%s/',gaitline,speed_param); %directory for vicon data
notefile = sprintf('C:/Users/qianf/Dropbox/USC/Research projects/Obstacle-Gait Coupling project/experiment/data/Anmol/Experiment_Data/%s/',gaitline); %directory for experiment notes excel sheet

%% Import filenames
%Reads file names in the directory and builds state arrays for the
%experiments

filenamelist = 0; clear filenamelist;
files = dir(vicondir);

allfiles = dir(vicondir); %Create struct with all the directory metadata

count = 1;

for jj = 3:size(allfiles,1)
    
    counter = findstr('z0', allfiles(jj).name); %ignore all files with initial position not equal to -0.5 
    if strcmp(allfiles(jj).name(counter+3:counter+5), '0p5') ~= 1
        continue
    end
    
    filenametmp=strcat(vicondir,allfiles(jj).name);
    filenamelist(count)={filenametmp};
    fnameintmp=allfiles(jj).name;    
    namelist(count)={fnameintmp};
    
    %Example filename:: 20191201_gd0p00_spacing1p98_speed040_z0-0p5_Theta000_run 1.csv
    
    i1 = findstr('gd', fnameintmp); %Build array for gait distance
    gait_d(count) = str2double(fnameintmp(i1+4:i1+5))/100;
    
    i2 = findstr('speed', fnameintmp); %Speed of robot
    speed = str2double(fnameintmp(i2+5:i2+7))/100; %speed is held constant
    
    i3 = findstr('Theta', fnameintmp); %Initial yaw angle given to robot
    theta(count) = str2double(fnameintmp(i3+5:i3+7));
    
    
    count = count + 1;
end

lff = length(filenamelist);

%% Import vicon data
%Read each excel file and store the trajectory data in a cell struct

yawdata = {}; %Yaw data of each trial
txdata = {}; %Foreaft direction of each trial
tydata = {}; %Lateral direction of each trial

for jj = 1:lff
    data = readtable(filenamelist{jj});
    temp = table2array(data(:,5));
    temp1 = table2array(data(:,6));
    temp2 = table2array(data(:,7));
    
    rz = [];
    tx = [];
    ty = [];

    for i = 3:length(temp)
       rz = [rz; str2double(temp{i,1})];
       tx = [tx; str2double(temp1{i,1})];
       ty = [ty; str2double(temp2{i,1})];
    end
    
    tx = (tx - tx(1))/1000; %Convert to metres
    ty = (ty - ty(1))/1000; %Convert to metres
    
    yawdata{jj} = rad2deg(rz); %Convert to degrees from radians
    txdata{jj} = tx;
    tydata{jj} = ty;
end

%% Clean and Process data
%Process all the trajectory data to extract meaningful observations

clc;
startindex = []; %startindex of the yaw angle array after it becomes steady
stopindex = []; %last index of the yaw angle array after it becomes steady
steadyangle = []; %Steady yaw angle value for the "i"th trajectory 
min_steady_error = []; %the lower bound of error for the "i"th trajectory
max_steady_error = []; %the upper bound of error for the "i"th trajectory
initial_angle = []; %initial starting angle for the "i"th trajectory

for i = 1:length(yawdata)
    %Vicon wasn't calibrated properly, so perform linear change of
    %coordinates to bring yaw angle data in the right coordinate system
    
    if strcmp(gaitline, 'Bound-Trot') %Bound-Trot data is weird, check
        yawdata{i} = 180 - yawdata{i};
    else
        if yawdata{i}(1) < -20
            yawdata{i} = -1*(yawdata{i} + 90);
        end
        if yawdata{i} > 150
            yawdata{i} = 270 - yawdata{i};
        end
    end
    
    %Calculate start and stop index based on robot speed
    
    if speed == 0.4 %Assign start and stop indices based on speed
        startindex(i) = round(length(yawdata{i})*0.9) - 500; %slower speed, more number of samples
        stopindex(i) = round(length(yawdata{i})*0.9);
    else
        startindex(i) = round(length(yawdata{i})*0.9) - 200; %faster speed, less samples
        stopindex(i) = round(length(yawdata{i})*0.9);
    end
    
    %Calculate desired parameters
    initial_angle(i) = yawdata{i}(200);
    steadyangle(i) = mean(yawdata{i}(startindex(i):stopindex(i)));
    min_steady_error(i) = mean(yawdata{i}(startindex(i):stopindex(i))) - min(yawdata{i}(startindex(i):stopindex(i)));
    max_steady_error(i) = max(yawdata{i}(startindex(i):stopindex(i))) - mean(yawdata{i}(startindex(i):stopindex(i)));
end

%% Basin Formation
%Calculate the basin angle that each trial falls into

steady_basin = []; %Array containing the basin angle for each "i"th trajectory

%Round to nearest 10 integer and clip the basin values between -90 and +90 degrees
for i = 1:length(yawdata)
    
    if(mod(steadyangle(i),10)>=5.0)
        steady_basin(i) = ceil(steadyangle(i)/10)*10;
    else
        steady_basin(i) = floor(steadyangle(i)/10)*10;
    end
    
    if steady_basin(i) > 90
        steady_basin(i) = 90;
    end
    if steady_basin(i) < -90
        steady_basin(i) = -90;
    end 
end

%Calculate the min and max error for each gait_d basin angles
unique_gait = unique(gait_d);
for i = 1:length(unique(gait_d))
   list_gd = find(gait_d == unique_gait(i));
   list_sb = unique(steady_basin(list_gd));
   for j=1:length(list_sb)
       index=[];
       for k=1:length(list_gd)
          if(steady_basin(list_gd(k))==list_sb(j))
            index=[index;list_gd(k)];
          end
       end
       temp_min = max(min_steady_error(index));
       temp_max = max(max_steady_error(index));
       for k=1:length(index)
           min_steady_error(index(k)) = temp_min;
           max_steady_error(index(k)) = temp_max;
       end
   end
end


%% Plot Yaw Angle trajectory

numgait_d = unique(gait_d);
figure();
for k = 1:length(numgait_d)
    legend_data =  {};
    count = 1;
    for i = 1:lff    
        if gait_d(i) == numgait_d(k)
            
            legend_data{count} = sprintf('%0.0f -> %0.0f', initial_angle(i) , steadyangle(i));
            count = count+1;
            
            if(mod(length(numgait_d),2)==0)
                subplot(2, length(numgait_d)-2, k)
            else
                subplot(length(numgait_d), 1, k)
            end
            
            plot(yawdata{i},'LineWidth',2); hold on;
        end
    end
    legend(legend_data);
    set(gcf,'color','w'); grid on;
    xlabel('Frame number'); ylabel('Yaw Angle [deg]');
    eval(sprintf('title("Gait Distance: %0.2f %s, Speed = %0.2f")',numgait_d(k), gaitline, speed));
end

%% Plot robot motion Trajectories

numgait_d = unique(gait_d);
AllBasins = -90:10:90;

lincolor = jet(length(AllBasins))';

for k = 1:length(numgait_d)
    legend_data =  {};
    figure();
    for i = 1:lff    
        if gait_d(i) == numgait_d(k) 

            col = getColor(steady_basin, i);
            plot(-tydata{i}, txdata{i},'LineWidth',2,'color',lincolor(:,col)); hold on;
        end
    end
    set(gcf,'color','w'); grid on;
    xlabel('Lateral Direction [m]'); ylabel('Foreaft Direction [m]');
    eval(sprintf('title("Gait Distance: %0.2f %s, Speed = %0.2f")',numgait_d(k), gaitline, speed));
    axis image;
end

%% Average yaw angle for each trial overlaid with the basin angle it falls into

numgait_d = unique(gait_d);
figure();

for k = 1:length(numgait_d)
    
    min_d = [];
    max_d = [];
    mean_d = [];
    initial = [];
    basin_data = [];
    
    for i = 1:lff    
        if gait_d(i) == numgait_d(k) 
            
            min_d = [min_d; min_steady_error(i)];
            max_d = [max_d; max_steady_error(i)];
            mean_d = [mean_d; steadyangle(i)];
            initial = [initial; initial_angle(i)];
            basin_data = [basin_data; steady_basin(i)];
        end
    end
    
    if(mod(length(numgait_d),2)==0)
        subplot(2, length(numgait_d)-2, k)
    else
        subplot(length(numgait_d), 1, k)
    end
    
    errorbar(initial, mean_d, min_d, max_d, 'o'); hold on;
    plot(initial, basin_data,'r.','MarkerSize',20);
    
    set(gcf,'color','w'); grid on;
    xlabel('Initial Angle [deg]'); ylabel('Final Yaw Angle [deg]'); zlabel('Gait Distance');    
    eval(sprintf('title("Gait Distance: %0.2f %s, Speed = %0.2f")',numgait_d(k), gaitline, speed));
end

%% Plot basin figure

AllBasins = -90:10:90;

lincolor = jet(length(AllBasins))';
figure();
for i = 1:length(initial_angle)
    k = getColor(steady_basin, i);
    rectangle('Position',[gait_d(i)-0.25/2 round2ten(initial_angle(i)) 0.25 10],'FaceColor',lincolor(:,k));
    hold on;
end

set(gca,'xtick',unique(gait_d));
ylim([-100 inf]);
set(gca,'ytick',-100:20:100);
grid on; ylabel('\theta (^o)','FontSize',14);
eval(sprintf('xlabel("Gait Distance d for %s","FontSize",14)', gaitline));
cmap = colormap(jet(length(AllBasins)));
cc = colorbar;
cc.Ticks = linspace(0,1,length(AllBasins));
cc.TickLabels = AllBasins;
cc.Label.String = 'Steady Basin Angle';

for i = 1:length(gait_d)
    k = getColor(steady_basin, i);
    errorbar(gait_d(i),steady_basin(i),min_steady_error(i),max_steady_error(i),'s','MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',lincolor(:,k),'LineWidth',1);
    hold on;
end

eval(sprintf('title("Basin of Attraction Plot at Speed = %0.2f")', speed));
set(gcf,'color','w');

%% GetColor Function

function k = getColor(steady_basin, i)
    list = -90:10:90;
    for j = 1:length(list)
        if list(j) == steady_basin(i)
            k = j;
            break
        end
    end
end

%% Round to nearest 10

function k = round2ten(num)
    
    if(mod(num,10)>=5.0)
        k = ceil(num/10)*10;
    else
        k = floor(num/10)*10;
    end

    if k > 90
        k = 90;
    end
    if k < 0
        k = 0;
    end 
end

