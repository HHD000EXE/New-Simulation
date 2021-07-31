clear all;
DefaultGait = {[1,2],[4,3]}; % alternating [leftind,rightind]. for bound: ([LF, RF],[LB, RB])
%% Initial parameters setup
clear all;
%% Initial parameters setup
StepNum = 30; %total number of steps being simulated
HalfBodyLength = 6.4; %cm
HalfBodyWidth = 7.5; %cm
HalfBodyDiag = sqrt(HalfBodyLength^2+HalfBodyWidth^2);
beta_rad = atan(HalfBodyWidth/HalfBodyLength); % robot aspect ratio angle (radians)
X_k_ini = -20; %cm
StepLengthlist =13; %cm for bounding gait

LogNum = 21;
LogDiameter = 4.8; %cm
D = LogDiameter;
LogSpacing = 3.5; %cm
P = LogSpacing;
F_rate = 0.15;

resolution_angle = 1;% decide loop size for robot initial angle
resolution_pos = 0.1;% decide loop size for robot initial position

SIZE = round((LogDiameter + LogSpacing)/resolution_pos+1); % define the size of data matrix
Theta_k_minus = zeros(90/resolution_angle+1,SIZE,StepNum); % Matrix for collecting robot orientation before each slides
Theta_k_plus = zeros(90/resolution_angle+1,SIZE,StepNum);  % Matrix for collecting robot orientation after each slides
X_k_minus = zeros(90/resolution_angle+1,SIZE,StepNum);     % Matrix for collecting robot position of CoM before each slides
X_k_plus = zeros(90/resolution_angle+1,SIZE,StepNum);      % Matrix for collecting robot position of CoM after each slides
X_lateral = zeros(90/resolution_angle+1,SIZE,StepNum);     % Matrix for collecting robot lateral position of CoM after each slides
S_L = zeros(90/resolution_angle+1,SIZE,StepNum);           % Matrix for collecting robot left leg slide distance
S_R = zeros(90/resolution_angle+1,SIZE,StepNum);           % Matrix for collecting robot right leg slide distance
avr_angel = zeros(90/resolution_angle+1,SIZE);             % Matrix for collecting robot average orientation in each trajectory
steady_state = zeros(90/resolution_angle+1,SIZE);          % Matrix for identify steay trajectory
Ang_post = zeros(1,StepNum);                               % Matrix for each step orientation in single trajectory

%% Obstacle field show
[edge,slip]=ObstacleShow(LogNum,LogDiameter,LogSpacing); % initial obstacle field
% for ii=0:LogNum
%     figure(1);hold on;box on;
%     xlim([-200 200]);
%     ylim([-30 210]);
%     zlim([-40 100]);
%     xL = get(gca,'XLim');
%     plot([-300,300],[ii*(P+D)+0,ii*(P+D)+0],'r','linewidth',0.2);
%     plot([-300,300],[ii*(P+D)+D/2,ii*(P+D)+D/2],'k','linewidth',0.2);
%     plot([-300,300],[ii*(P+D)+D,ii*(P+D)+D],'b','linewidth',0.2);
%     x=[-300 300 300 -300];y=[ii*(P+D)+0 ii*(P+D)+0 ii*(P+D)+D/2 ii*(P+D)+D/2];fill(x,y,'r','FaceAlpha',0.1,'Edgecolor','none');
%     x=[-300 300 300 -300];y=[ii*(P+D)+D/2-F_rate*D ii*(P+D)+D/2-F_rate*D ii*(P+D)+D/2+F_rate*D ii*(P+D)+D/2+F_rate*D];fill(x,y,'k','FaceAlpha',0.02,'Edgecolor','none');
%     x=[-300 300 300 -300];y=[ii*(P+D)+D/2 ii*(P+D)+D/2 ii*(P+D)+D ii*(P+D)+D];fill(x,y,'b','FaceAlpha',0.1,'Edgecolor','none');
% end
set(gca,'Fontsize',24);
xlabel('Lateral position(cm)');
ylabel('Fore-aft position(cm)');
colormap('jet');caxis([0,90]);colorbar;

hold on;
MarkerEdgeColors =jet(91);
% for angle = 0:5:70
% hold on;
% A=xlsread(strcat(strcat('D:\MATLAB\New_Simulation\New_data\Bounding_Degree_-',num2str(angle)),'_Position_-24_Spacing_3.5.csv'),1);
% xx=A(:,[7:7])*100 -30 ;
% zz=A(:,[9:9])*100;
% plot(zz,xx,'*-','Markersize',0.5,'color',MarkerEdgeColors(angle+1,:));
% end
% 
% for angle = 0:5:70
% hold on;
% A=xlsread(strcat(strcat('D:\MATLAB\New_Simulation\New_data\Bounding_Degree_-',num2str(angle)),'_Position_-24_Spacing_3.5.csv'),1);
% a=A(:,[4:4])/0.008;
% figure(2);
% plot(linspace(1,length(a),length(a)),a,'*-','Markersize',0.5,'color',MarkerEdgeColors(angle+1,:));
% end

for angle = 0:5:75
figure(1);hold on;
A = xlsread(strcat(strcat('D:\MATLAB\New_Simulation\New_data_final\Basin_Pace_degree_',num2str(angle)),'_position_5_3_(P+D).csv'));
xx=A(:,[6:6])-70;
zz=A(:,[8:8]);
for i = 1 : length(xx)
        if(xx(i,1)>180)
            cut_e(1,1) = i;
            break;
        end
        if(i==length(xx))
            cut_e(1,1) = i-300;
        end
end
cut_s(1,1) = 1;

% for j = 2 : length(xx)
%         if(xx(i,1)>10)%if(abs(xx(j,1)-xx(50,1))>1)
%             cut_s(1,1) = j;
%             break;
%         end
% end

plot(zz,xx,'*-','Markersize',0.5,'color',MarkerEdgeColors(angle+1,:));
% plot(zz,xx,'b*-','Markersize',0.5);
figure(2);grid on;hold on;
A = xlsread(strcat(strcat('D:\MATLAB\New_Simulation\New_data_final\Basin_Pace_degree_',num2str(angle)),'_position_5_3_(P+D).csv'));
a=A(cut_s(1, 1):cut_e(1, 1),[4:4]);
if(angle < 30)
    plot(linspace(1,length(a),length(a))/120.0,a,'*-','Markersize',0.5,'color',MarkerEdgeColors(angle+1,:));
%     plot(linspace(1,length(a),length(a))/120.0,a,'b*-','Markersize',0.5);
else
    plot(linspace(1,length(a),length(a))/120.0,abs(a),'*-','Markersize',0.5,'color',MarkerEdgeColors(angle+1,:));
%     plot(linspace(1,length(a),length(a))/120.0,abs(a),'b*-','Markersize',0.5);
end
% ylim([-90,90]);
end

set(gca,'Fontsize',20);
xlabel('Time(s)');
ylabel('Orientation(°)');
colormap('jet');caxis([0,90]);colorbar;
