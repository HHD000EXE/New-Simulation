clear all;
%% Initial parameters setup
StepNum = 30; %total number of steps being simulated
HalfBodyLength = 6.4; %cm
HalfBodyWidth = 7.5; %cm
HalfBodyDiag = sqrt(HalfBodyLength^2+HalfBodyWidth^2);
beta_rad = atan(HalfBodyWidth/HalfBodyLength); % robot aspect ratio angle (radians)
X_k_ini = -20; %cm
StepNum1 = 20; %Gait transit point 1
%StepNum2 = 20; %Gait transit point 2
StepLengthlist_1 = 13; %cm for bounding gait
StepLengthlist_2 = 13; %cm for trotting gait
StepLengthlist_3 = 9; %cm for pacing gait

LogNum = 40;
LogDiameter = 4.80; %cm
LogSpacing = 3.5; %cm

resolution_angle = 1;% decide loop size for robot initial angle
resolution_pos = 0.1;% decide loop size for robot initial position
Gait = ones(1,StepNum); Gait(1,1:StepNum1)=1;Gait(1,StepNum1:StepNum)=3;

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

%% Main Loop
ls=0;Ang=0;
for Initial_Angle = 45 %0:resolution_angle:90
    Pos = 0;
    Ang = Ang + 1;
    for X_k = 0 %(LogDiameter + LogSpacing):resolution_pos:2*(LogDiameter + LogSpacing)
        Pos = Pos + 1;
        X_k_minus(Ang,Pos,1) = X_k_ini;%+resolution_pos*Pos;
        X_lateral(Ang,Pos,1) = 0;
        Theta_k_minus(Ang,Pos,1) = Initial_Angle/180*pi;
        for i=1:StepNum
            gait = Gait(1,i); % 1 is bounding, 2 is trotting, and 3 is pacing
            if(gait == 1)
                if (mod(i,2)==1)    %Main loop for bounding gait start
                    [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode1_B(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
                    [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode2_B(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist_1);
                else
                    [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode3_B(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
                    [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode4_B(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist_1);
                end     %Main loop for bounding gait end
            elseif(gait == 2)
                if (mod(i,2)==1)    %Main loop for trotting gait start
                    [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode1_T(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
                    [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode2_T(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist_2);
                else
                    [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode3_T(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
                    [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode4_T(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist_2);
                end     %Main loop for trotting gait end
            else
                if (mod(i,2)==1)    %Main loop for pacing gait start
                    [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode1_P(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
                    [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode2_P(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist_3);
                else
                    [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode3_P(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
                    [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode4_P(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist_3);
                end     %Main loop for pacing gait end
            end
        end
    end
end

%% Obstacle field show
[edge,slip]=ObstacleShow(LogNum,LogDiameter,LogSpacing); % initial obstacle field
P=LogSpacing;   D=LogDiameter;
for ii=0:LogNum
    figure(1111);hold on;box on;
    xlim([-100 300]);
    ylim([-30 300]);
    zlim([-40 100]);
    xL = get(gca,'XLim');
    plot([-100,300],[ii*(P+D)+0,ii*(P+D)+0],'r','linewidth',0.5);
    plot([-100,300],[ii*(P+D)+D/2,ii*(P+D)+D/2],'k','linewidth',0.5);
    plot([-100,300],[ii*(P+D)+D,ii*(P+D)+D],'b','linewidth',0.5);
    x=[-100 300 300 -100];y=[ii*(P+D)+0 ii*(P+D)+0 ii*(P+D)+D/2 ii*(P+D)+D/2];fill(x,y,'r','FaceAlpha',0.1,'Edgecolor','none');
    x=[-100 300 300 -100];y=[ii*(P+D)+D/2 ii*(P+D)+D/2 ii*(P+D)+D ii*(P+D)+D];fill(x,y,'b','FaceAlpha',0.1,'Edgecolor','none');
end
h=figure(1111);
% set(h,'visible','off');
%% Trajectory show
Ang=1;  Pos=1;
for i=1:StepNum
    gait = Gait(1,i); % 1 is bounding, 2 is trotting, and 3 is pacing
    %             set(gcf,'outerposition',get(0,'screensize'));
    %             saveas(gcf,['D:\MATLAB\New_Simulation_Materials(4 inches)\Trajectory\Bounding\',strcat(strcat('Pos_',num2str(Pos)),strcat('Angle_',num2str(Ang))),'.jpg']);
    %             clf(1111);
    if(gait == 1)
        PlotPose_frame_bound(1111, 0, X_lateral(Ang,Pos,i), X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), 1, HalfBodyDiag, beta_rad, mod(i,2));   %Trajectory show for bounding start
        % visualize robot pose after slip
        if (S_L(Ang,Pos,i) ~= 0 || S_R(Ang,Pos,i) ~= 0)
            PlotPose_frame_bound(1111, 0, X_lateral(Ang,Pos,i), X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i), 2, HalfBodyDiag, beta_rad, mod(i,2));
        end     % Trajectory show for bounding end
    elseif(gait == 2)
        PlotPose_frame_trot(1111, 0, X_lateral(Ang,Pos,i), X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), 1, HalfBodyDiag, beta_rad, mod(i,2));    %Trajectory show for trotting start
        % visualize robot pose after slip
        if (S_L(Ang,Pos,i) ~= 0 || S_R(Ang,Pos,i) ~= 0)
            PlotPose_frame_trot(1111, 0, X_lateral(Ang,Pos,i), X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i), 2, HalfBodyDiag, beta_rad, mod(i,2));
        end     % Trajectory show for trotting end
    else
        PlotPose_frame_pacing(1111, 0, X_lateral(Ang,Pos,i), X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), 1, HalfBodyDiag, beta_rad, mod(i,2));  %Trajectory show for pacing start
        %visualize robot pose after slip
        if (S_L(Ang,Pos,i) ~= 0 || S_R(Ang,Pos,i) ~= 0)
            PlotPose_frame_pacing(1111, 0, X_lateral(Ang,Pos,i), X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i), 2, HalfBodyDiag, beta_rad, mod(i,2));
        end     % Trajectory show for pacing end
    end
end

%% Probability map


%% Angle tranfer
for Ang = 1:90/resolution_angle+1
    for Pos = 1:(LogDiameter + LogSpacing)/resolution_pos+1
        for i=1:StepNum
            Theta_k_minus(Ang,Pos,i) = Theta_k_minus(Ang,Pos,i)/pi*180;
            Theta_k_plus(Ang,Pos,i) = Theta_k_plus(Ang,Pos,i)/pi*180;
        end
    end
end


