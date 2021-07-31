clear all;
close all;
DefaultGait = {[1,2],[4,3]}; % alternating [leftind,rightind]. for bound: ([LF, RF],[LB, RB])
Gait = 'pace';
%% Initial parameters setup
StepNum = 30; %total number of steps being simulated
HalfBodyLength = 0.7; %Unit 10 inches
HalfBodyWidth = 0.5; %Unit 10 inches
HalfBodyDiag = sqrt(HalfBodyLength^2+HalfBodyWidth^2);
beta_rad = atan(HalfBodyWidth/HalfBodyLength); % robot aspect ratio angle (radians)
Theta_k = 45; %degree, positive is clockwise %20; %10; %0; %70;
X_k = -0.5*100/2.54/10; %Unit 10 inches %initial position
StepLengthlist = 0.28*100/2.54/10; %Unit 10 inches

LogNum = 40;%Simulation step number
LogDiameter = 0.45;%Obstacle diameter
LogSpacing = 0.45; %Space between obstacles

resolution = 1;%resolution for initial angle
Theta_k_minus = zeros(90/resolution+1,StepNum);%array to reccord robot's angle before contact with obstacle
Theta_k_plus = zeros(90/resolution+1,StepNum);%array to reccord robot's angle after contact with obstacle
X_k_minus = zeros(90/resolution+1,StepNum);%array to reccord robot's CoM position at fore-aft direction before contact with obstacle
X_k_plus = zeros(90/resolution+1,StepNum);%array to reccord robot's CoM position at fore-aft direction after contact with obstacle
X_lateral = zeros(90/resolution+1,StepNum);%array to reccord robot's CoM position at horizontal direction after contact with obstacle
S_L = zeros(90/resolution+1,StepNum);%slide distance for one active leg
S_R = zeros(90/resolution+1,StepNum);%slide distance for another active leg
avr_angel = zeros(90/resolution+1,StepNum);%average robot's angle in one simulation with certain initial angle and certain initial position
steady_state = zeros(90/resolution+1,StepNum);%array to recorrd whether in a single simulation robot has reach the steady states or not



%% Main Loop
%X_k_minus(1,1) = X_k;
% Theta_k_minus(1,1) = Theta_k/180*pi;
%X_lateral(1,1) = 8;
hhd = 0;%array flag

%there are three for loops inside that corresponding to three different
%gaits(bound,pace,trot),annotate the other two when you are running
%simulation
for Initial_Angle = 0:resolution:90
    hhd = hhd + 1;   
    X_k_minus(hhd,1) = X_k;
    X_lateral(hhd,1) = 8;
    Theta_k_minus(hhd,1) = Initial_Angle/180*pi;
%     for i=1:StepNum %Main loop for bounding gait start
%         if (mod(i,2)==1)
%             [X_k_plus(hhd,i),Theta_k_plus(hhd,i),S_L(hhd,i),S_R(hhd,i)]=Mode1_B(X_k_minus(hhd,i), Theta_k_minus(hhd,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
%             [X_k_minus(hhd,i+1),Theta_k_minus(hhd,i+1),X_lateral(hhd,i+1)] = Mode2_B(X_k_plus(hhd,i), Theta_k_plus(hhd,i),X_lateral(hhd,i), StepLengthlist);
%         else
%             [X_k_plus(hhd,i),Theta_k_plus(hhd,i),S_L(hhd,i),S_R(hhd,i)]=Mode3_B(X_k_minus(hhd,i), Theta_k_minus(hhd,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
%             [X_k_minus(hhd,i+1),Theta_k_minus(hhd,i+1),X_lateral(hhd,i+1)] = Mode4_B(X_k_plus(hhd,i), Theta_k_plus(hhd,i),X_lateral(hhd,i), StepLengthlist);
%         end
%     end %Main loop for bounding gait end
    
    for i=1:StepNum %Main loop for trotting gait start
        if (mod(i,2)==1)
            [X_k_plus(hhd,i),Theta_k_plus(hhd,i),S_L(hhd,i),S_R(hhd,i)]=Mode1_T(X_k_minus(hhd,i), Theta_k_minus(hhd,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
            [X_k_minus(hhd,i+1),Theta_k_minus(hhd,i+1),X_lateral(hhd,i+1)] = Mode2_T(X_k_plus(hhd,i), Theta_k_plus(hhd,i),X_lateral(hhd,i), StepLengthlist);
        else
            [X_k_plus(hhd,i),Theta_k_plus(hhd,i),S_L(hhd,i),S_R(hhd,i)]=Mode3_T(X_k_minus(hhd,i), Theta_k_minus(hhd,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
            [X_k_minus(hhd,i+1),Theta_k_minus(hhd,i+1),X_lateral(hhd,i+1)] = Mode4_T(X_k_plus(hhd,i), Theta_k_plus(hhd,i),X_lateral(hhd,i), StepLengthlist);
        end
    end %Main loop for trotting gait end
    
%     for i=1:StepNum %Main loop for pacing gait start
%         if (mod(i,2)==1)
%             [X_k_plus(hhd,i),Theta_k_plus(hhd,i),S_L(hhd,i),S_R(hhd,i)]=Mode1_P(X_k_minus(hhd,i), Theta_k_minus(hhd,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
%             [X_k_minus(hhd,i+1),Theta_k_minus(hhd,i+1),X_lateral(hhd,i+1)] = Mode2_P(X_k_plus(hhd,i), Theta_k_plus(hhd,i),X_lateral(hhd,i), StepLengthlist);
%         else
%             [X_k_plus(hhd,i),Theta_k_plus(hhd,i),S_L(hhd,i),S_R(hhd,i)]=Mode3_P(X_k_minus(hhd,i), Theta_k_minus(hhd,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
%             [X_k_minus(hhd,i+1),Theta_k_minus(hhd,i+1),X_lateral(hhd,i+1)] = Mode4_P(X_k_plus(hhd,i), Theta_k_plus(hhd,i),X_lateral(hhd,i), StepLengthlist);
%         end
%     end %Main loop for pacing gait end
end



%% Obstacle field show
%show the obstacle field
[edge,slip]=ObstacleShow(LogNum,LogDiameter,LogSpacing); % initial obstacle field
for ii=0:LogNum
    figure(1111);hold on;box on;
    xlim([-300 1500]);
    xL = get(gca,'XLim');
    line(xL,[edge(1+ii*3)*10*2.54 edge(1+ii*3)*10*2.54],'Color','r');
    line(xL,[edge(2+ii*3)*10*2.54 edge(2+ii*3)*10*2.54],'Color','k');
    line(xL,[edge(3+ii*3)*10*2.54 edge(3+ii*3)*10*2.54],'Color','b');
end



%% Trajectory show
%show a certain trajectory simulation, the hhd means the initial degree,
%you can modify this parameter to show the trajectory with certain initial
%degree you want, and also there are three loops corresponding to different
%gait, remember to annotate others when you running simulation
for hhd = 14
%     for i=1:StepNum %Trajectory show for bounding start
%         PlotPose_frame_bound(1111, 0, X_lateral(hhd,i)*10*2.54, X_k_minus(hhd,i)*10*2.54, Theta_k_minus(hhd,i), 1, HalfBodyDiag*10*2.54, beta_rad, mod(i,2));
%         % visualize robot pose after slip
%         if (S_L(1,i) ~= 0 || S_R(1,i) ~= 0)
%             PlotPose_frame_bound(1111, 0, X_lateral(hhd,i)*10*2.54, X_k_plus(hhd,i)*10*2.54, Theta_k_plus(hhd,i), 2, HalfBodyDiag*10*2.54, beta_rad, mod(i,2));
%         end
%     end % Trajectory show for bounding end
    
    for i=1:StepNum %Trajectory show for trotting start
        PlotPose_frame_trot(1111, 0, X_lateral(hhd,i)*10*2.54, X_k_minus(hhd,i)*10*2.54, Theta_k_minus(hhd,i), 1, HalfBodyDiag*10*2.54, beta_rad, mod(i,2));
        % visualize robot pose after slip
        if (S_L(1,i) ~= 0 || S_R(1,i) ~= 0)
            PlotPose_frame_trot(1111, 0, X_lateral(hhd,i)*10*2.54, X_k_plus(hhd,i)*10*2.54, Theta_k_plus(hhd,i), 2, HalfBodyDiag*10*2.54, beta_rad, mod(i,2));
        end
    end % Trajectory show for trotting end
    
%     for i=1:StepNum %Trajectory show for pacing start
%         PlotPose_frame_pacing(1111, 0, X_lateral(hhd,i)*10*2.54, X_k_minus(hhd,i)*10*2.54, Theta_k_minus(hhd,i), 1, HalfBodyDiag*10*2.54, beta_rad, mod(i,2));
%         % visualize robot pose after slip
%         if (S_L(1,i) ~= 0 || S_R(1,i) ~= 0)
%             PlotPose_frame_pacing(1111, 0, X_lateral(hhd,i)*10*2.54, X_k_plus(hhd,i)*10*2.54, Theta_k_plus(hhd,i), 2, HalfBodyDiag*10*2.54, beta_rad, mod(i,2));
%         end
%     end % Trajectory show for pacing end
end



%% Angel criteria transition
%transfer angle unit from rad to degree
for hhd = 1:90/resolution+1
    for i=1:StepNum
        Theta_k_minus(hhd,i) = Theta_k_minus(hhd,i)/pi*180;
        Theta_k_plus(hhd,i) = Theta_k_plus(hhd,i)/pi*180;
    end
end

