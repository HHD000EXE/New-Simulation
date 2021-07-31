clear all;
DefaultGait = {[1,2],[4,3]}; % alternating [leftind,rightind]. for bound: ([LF, RF],[LB, RB])
%% Initial parameters setup
clear all;
%% Initial parameters setup
StepNum = 20; %total number of steps being simulated
HalfBodyLength = 7.8; %cm
HalfBodyWidth = 7.75; %cm
HalfBodyDiag = sqrt(HalfBodyLength^2+HalfBodyWidth^2);
beta_rad = atan(HalfBodyWidth/HalfBodyLength); % robot aspect ratio angle (radians)
StepLengthlist =9.1; %cm for    %Pacing gait 9~9.5cm     %trotting gait 13.2cm    %bounding gait 13.5cm

LogNum = 14;
LogDiameter = 4.80; %cm
D = LogDiameter;
LogSpacing = 4.0; %cm 
P = LogSpacing;
X_k_ini = P + D; %cm

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



%% Main Loop
%X_k_minus(1,1) = X_k;
% Theta_k_minus(1,1) = Theta_k/180*pi;
%X_lateral(1,1) = 8;
Ang = 0;gait = 3; % 1 is bounding, 2 is trotting, and 3 is pacing
for Initial_Angle = 0:resolution_angle:90
    Pos = 0;
    Ang = Ang + 1;
    for X_k = LogDiameter + LogSpacing:resolution_pos:2*(LogDiameter + LogSpacing)
        Pos = Pos + 1;
        X_k_minus(Ang,Pos,1) = X_k_ini+resolution_pos*Pos;
        X_lateral(Ang,Pos,1) = 8;
        Theta_k_minus(Ang,Pos,1) = Initial_Angle/180*pi;
        if(gait==1)
            for i=1:StepNum %Main loop for bounding gait start
                if (mod(i,2)==1)
                    [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode1_B(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
                    [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode2_B(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist);
                else
                    [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode3_B(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
                    [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode4_B(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist);
                end
            end %Main loop for bounding gait end
        elseif(gait==2)
            for i=1:StepNum %Main loop for trotting gait start
                if (mod(i,2)==1)
                    [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode1_T(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
%                     Theta_k_plus(Ang,Pos,i) = Theta_k_plus(Ang,Pos,i) - 1/180*pi*cos(Theta_k_plus(Ang,Pos,i));
                    [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode2_T(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist);
                else
                    [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode3_T(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
%                     Theta_k_plus(Ang,Pos,i) = Theta_k_plus(Ang,Pos,i) + 1/180*pi*cos(Theta_k_plus(Ang,Pos,i));
                    [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode4_T(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist);
                end
            end %Main loop for trotting gait end
        elseif(gait==3)
            for i=1:StepNum %Main loop for pacing gait start
                if (mod(i,2)==1)
                    [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode1_P(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
                    Theta_k_plus(Ang,Pos,i) = Theta_k_plus(Ang,Pos,i) - 8/180*pi;
                    [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode2_P(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist);
                    
                else
                    [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode3_P(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
                    Theta_k_plus(Ang,Pos,i) = Theta_k_plus(Ang,Pos,i) + 8/180*pi;
                    [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode4_P(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist);
                    
                end
            end %Main loop for pacing gait end
        end
    end
end



%% Obstacle field show
[edge,slip]=ObstacleShow(LogNum,LogDiameter,LogSpacing); % initial obstacle field
for ii=0:LogNum
    figure(1111);hold on;box on;
    xlim([-100 300]);
    ylim([-30 150]);
    zlim([-40 100]);
    xL = get(gca,'XLim');
    plot([-300,300],[ii*(P+D)+0,ii*(P+D)+0],'r','linewidth',0.5);
    plot([-300,300],[ii*(P+D)+D/2,ii*(P+D)+D/2],'k','linewidth',0.5);
    plot([-300,300],[ii*(P+D)+D,ii*(P+D)+D],'b','linewidth',0.5);
    x=[-300 300 300 -300];y=[ii*(P+D)+0 ii*(P+D)+0 ii*(P+D)+D/2 ii*(P+D)+D/2];fill(x,y,'r','FaceAlpha',0.1,'Edgecolor','none');
    x=[-300 300 300 -300];y=[ii*(P+D)+D/2 ii*(P+D)+D/2 ii*(P+D)+D ii*(P+D)+D];fill(x,y,'b','FaceAlpha',0.1,'Edgecolor','none');
end
%h=figure(1111);
% set(h,'visible','off');

%% Trajectory show
for Ang = 30%1:90/resolution_angle+1
    for Pos = 15%1:(LogDiameter + LogSpacing)/resolution_pos+1
%         for ii=0:LogNum
%             hold on;box on;
%             xlim([-300 1500]);
%             xL = get(gca,'XLim');
%             line(xL,[edge(1+ii*3) edge(1+ii*3)],'Color','r');
%             line(xL,[edge(2+ii*3) edge(2+ii*3)],'Color','k');
%             line(xL,[edge(3+ii*3) edge(3+ii*3)],'Color','b');
%         end
        if(gait == 1)
            for i=1:StepNum %Trajectory show for bounding start
                PlotPose_frame_bound(1111, 0, X_lateral(Ang,Pos,i), X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), 1, HalfBodyDiag, beta_rad, mod(i,2));
                % visualize robot pose after slip
                if (S_L(Ang,Pos,i) ~= 0 || S_R(Ang,Pos,i) ~= 0)
                    PlotPose_frame_bound(1111, 0, X_lateral(Ang,Pos,i), X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i), 2, HalfBodyDiag, beta_rad, mod(i,2));
                end
            end % Trajectory show for bounding end
        end
        %             set(gcf,'outerposition',get(0,'screensize'));
        %             saveas(gcf,['D:\MATLAB\New_Simulation_Materials(4 inches)\Trajectory\Bounding\',strcat(strcat('Pos_',num2str(Pos)),strcat('Angle_',num2str(Ang))),'.jpg']);
        %             clf(1111);
        if(gait == 2)
                for i=1:StepNum %Trajectory show for trotting start
                    PlotPose_frame_trot(1111, 0, X_lateral(Ang,Pos,i), X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), 1, HalfBodyDiag, beta_rad, mod(i,2));
                    % visualize robot pose after slip
                    if (S_L(Ang,Pos,i) ~= 0 || S_R(Ang,Pos,i) ~= 0)
                        PlotPose_frame_trot(1111, 0, X_lateral(Ang,Pos,i), X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i), 2, HalfBodyDiag, beta_rad, mod(i,2));
                    end
                end % Trajectory show for trotting end
        end
        if(gait == 3)
                for i=1:StepNum %Trajectory show for pacing start
                    PlotPose_frame_pacing(1111, 0, X_lateral(Ang,Pos,i), X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), 1, HalfBodyDiag, beta_rad, mod(i,2));
                    %visualize robot pose after slip
                    if (S_L(Ang,Pos,i) ~= 0 || S_R(Ang,Pos,i) ~= 0)
                        PlotPose_frame_pacing(1111, 0, X_lateral(Ang,Pos,i), X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i), 2, HalfBodyDiag, beta_rad, mod(i,2));
                    end
                end % Trajectory show for pacing end
        end
    end
end
    xlim([-100 300]);
    ylim([-30 150]);
    zlim([-40 100]);


%% Angel criteria transition
for Ang = 1:90/resolution_angle+1
    for Pos = 1:(LogDiameter + LogSpacing)/resolution_pos+1
        for i=1:StepNum
            Theta_k_minus(Ang,Pos,i) = Theta_k_minus(Ang,Pos,i)/pi*180;
            Theta_k_plus(Ang,Pos,i) = Theta_k_plus(Ang,Pos,i)/pi*180;
        end
    end
end



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    State analyze    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for Ang = 1:90/resolution_angle+1
%     for Pos = 1:(LogDiameter + LogSpacing)/resolution_pos+1
%         temp_angle=0;
%         for i=1:StepNum
%             temp_angle = temp_angle+Theta_k_plus(Ang,Pos,i);
%         end
%         avr_angel(Ang,Pos) = temp_angle/30;
%     end
% end
% 
% for Ang = 1:90/resolution_angle+1
%     for Pos = 1:(LogDiameter + LogSpacing)/resolution_pos+1
%         state_flag=1;
%         for i= StepNum-8:StepNum
%             if abs(Theta_k_plus(Ang,Pos,i) - Theta_k_plus(Ang,Pos,StepNum-10))>5
%                 state_flag=0.5;
%             end
%             if state_flag==0.5
%                 if abs(Theta_k_plus(Ang,Pos,i)+Theta_k_plus(Ang,Pos,i-1)+Theta_k_plus(Ang,Pos,i-2)+Theta_k_plus(Ang,Pos,i-3)+Theta_k_plus(Ang,Pos,i-4)-5*avr_angel(Ang,Pos))>25
%                     state_flag=0;
%                 end
%             end
%         end
%         if state_flag==1
%             steady_state(Ang,Pos)=1;
%         elseif state_flag==0.5
%             steady_state(Ang,Pos)=0.5;
%         else
%             steady_state(Ang,Pos)=0;
%         end
%     end
% end
% 
% % Angel3D(steady_state,resolution_angle,resolution_pos,1); %plot robot steady states
% time = 1:StepNum;
% for Ang = 1:90/resolution_angle+1
%     for Pos = 1:(LogDiameter + LogSpacing)/resolution_pos+1
%         Ang_post(1,:) = Theta_k_plus(Ang,Pos,:);
%         if steady_state(Ang,Pos)==1
%             figure(2);hold on;
%             plot(time,Ang_post,'-*','color',[0 Ang/91 0]);
%             xlabel('StepNum');
%             ylabel('Robot Orientation(¡ã)');
%         end
%         if steady_state(Ang,Pos)==0.5
%             figure(2);hold on;
%             plot(time,Ang_post,'-*','color',[1 0 0]);
%         end
%         if steady_state(Ang,Pos)==0
%             figure(2);hold on;
%             plot(time,Ang_post,'-*','color',[0 0 1]);
%         end
%     end
% end
%
% %Show single Black-Green color bar
% colormatrix = zeros(91,1);
% for kk=1:91
%     colormatrix(kk,1)=kk/91;
% end
% mycolor =[zeros(91,1),colormatrix(:,1),zeros(91,1)];
% colormap(mycolor);
% caxis([0 90]);
% colorbar
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Zbar display    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%h=figure(2);
% set(h,'visible','off');
% set(gca,'ydir','reverse');
figure(2);
ylabel('Relative contact position(Cm)','fontsize',18);
xlabel('Number of step','fontsize',18);
set(gca,'fontsize',18);
% for Ang = 1:90/resolution_angle+1
%     for Pos = 1:(LogDiameter + LogSpacing)/resolution_pos+1
ylim([-1 LogDiameter+LogSpacing+1]); xlim([0 StepNum]);
line([0 StepNum],[0 0]);
line([0 StepNum],[LogDiameter/2 LogDiameter/2]);
line([0 StepNum],[LogDiameter   LogDiameter]);
line([0 StepNum],[(LogDiameter+LogSpacing)   (LogDiameter+LogSpacing)]);
pos_show = 15;
Ang_show = 30;
for i=1:StepNum
    box on;hold all;
    if mod(i,2)==1 %First pair
                         plot(i,mod((X_k_minus(Ang_show,pos_show,i) + HalfBodyDiag*cos(beta_rad-Theta_k_minus(Ang_show,pos_show,i)/180*pi)),(LogDiameter+LogSpacing)),'r*','markersize',10,'linewidth',2);%bounding
                         plot(i,mod((X_k_minus(Ang_show,pos_show,i) + HalfBodyDiag*cos(beta_rad+Theta_k_minus(Ang_show,pos_show,i)/180*pi)),(LogDiameter+LogSpacing)),'c*','markersize',10,'linewidth',1.5);%bounding
%                         plot(i,mod((X_k_minus(Ang_show,pos_show,i) + HalfBodyDiag*cos(beta_rad-Theta_k_minus(Ang_show,pos_show,i)/180*pi)),(LogDiameter+LogSpacing)),'r*','markersize',10,'linewidth',2);%Trotting
%                         plot(i,mod((X_k_minus(Ang_show,pos_show,i) - HalfBodyDiag*cos(beta_rad-Theta_k_minus(Ang_show,pos_show,i)/180*pi)),(LogDiameter+LogSpacing)),'c*','markersize',8,'linewidth',2); %Trotting
%                         plot(i,mod((X_k_minus(Ang_show,pos_show,i) + HalfBodyDiag*cos(beta_rad+Theta_k_minus(Ang_show,pos_show,i)/180*pi)),(LogDiameter+LogSpacing)),'ko','markersize',10,'linewidth',1.5);%Trotting analysis
%                         plot(i,mod((X_k_minus(Ang_show,pos_show,i) - HalfBodyDiag*cos(beta_rad+Theta_k_minus(Ang_show,pos_show,i)/180*pi)),(LogDiameter+LogSpacing)),'ko','markersize',8,'linewidth',1.5);%Trotting analysis
%                         plot(i,mod((X_k_minus(Ang,Pos,i) + HalfBodyDiag*cos(beta_rad-Theta_k_minus(Ang,Pos,i)/180*pi)),(LogDiameter+LogSpacing)),'r*','markersize',10,'linewidth',2);%loop show
%                         plot(i,mod((X_k_minus(Ang,Pos,i) + HalfBodyDiag*cos(beta_rad+Theta_k_minus(Ang,Pos,i)/180*pi)),(LogDiameter+LogSpacing)),'c*','markersize',10,'linewidth',1.5);%loop show
%                          plot(i,mod((X_k_minus(Ang_show,pos_show,i) + HalfBodyDiag*cos(beta_rad-Theta_k_minus(Ang_show,pos_show,i)/180*pi)),(LogDiameter+LogSpacing)),'r*','markersize',10,'linewidth',2);%pacing
%                          plot(i,mod((X_k_minus(Ang_show,pos_show,i) - HalfBodyDiag*cos(beta_rad+Theta_k_minus(Ang_show,pos_show,i)/180*pi)),(LogDiameter+LogSpacing)),'c*','markersize',8,'linewidth',1.5);%pacing
    else %Second pair
                        plot(i,mod((X_k_minus(Ang_show,pos_show,i) - HalfBodyDiag*cos(beta_rad-Theta_k_minus(Ang_show,pos_show,i)/180*pi)),(LogDiameter+LogSpacing)),'bo','markersize',8,'linewidth',2);%bounding
                        plot(i,mod((X_k_minus(Ang_show,pos_show,i) - HalfBodyDiag*cos(beta_rad+Theta_k_minus(Ang_show,pos_show,i)/180*pi)),(LogDiameter+LogSpacing)),'mo','markersize',8,'linewidth',1.5);%bounding
%                         plot(i,mod((X_k_minus(Ang_show,pos_show,i) + HalfBodyDiag*cos(beta_rad+Theta_k_minus(Ang_show,pos_show,i)/180*pi)),(LogDiameter+LogSpacing)),'mo','markersize',10,'linewidth',1.5);%Trotting
%                         plot(i,mod((X_k_minus(Ang_show,pos_show,i) - HalfBodyDiag*cos(beta_rad+Theta_k_minus(Ang_show,pos_show,i)/180*pi)),(LogDiameter+LogSpacing)),'bo','markersize',8,'linewidth',1.5);%Trotting
%                         plot(i,mod((X_k_minus(Ang_show,pos_show,i) + HalfBodyDiag*cos(beta_rad-Theta_k_minus(Ang_show,pos_show,i)/180*pi)),(LogDiameter+LogSpacing)),'k*','markersize',10,'linewidth',2);%Trotting analysis
%                         plot(i,mod((X_k_minus(Ang_show,pos_show,i) - HalfBodyDiag*cos(beta_rad-Theta_k_minus(Ang_show,pos_show,i)/180*pi)),(LogDiameter+LogSpacing)),'k*','markersize',8,'linewidth',2); %Trotting analysis
%                         plot(i,mod((X_k_minus(Ang,Pos,i) - HalfBodyDiag*cos(beta_rad-Theta_k_minus(Ang,Pos,i)/180*pi)),(LogDiameter+LogSpacing)),'bo','markersize',8,'linewidth',2); %loop show
%                         plot(i,mod((X_k_minus(Ang,Pos,i) - HalfBodyDiag*cos(beta_rad+Theta_k_minus(Ang,Pos,i)/180*pi)),(LogDiameter+LogSpacing)),'mo','markersize',8,'linewidth',1.5);%loop show
%                         plot(i,mod((X_k_minus(Ang_show,pos_show,i) + HalfBodyDiag*cos(beta_rad+Theta_k_minus(Ang_show,pos_show,i)/180*pi)),(LogDiameter+LogSpacing)),'mo','markersize',10,'linewidth',1.5);%pacing
%                         plot(i,mod((X_k_minus(Ang_show,pos_show,i) - HalfBodyDiag*cos(beta_rad-Theta_k_minus(Ang_show,pos_show,i)/180*pi)),(LogDiameter+LogSpacing)),'bo','markersize',8,'linewidth',2);%pacing
    end
end
%         set(gcf,'outerposition',get(0,'screensize'));
%         saveas(gcf,['D:\MATLAB\New_Simulation_Materials(4 inches)\Contact_position\Bounding\',strcat(strcat('Pos_',num2str(Pos)),strcat('Angle_',num2str(Ang))),'.jpg']);
%         clf(2);
%     end
% end
% % legend('LF','RF','LB','RB','LF','RF','RB','LB');

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Orientation vs step    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate Orientation V.S. step for each signle trahectory
figure(3);hold on;grid on;
time = 1:StepNum;
% pos_show = 2;
% Ang_show = 18;
MarkerEdgeColors =jet(91);
for ang=1:1:70
    for pos=1:5:(LogDiameter + LogSpacing)/0.1
        Ang_post(1,:) = Theta_k_plus(ang,pos,:);
        plot(time,Ang_post,'*-','Markersize',6,'color',MarkerEdgeColors(ang,:),'linewidth',3);
    end
end
xlabel('StepNum');
ylabel('Orientation(°   )');
xlim([1 StepNum]);
colormap(jet);
caxis([0,90]);
colorbar;
set(gca,'Fontsize',20);