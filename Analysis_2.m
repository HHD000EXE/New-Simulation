clear all;
close all;
DefaultGait = {[1,2],[4,3]}; % alternating [leftind,rightind]. for bound: ([LF, RF],[LB, RB])
%% Initial parameters setup
StepNum = 30; %total number of steps being simulated
HalfBodyLength = 0.7; %1/10 in
HalfBodyWidth = 0.5; %1/10 in
HalfBodyDiag = sqrt(HalfBodyLength^2+HalfBodyWidth^2);
beta_rad = atan(HalfBodyWidth/HalfBodyLength); % robot aspect ratio angle (radians)
Theta_k = 45; %degree, positive is clockwise %20; %10; %0; %70;
X_k = -0.5*100/2.54/10; %%1/10 in %-0.5*100/2.54/10; %-0.7*100/2.54/10; %-0.7*100/2.54/10; %-0.9*100/2.54/10;
StepLengthlist = 0.28*100/2.54/10; %1/10 in;  %0.15*100/2.54/10; %0.15*100/2.54/10; %0.13*100/2.54/10; %0.28*100/2.54/10;

LogNum = 40;
LogDiameter = 0.45; %1/10 in
LogSpacing = 0.40; %1/10 in   %EXPERIMENT: 0.25; %0.35; %0.45; %0.65;

resolution_angle = 1;
resolution_pos = 0.01*100/2.54/10;

SIZE = round((LogDiameter + LogSpacing)*100/2.54/10/resolution_pos+1);
Theta_k_minus = zeros(90/resolution_angle+1,SIZE,StepNum);
Theta_k_plus = zeros(90/resolution_angle+1,SIZE,StepNum);
X_k_minus = zeros(90/resolution_angle+1,SIZE,StepNum);
X_k_plus = zeros(90/resolution_angle+1,SIZE,StepNum);
X_lateral = zeros(90/resolution_angle+1,SIZE,StepNum);
S_L = zeros(90/resolution_angle+1,SIZE,StepNum);
S_R = zeros(90/resolution_angle+1,SIZE,StepNum);
avr_angel = zeros(90/resolution_angle+1,SIZE);
steady_state = zeros(90/resolution_angle+1,SIZE);
Ang_post = zeros(1,StepNum);



%% Main Loop
%X_k_minus(1,1) = X_k;
% Theta_k_minus(1,1) = Theta_k/180*pi;
%X_lateral(1,1) = 8;
Ang = 0;
for Initial_Angle = 0:resolution_angle:90
    Pos = 0;
    Ang = Ang + 1;
    for X_k = 0.9*100/2.54/10:resolution_pos:1.8*100/2.54/10
        Pos = Pos + 1;
        X_k_minus(Ang,Pos,1) = X_k;
        X_lateral(Ang,Pos,1) = 8;
        Theta_k_minus(Ang,Pos,1) = Initial_Angle/180*pi;
%         for i=1:StepNum %Main loop for bounding gait start
%             if (mod(i,2)==1)
%                 [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode1_B(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
%                 [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode2_B(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist);
%             else
%                 [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode3_B(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
%                 [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode4_B(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist);
%             end
%         end %Main loop for bounding gait end
        
        for i=1:StepNum %Main loop for trotting gait start
            if (mod(i,2)==1)
                [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode1_T(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
                [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode2_T(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist);
            else
                [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode3_T(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
                [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode4_T(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist);
            end
        end %Main loop for trotting gait end
        
%         for i=1:StepNum %Main loop for pacing gait start
%             if (mod(i,2)==1)
%                 [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode1_P(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
%                 [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode2_P(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist);
%             else
%                 [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode3_P(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
%                 [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode4_P(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist);
%             end
%         end %Main loop for pacing gait end
    end
end



%% Obstacle field show
[edge,slip]=ObstacleShow(LogNum,LogDiameter,LogSpacing); % initial obstacle field
% for ii=0:LogNum
%     figure(1111);hold on;box on;
%     xlim([-300 1500]);
%     xL = get(gca,'XLim');
%     line(xL,[edge(1+ii*3)*10*2.54 edge(1+ii*3)*10*2.54],'Color','r');
%     line(xL,[edge(2+ii*3)*10*2.54 edge(2+ii*3)*10*2.54],'Color','k');
%     line(xL,[edge(3+ii*3)*10*2.54 edge(3+ii*3)*10*2.54],'Color','b');
% end
h=figure(1111);
set(h,'visible','off');
%% Trajectory show
for Ang = 1:90/resolution_angle+1
    for Pos = 1:(LogDiameter + LogSpacing)*100/2.54/10/resolution_pos+1
%         for ii=0:LogNum
%             hold on;box on;
%             xlim([-300 1500]);
%             xL = get(gca,'XLim');
%             line(xL,[edge(1+ii*3)*10*2.54 edge(1+ii*3)*10*2.54],'Color','r');
%             line(xL,[edge(2+ii*3)*10*2.54 edge(2+ii*3)*10*2.54],'Color','k');
%             line(xL,[edge(3+ii*3)*10*2.54 edge(3+ii*3)*10*2.54],'Color','b');
%         end
%         for i=1:StepNum %Trajectory show for bounding start
%             PlotPose_frame_bound(1111, 0, X_lateral(Ang,Pos,i)*10*2.54, X_k_minus(Ang,Pos,i)*10*2.54, Theta_k_minus(Ang,Pos,i), 1, HalfBodyDiag*10*2.54, beta_rad, mod(i,2));
%             % visualize robot pose after slip
%             if (S_L(Ang,Pos,i) ~= 0 || S_R(Ang,Pos,i) ~= 0)
%                 PlotPose_frame_bound(1111, 0, X_lateral(Ang,Pos,i)*10*2.54, X_k_plus(Ang,Pos,i)*10*2.54, Theta_k_plus(Ang,Pos,i), 2, HalfBodyDiag*10*2.54, beta_rad, mod(i,2));
%             end
%         end % Trajectory show for bounding end

        
%         for i=1:StepNum %Trajectory show for trotting start
%             PlotPose_frame_trot(1111, 0, X_lateral(Ang,Pos,i)*10*2.54, X_k_minus(Ang,Pos,i)*10*2.54, Theta_k_minus(Ang,Pos,i), 1, HalfBodyDiag*10*2.54, beta_rad, mod(i,2));
%             % visualize robot pose after slip
%             if (S_L(Ang,Pos,i) ~= 0 || S_R(Ang,Pos,i) ~= 0)
%                 PlotPose_frame_trot(1111, 0, X_lateral(Ang,Pos,i)*10*2.54, X_k_plus(Ang,Pos,i)*10*2.54, Theta_k_plus(Ang,Pos,i), 2, HalfBodyDiag*10*2.54, beta_rad, mod(i,2));
%             end
%         end % Trajectory show for trotting end
%             set(gcf,'outerposition',get(0,'screensize'));
%             saveas(gcf,['D:\MATLAB\New_Simulation_Materials(4 inches)\Trajectory\Trotting\',strcat(strcat('Pos_',num2str(Pos)),strcat('Angle_',num2str(Ang))),'.jpg']);
%             clf(1111);
        
%         for i=1:StepNum %Trajectory show for pacing start
%             PlotPose_frame_pacing(1111, 0, X_lateral(Ang,Pos,i)*10*2.54, X_k_minus(Ang,Pos,i)*10*2.54, Theta_k_minus(Ang,Pos,i), 1, HalfBodyDiag*10*2.54, beta_rad, mod(i,2));
%             % visualize robot pose after slip
%             if (S_L(Ang,Pos,i) ~= 0 || S_R(Ang,Pos,i) ~= 0)
%                 PlotPose_frame_pacing(1111, 0, X_lateral(Ang,Pos,i)*10*2.54, X_k_plus(Ang,Pos,i)*10*2.54, Theta_k_plus(Ang,Pos,i), 2, HalfBodyDiag*10*2.54, beta_rad, mod(i,2));
%             end
%         end % Trajectory show for pacing end
    end
end



%% Angel criteria transition
for Ang = 1:90/resolution_angle+1
    for Pos = 1:(LogDiameter + LogSpacing)*100/2.54/10/resolution_pos+1
        for i=1:StepNum
            Theta_k_minus(Ang,Pos,i) = Theta_k_minus(Ang,Pos,i)/pi*180;
            Theta_k_plus(Ang,Pos,i) = Theta_k_plus(Ang,Pos,i)/pi*180;
        end
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    State analyze    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for Ang = 1:90/resolution_angle+1
%     for Pos = 1:(LogDiameter + LogSpacing)*100/2.54/10/resolution_pos+1
%         temp_angle=0;
%         for i=1:StepNum
%             temp_angle = temp_angle+Theta_k_plus(Ang,Pos,i);
%         end
%         avr_angel(Ang,Pos) = temp_angle/30;
%     end
% end
% 
% for Ang = 1:90/resolution_angle+1
%     for Pos = 1:(LogDiameter + LogSpacing)*100/2.54/10/resolution_pos+1
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
% %Show single Black-Green color bar
% colormatrix = zeros(90,1);
% for kk=1:90
%     colormatrix(kk,1)=kk/90;
% end
% mycolor =[zeros(90,1),colormatrix(:,1),zeros(90,1)];
% colormap(mycolor);
% colorbar
% 
% % Angel3D(steady_state,resolution_angle,resolution_pos,1); %plot robot steady states
% time = 1:StepNum;
% for Ang = 1:90/resolution_angle+1
%     for Pos = 1:(LogDiameter + LogSpacing)*100/2.54/10/resolution_pos+1
%         Ang_post(1,:) = Theta_k_plus(Ang,Pos,:);
%         if steady_state(Ang,Pos)==1
%             figure(2);hold on;
%             plot(time,Ang_post,'-*','color',[0 Ang/90 0]);
% %             xlabel('StepNum');
% %             ylabel('Robot Orientation(бу)');
%         end
%         if steady_state(Ang,Pos)==0.5
%             figure(3);hold on;
%             plot(time,Ang_post,'-*g');
%         end
%         if steady_state(Ang,Pos)==0
%             figure(4);hold on;
%             plot(time,Ang_post,'-*k');
%         end
%     end
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Zbar display    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot zbar
h=figure(2);
set(h,'visible','off');
%set(gca,'ydir','reverse');
ylabel('Relative contact position(Cm)','fontsize',12);
xlabel('Number of step','fontsize',12);
set(gca,'fontsize',12);
for Ang = 1:90/resolution_angle+1
    for Pos = 1:(LogDiameter + LogSpacing)*100/2.54/10/resolution_pos+1
        ylim([-0.2*10*2.54 1.0*10*2.54]); xlim([0 30]);
        line([0 StepNum],[0 0]);
        line([0 StepNum],[LogDiameter/2*10*2.54 LogDiameter/2*10*2.54]);
        line([0 StepNum],[LogDiameter*10*2.54   LogDiameter*10*2.54]);
        line([0 StepNum],[(LogDiameter+LogSpacing)*10*2.54   (LogDiameter+LogSpacing)*10*2.54]);
%         pos_show = 2;
%         Ang_show = 3;
        for i=1:StepNum
            box on;hold all;
            if mod(i,2)==1 %First pair
%                 plot(i,mod((X_k_minus(Ang_show,pos_show,i) + HalfBodyDiag*cos(beta_rad-Theta_k_minus(Ang_show,pos_show,i)/180*pi))*10*2.54,(LogDiameter+LogSpacing)*10*2.54),'r*','markersize',10,'linewidth',2);%10, 2
%                 plot(i,mod((X_k_minus(Ang_show,pos_show,i) + HalfBodyDiag*cos(beta_rad+Theta_k_minus(Ang_show,pos_show,i)/180*pi))*10*2.54,(LogDiameter+LogSpacing)*10*2.54),'c*','markersize',10,'linewidth',1.5);
                plot(i,mod((X_k_minus(Ang,Pos,i) + HalfBodyDiag*cos(beta_rad-Theta_k_minus(Ang,Pos,i)/180*pi))*10*2.54,(LogDiameter+LogSpacing)*10*2.54),'r*','markersize',10,'linewidth',2);%10, 2
                plot(i,mod((X_k_minus(Ang,Pos,i) - HalfBodyDiag*cos(beta_rad-Theta_k_minus(Ang,Pos,i)/180*pi))*10*2.54,(LogDiameter+LogSpacing)*10*2.54),'c*','markersize',8,'linewidth',2); %10, 2
            else %Second pair
%                 plot(i,mod((X_k_minus(Ang_show,pos_show,i) - HalfBodyDiag*cos(beta_rad-Theta_k_minus(Ang_show,pos_show,i)/180*pi))*10*2.54,(LogDiameter+LogSpacing)*10*2.54),'bo','markersize',8,'linewidth',2); %10, 2
%                 plot(i,mod((X_k_minus(Ang_show,pos_show,i) - HalfBodyDiag*cos(beta_rad+Theta_k_minus(Ang_show,pos_show,i)/180*pi))*10*2.54,(LogDiameter+LogSpacing)*10*2.54),'mo','markersize',8,'linewidth',1.5);
                plot(i,mod((X_k_minus(Ang,Pos,i) + HalfBodyDiag*cos(beta_rad+Theta_k_minus(Ang,Pos,i)/180*pi))*10*2.54,(LogDiameter+LogSpacing)*10*2.54),'mo','markersize',10,'linewidth',1.5);
                plot(i,mod((X_k_minus(Ang,Pos,i) - HalfBodyDiag*cos(beta_rad+Theta_k_minus(Ang,Pos,i)/180*pi))*10*2.54,(LogDiameter+LogSpacing)*10*2.54),'bo','markersize',8,'linewidth',1.5);
            end
        end
        set(gcf,'outerposition',get(0,'screensize'));
        saveas(gcf,['D:\MATLAB\New_Simulation_Materials(4 inches)\Contact_position\Trotting\',strcat(strcat('Pos_',num2str(Pos)),strcat('Angle_',num2str(Ang))),'.jpg']);
        clf(2);
    end
end
% legend('LF','RF','LB','RB','LF','RF','RB','LB');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Orientation vs step    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot orientation vs step
% figure(3);hold on;grid on;
% time = 1:StepNum;
% pos_show = 2;
% Ang_show = 2;
% Ang_post(1,:) = Theta_k_plus(Ang_show,pos_show,:);
% plot(time,Ang_post,'-*r');
% xlabel('StepNum');
% ylabel('Orientation(бу)')
% axis([1,20,0,90]);