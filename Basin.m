clear all;
%% Initial parameters setup
StepNum = 30; %total number of steps being simulated
HalfBodyLength = 8.0; %cm
HalfBodyWidth = 7.7; %cm
HalfBodyDiag = sqrt(HalfBodyLength^2+HalfBodyWidth^2);
beta_rad = atan(HalfBodyWidth/HalfBodyLength); % robot aspect ratio angle (radians)
Theta_k = 45; %degree, positive is clockwise %20; %10; %0; %70;
X_k = -0.5; %%1/10 in %-0.5; %-0.7; %-0.7; %-0.9;
StepLengthlist =12.5; %1/10 in;  %0.15; %0.15; %0.13; %0.28;

LogNum = 30;
LogDiameter = 4.80; %1/10 in
LogSpacing = 3.5; %1/10 in   %EXPERIMENT: 0.25; %0.35; %0.45; %0.65;

resolution_angle = 1;
resolution_pos = 0.1;

SIZE = round((LogDiameter + LogSpacing)/resolution_pos+1);
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

MarkerEdgeColors =jet(91);
% Basin_data = [];
% Basin_bar_data = [];
% Basin_bar_marker = [];
% Basin_data(1,1)=0;
%% Main Loop
%X_k_minus(1,1) = X_k;
% Theta_k_minus(1,1) = Theta_k/180*pi;
%X_lateral(1,1) = 8;
ls=0;gait=2;% 1 is bounding, 2 is trotting, and 3 is pacing
for LogSpacing=0:0.5:3.5
    Ang = 0;
    Basin_data = [];
    Basin_bar_data = [];
    Basin_bar_marker = [];
    Basin_data(1,1)=0;
    for Initial_Angle = 0:resolution_angle:90
        Pos = 0;
        Ang = Ang + 1;
        for X_k =(LogDiameter + LogSpacing):resolution_pos:2*(LogDiameter + LogSpacing)
            Pos = Pos + 1;
            X_k_minus(Ang,Pos,1) = -30+resolution_pos*Pos;
            X_lateral(Ang,Pos,1) = 0;
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
                        [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode2_T(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist);
                    else
                        [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode3_T(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
                        [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode4_T(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist);
                    end
                end %Main loop for trotting gait end
            elseif(gait==3)
                for i=1:StepNum %Main loop for pacing gait start
                    if (mod(i,2)==1)
                        [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode1_P(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
                        [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode2_P(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist);
                    else
                        [X_k_plus(Ang,Pos,i),Theta_k_plus(Ang,Pos,i),S_L(Ang,Pos,i),S_R(Ang,Pos,i)]=Mode3_P(X_k_minus(Ang,Pos,i), Theta_k_minus(Ang,Pos,i), LogSpacing, LogDiameter, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag);
                        [X_k_minus(Ang,Pos,i+1),Theta_k_minus(Ang,Pos,i+1),X_lateral(Ang,Pos,i+1)] = Mode4_P(X_k_plus(Ang,Pos,i), Theta_k_plus(Ang,Pos,i),X_lateral(Ang,Pos,i), StepLengthlist);
                    end
                end %Main loop for pacing gait end
            end
        end
    end
    
    for Ang = 1:90/resolution_angle+1
        for Pos = 1:(LogDiameter + LogSpacing)/resolution_pos+1
            for i=1:StepNum
                Theta_k_minus(Ang,Pos,i) = Theta_k_minus(Ang,Pos,i)/pi*180;
                Theta_k_plus(Ang,Pos,i) = Theta_k_plus(Ang,Pos,i)/pi*180;
            end
        end
    end
    
    for Ang = 1:90/resolution_angle+1
        for Pos = 1:(LogDiameter + LogSpacing)/resolution_pos+1
            temp_angle=0;
            for i=1:StepNum
                temp_angle = temp_angle+Theta_k_plus(Ang,Pos,i);
            end
            avr_angel(Ang,Pos) = temp_angle/30;
        end
    end
    
    for Ang = 1:90/resolution_angle+1
        for Pos = 1:(LogDiameter + LogSpacing)/resolution_pos+1
            state_flag=1;
            for i= StepNum-8:StepNum
                if abs(Theta_k_plus(Ang,Pos,i) - Theta_k_plus(Ang,Pos,StepNum-10))>5
                    state_flag=0.5;
                end
                if state_flag==0.5
                    if abs(Theta_k_plus(Ang,Pos,i)+Theta_k_plus(Ang,Pos,i-1)+Theta_k_plus(Ang,Pos,i-2)+Theta_k_plus(Ang,Pos,i-3)+Theta_k_plus(Ang,Pos,i-4)-5*avr_angel(Ang,Pos))>25
                        state_flag=0;
                    end
                end
            end
            if state_flag==1
                steady_state(Ang,Pos)=1;
            elseif state_flag==0.5
                steady_state(Ang,Pos)=0.5;
            else
                steady_state(Ang,Pos)=0;
            end
        end
    end
    time = 1:StepNum;
    for Ang = 1:90/resolution_angle+1
        for Pos = 1:(LogDiameter + LogSpacing)/resolution_pos+1
            Ang_post(1,:) = Theta_k_plus(Ang,Pos,:);
            if steady_state(Ang,Pos)==1
                basin_steady_new=1;
                for kk=1:length(Basin_data(:,1))
                    if(abs(Ang_post(1,StepNum)-mean_sp(Basin_data(kk,:)))<5)
                        Basin_data(kk,end+1)=Ang_post(1,StepNum);
                        Basin_data(1:kk-1,end)=-180;
                        Basin_data(kk+1:length(Basin_data(:,1)),end)=-180;
                        Basin_bar_data(kk,end+1)=Theta_k_minus(Ang,Pos,1);
                        Basin_bar_data(1:kk-1,end)=-180;
                        Basin_bar_data(kk+1:length(Basin_data(:,1)),end)=-180;
                        basin_steady_new = 0;
                        break;
                    end
                end
                if(basin_steady_new == 1)
                    Basin_data(end+1,1)=Ang_post(1,StepNum);
                    Basin_data(end,2:end)=-180;
                    Basin_bar_data(end+1,1)=Theta_k_minus(Ang,Pos,1);
                    Basin_bar_data(end,2:end)=-180;
                end
            end
        end
    end
    %% Bar Marker
    for kk=1:length(Basin_bar_data(:,1))
        Basin_bar_marker(kk,1)=min(abs(Basin_bar_data(kk,:)));
        Basin_bar_marker(kk,2)=max(Basin_bar_data(kk,:));
    end
    %% Standard Deviation
        for kk=1:length(Basin_data(:,1))
            Basin_data_std(kk)=max(Basin_data(kk,:))-min(abs(Basin_data(kk,:)));
        end
    
    %% figure
    hhd_count=[];
    for t=1:length(Basin_bar_marker(:,1))
        if (((Basin_data(t,1)==0 && Basin_bar_marker(1,2)~=0) || (Basin_data(t,1)>0 && Basin_data(t,1)<80)) && (Basin_bar_marker(t,1)~=Basin_bar_marker(t,2)))
            hhd_count(1,end+1)=t;
        end
    end
    for t=1:length(hhd_count)
        figure(1);hold on;grid on;
        xlim([-0.3 5.5]);
%                 ylim([-40 90]); %% for compound gait figure
%                 x=[0.5*2/3+0+ls*0.5 0.5*2/3+0.5/3+ls*0.5 0.5*2/3+0.5/3+ls*0.5 0.5*2/3+0+ls*0.5];
%                 x=[0.5/3+0+ls*0.5 0.5/3+0.5/3+ls*0.5 0.5/3+0.5/3+ls*0.5 0.5/3+0+ls*0.5];
%                 x=[ls*0.5-length(hhd_count)*0.5/16+t*0.5/8-0.5/8 ls*0.5-length(hhd_count)*0.5/16+t*0.5/8 ls*0.5-length(hhd_count)*0.5/16+t*0.5/8 ls*0.5-length(hhd_count)*0.5/16+t*0.5/8-0.5/8];
%                 y=[Basin_bar_marker(hhd_count(1,t),1) Basin_bar_marker(hhd_count(1,t),1) Basin_bar_marker(hhd_count(1,t),2) Basin_bar_marker(hhd_count(1,t),2)]; %end compound gait figure
%         if(gait==1)
%             x=[0+ls*0.5, 0.5/4+ls*0.5, 0.5/4+ls*0.5, 0+ls*0.5];
%         elseif(gait==2)
%             x=[0.5/4+ls*0.5, 0.5*2/4+ls*0.5, 0.5*2/4+ls*0.5, 0.5/4+ls*0.5];
%         elseif(gait==3)
%             x=[0.5*2/4+ls*0.5, 0.5*3/4+ls*0.5, 0.5*3/4+ls*0.5, 0.5*2/4+ls*0.5];
%         end
        x=[0+ls*0.5, 0.5/2+ls*0.5, 0.5/2+ls*0.5, 0+ls*0.5];
        y=[Basin_bar_marker(hhd_count(1,t),1) Basin_bar_marker(hhd_count(1,t),1) Basin_bar_marker(hhd_count(1,t),2) Basin_bar_marker(hhd_count(1,t),2)];
%         if t==1
%             fill(x,y,'r','FaceAlpha',0.6);
%         elseif t==2
%             fill(x,y,[1 0.8431 0],'FaceAlpha',0.6);
%         elseif t==3
%             fill(x,y,[0.513 0.545 0.545],'FaceAlpha',0.6);
%         else
%             fill(x,y,[0.109 0.525 0.933],'FaceAlpha',0.6);
%         end

        if t==1
%             fill(x,y,[0.235 0.701 0.443],'FaceAlpha',0.6);
            fill(x,y,MarkerEdgeColors(round(Basin_data(hhd_count(1,t),1))+1,:),'FaceAlpha',0.2);
            if(gait==1)
                plot(ls*0.5,Basin_data(hhd_count(1,t),1),'Color', 'k','Marker','*','MarkerFaceColor',MarkerEdgeColors(round(Basin_data(hhd_count(1,t),1))+1,:),'MarkerSize',18,'linewidth',2);
            elseif(gait==2)
                plot(ls*0.5,Basin_data(hhd_count(1,t),1),'Color', 'k','Marker','o','MarkerFaceColor',MarkerEdgeColors(round(Basin_data(hhd_count(1,t),1))+1,:),'MarkerSize',18,'linewidth',2);
            elseif(gait==3)
                plot(ls*0.5,Basin_data(hhd_count(1,t),1),'Color', 'k','Marker','p','MarkerFaceColor',MarkerEdgeColors(round(Basin_data(hhd_count(1,t),1))+1,:),'MarkerSize',18,'linewidth',2);
            end
            %plot(0.5/4+ls*0.5,Basin_data(hhd_count(1,t),1),'go','MarkerSize',18,'linewidth',2);
        elseif t==2
%             fill(x,y,[1 0.8431 0],'FaceAlpha',0.6);
            fill(x,y,MarkerEdgeColors(round(Basin_data(hhd_count(1,t),1))+1,:),'FaceAlpha',0.2);
            if(gait==1)
                plot(ls*0.5,Basin_data(hhd_count(1,t),1),'Color', 'k','Marker','*','MarkerFaceColor',MarkerEdgeColors(round(Basin_data(hhd_count(1,t),1))+1,:),'MarkerSize',18,'linewidth',2);
            elseif(gait==2)
                plot(ls*0.5,Basin_data(hhd_count(1,t),1),'Color', 'k','Marker','o','MarkerFaceColor',MarkerEdgeColors(round(Basin_data(hhd_count(1,t),1))+1,:),'MarkerSize',18,'linewidth',2);
            elseif(gait==3)
                plot(ls*0.5,Basin_data(hhd_count(1,t),1),'Color', 'k','Marker','p','MarkerFaceColor',MarkerEdgeColors(round(Basin_data(hhd_count(1,t),1))+1,:),'MarkerSize',18,'linewidth',2);
            end
            %plot(0.5/4+ls*0.5,Basin_data(hhd_count(1,t),1),'go','MarkerSize',18,'linewidth',2);
        elseif t==3
%             fill(x,y,[0.513 0.545 0.545],'FaceAlpha',0.6);
            fill(x,y,MarkerEdgeColors(round(Basin_data(hhd_count(1,t),1))+1,:),'FaceAlpha',0.2);
            if(gait==1)
                plot(ls*0.5,Basin_data(hhd_count(1,t),1),'Color', 'k','Marker','*','MarkerFaceColor',MarkerEdgeColors(round(Basin_data(hhd_count(1,t),1))+1,:),'MarkerSize',18,'linewidth',2);
            elseif(gait==2)
                plot(ls*0.5,Basin_data(hhd_count(1,t),1),'Color', 'k','Marker','o','MarkerFaceColor',MarkerEdgeColors(round(Basin_data(hhd_count(1,t),1))+1,:),'MarkerSize',18,'linewidth',2);
            elseif(gait==3)
                plot(ls*0.5,Basin_data(hhd_count(1,t),1),'Color', 'k','Marker','p','MarkerFaceColor',MarkerEdgeColors(round(Basin_data(hhd_count(1,t),1))+1,:),'MarkerSize',18,'linewidth',2);
            end
            %plot(0.5/4+ls*0.5,Basin_data(hhd_count(1,t),1),'go','MarkerSize',18,'linewidth',2);
        else
%             fill(x,y,[0.109 0.525 0.933],'FaceAlpha',0.6);
            fill(x,y,MarkerEdgeColors(round(Basin_data(hhd_count(1,t),1))+1,:),'FaceAlpha',0.2);
            if(gait==1)
                plot(ls*0.5,Basin_data(hhd_count(1,t),1),'Color', 'k','Marker','*','MarkerFaceColor',MarkerEdgeColors(round(Basin_data(hhd_count(1,t),1))+1,:),'MarkerSize',18,'linewidth',2);
            elseif(gait==2)
                plot(ls*0.5,Basin_data(hhd_count(1,t),1),'Color', 'k','Marker','o','MarkerFaceColor',MarkerEdgeColors(round(Basin_data(hhd_count(1,t),1))+1,:),'MarkerSize',18,'linewidth',2);
            elseif(gait==3)
                plot(ls*0.5,Basin_data(hhd_count(1,t),1),'Color', 'k','Marker','p','MarkerFaceColor',MarkerEdgeColors(round(Basin_data(hhd_count(1,t),1))+1,:),'MarkerSize',18,'linewidth',2);
            end
            %plot(0.5/4+ls*0.5,Basin_data(hhd_count(1,t),1),'go','MarkerSize',18,'linewidth',2);
        end
    end
    ls=ls+1;
end
set(gca,'Fontsize',20)
xlabel('Obstacle spacing(cm)','fontsize',28);
ylabel('Orientation (^\circ)','fontsize',28);
colormap('jet');
colorbar;
caxis([0,90]);