%% Initial parameters setup
StepNum = 20; %total number of steps being simulated
HalfBodyLength = 8.0; %cm
HalfBodyWidth = 7.7; %cm
HalfBodyDiag = sqrt(HalfBodyLength^2+HalfBodyWidth^2);
beta_rad = atan(HalfBodyWidth/HalfBodyLength); % robot aspect ratio angle (radians)
X_k_ini = -30; %cm
StepLengthlist =9.0; %cm %S_B = 9.0; S_T = 12.5; S_P = 8.75;

LogNum = 14;
LogDiameter = 4.8; %cm
D = LogDiameter;
LogSpacing = 3.5; %cm
P = LogSpacing;
C = HalfBodyDiag;

resolution_angle = 1;% decide loop size for robot initial angle
resolution_pos = 0.1;% decide loop size for robot initial position

SIZE = round((LogDiameter + LogSpacing)/resolution_pos+1); % define the size of data matrix
Theta_k_minus = zeros(181/resolution_angle+1,SIZE,StepNum); % Matrix for collecting robot orientation before each slides
Theta_k_plus = zeros(181/resolution_angle+1,SIZE,StepNum);  % Matrix for collecting robot orientation after each slides
X_k_minus = zeros(181/resolution_angle+1,SIZE,StepNum);     % Matrix for collecting robot position of CoM before each slides
X_k_plus = zeros(181/resolution_angle+1,SIZE,StepNum);      % Matrix for collecting robot position of CoM after each slides
X_lateral = zeros(181/resolution_angle+1,SIZE,StepNum);     % Matrix for collecting robot lateral position of CoM after each slides
S_L = zeros(181/resolution_angle+1,SIZE,StepNum);           % Matrix for collecting robot left leg slide distance
S_R = zeros(181/resolution_angle+1,SIZE,StepNum);           % Matrix for collecting robot right leg slide distance

State_Space = zeros(360/resolution_angle + 1, (P + D)/resolution_pos + 1, StepNum);% Initialize State Space
color_space = zeros(180/resolution_angle + 1, (P + D)/resolution_pos + 1, 1);% Initialize Color Space
MarkerEdgeColors =jet(91);

figure(1);hold on;
%% Main Loop
Ang = 0;gait = 1; % 1 is bounding, 2 is trotting, and 3 is pacing
for Initial_Angle = -90 : 90/resolution_angle
    Pos = 0;
    Ang = Ang + 1;
    for X_k = 0 : 0.1 : (P + D)
        Pos = Pos + 1;
        X_k_minus(Ang,Pos,1) = X_k;
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

for Ang = 1: 180/resolution_angle + 1
    for Pos = 1 : (P + D)/resolution_pos + 1
        for Step = 1: StepNum
            if(round(Theta_k_plus(Ang,Pos,Step)/pi*180) + 91 > 0)
                State_Space(round(Theta_k_plus(Ang,Pos,Step)/pi*180) + 91 , round(mod(real(X_k_plus(Ang,Pos,Step)*10),(P+D)*10) + 1), Step) = State_Space(round(Theta_k_plus(Ang,Pos,Step)/pi*180) + 91 , round(mod(real(X_k_plus(Ang,Pos,Step)*10),(P+D)*10) + 1), Step) + 1;
            end
        end
    end
end

for Ang = 1: 180/resolution_angle + 1
    for Pos = 1 : (P + D)/resolution_pos + 1
        if(abs(round(Theta_k_plus(Ang,Pos,20)/pi*180)) <= 90)
            color_space(Ang, Pos, 1) = abs(round(Theta_k_plus(Ang,Pos,20)/pi*180));
        else
            color_space(Ang, Pos, 1) = 90 ;
        end
    end
end

for Ang = 11 : 170/resolution_angle + 1
    for Pos = 1 : (P + D)/resolution_pos + 1
        plot3(Ang-91, (Pos-1)/10.0/cos((Ang-91)/180*pi),(Pos-1)/10.0, '.','MarkerSize', 20,'color',MarkerEdgeColors(color_space(Ang, Pos, 1)+1,:),'linewidth',8);
    end
end
%%
x = -80:1:80;
y = 0:0.1:8.3;
[xx,yy] = meshgrid(x,y);
zz = yy./cos(x/180*pi);
Cc = 0.5*ones(length(y),length(x),3);

figure(1);hold on;grid on;t=-80:80;

meshc(xx,zz,yy,Cc);

% plot3(t, (D+P+C*cos(t/180*pi+beta_rad))./cos(t/180*pi), D+P+C*cos(t/180*pi+beta_rad), 'r', 'linewidth', 3);
% plot3(t, (D+C*cos(t/180*pi+beta_rad))./cos(t/180*pi), D+C*cos(t/180*pi+beta_rad), 'r', 'linewidth', 3);
% plot3(t, (D/2+C*cos(t/180*pi+beta_rad))./cos(t/180*pi), D/2+C*cos(t/180*pi+beta_rad), 'r--', 'linewidth', 3);
% plot3(t, (C*cos(t/180*pi+beta_rad))./cos(t/180*pi),C*cos(t/180*pi+beta_rad), 'r', 'linewidth', 3);
% plot3(t, (-P+C*cos(t/180*pi+beta_rad))./cos(t/180*pi), -P+C*cos(t/180*pi+beta_rad), 'r', 'linewidth', 3);
% plot3(t, (-D/2-P+C*cos(t/180*pi+beta_rad))./cos(t/180*pi), -D/2-P+C*cos(t/180*pi+beta_rad), 'r--', 'linewidth', 3);
% plot3(t, (-D-P+C*cos(t/180*pi+beta_rad))./cos(t/180*pi), -D-P+C*cos(t/180*pi+beta_rad), 'r', 'linewidth', 3);
% plot3(t, (-D-2*P+C*cos(t/180*pi+beta_rad))./cos(t/180*pi), -D-2*P+C*cos(t/180*pi+beta_rad), 'r', 'linewidth', 3);
% plot3(t, (-3/2*D-2*P+C*cos(t/180*pi+beta_rad))./cos(t/180*pi), -3/2*D-2*P+C*cos(t/180*pi+beta_rad), 'r--', 'linewidth', 3);
% plot3(t, (-2*D-2*P+C*cos(t/180*pi+beta_rad))./cos(t/180*pi), -2*D-2*P+C*cos(t/180*pi+beta_rad), 'r', 'linewidth', 3);
% % plot3(t, 3/2*D+P+C*cos(t/180*pi+beta_rad), 'g--', 'linewidth', 3);
% plot3(t, (D+P+C*cos(t/180*pi-beta_rad))./cos(t/180*pi), D+P+C*cos(t/180*pi-beta_rad), 'g', 'linewidth', 3);
% plot3(t, (D+C*cos(t/180*pi-beta_rad))./cos(t/180*pi), D+C*cos(t/180*pi-beta_rad), 'g', 'linewidth', 3);
% plot3(t, (D/2+C*cos(t/180*pi-beta_rad))./cos(t/180*pi), D/2+C*cos(t/180*pi-beta_rad), 'g--', 'linewidth', 3);
% plot3(t, (0+C*cos(t/180*pi-beta_rad))./cos(t/180*pi), 0+C*cos(t/180*pi-beta_rad), 'g', 'linewidth', 3);
% plot3(t, (-P+C*cos(t/180*pi-beta_rad))./cos(t/180*pi), -P+C*cos(t/180*pi-beta_rad), 'g', 'linewidth', 3);
% plot3(t, (-D/2-P+C*cos(t/180*pi-beta_rad))./cos(t/180*pi), -D/2-P+C*cos(t/180*pi-beta_rad), 'g--', 'linewidth', 3);
% plot3(t, (-D-P+C*cos(t/180*pi-beta_rad))./cos(t/180*pi), -D-P+C*cos(t/180*pi-beta_rad), 'g', 'linewidth', 3);
% plot3(t, (-D-2*P+C*cos(t/180*pi-beta_rad))./cos(t/180*pi), -D-2*P+C*cos(t/180*pi-beta_rad), 'g', 'linewidth', 3);
% plot3(t, (-3/2*D-2*P+C*cos(t/180*pi-beta_rad))./cos(t/180*pi), -3/2*D-2*P+C*cos(t/180*pi-beta_rad), 'g--', 'linewidth', 3);
% plot3(t, (-2*D-2*P+C*cos(t/180*pi-beta_rad))./cos(t/180*pi), -2*D-2*P+C*cos(t/180*pi-beta_rad), 'g', 'linewidth', 3);

plot3(t, (0-C*cos(t/180*pi+beta_rad))./cos(t/180*pi), 0-C*cos(t/180*pi+beta_rad), 'r', 'linewidth', 3);
plot3(t, (D/2-C*cos(t/180*pi+beta_rad))./cos(t/180*pi), D/2-C*cos(t/180*pi+beta_rad), 'r--', 'linewidth', 3);
plot3(t, (D-C*cos(t/180*pi+beta_rad))./cos(t/180*pi), D-C*cos(t/180*pi+beta_rad), 'r', 'linewidth', 3);
plot3(t, (D+P-C*cos(t/180*pi+beta_rad))./cos(t/180*pi), D+P-C*cos(t/180*pi+beta_rad), 'r', 'linewidth', 3);
plot3(t, (3/2*D+P-C*cos(t/180*pi+beta_rad))./cos(t/180*pi), 3/2*D+P-C*cos(t/180*pi+beta_rad), 'r--');
plot3(t, (2*D+P-C*cos(t/180*pi+beta_rad))./cos(t/180*pi), 2*D+P-C*cos(t/180*pi+beta_rad), 'r', 'linewidth', 3);
plot3(t, (2*D+2*P-C*cos(t/180*pi+beta_rad))./cos(t/180*pi), 2*D+2*P-C*cos(t/180*pi+beta_rad), 'r', 'linewidth', 3);
plot3(t, (5/2*D+2*P-C*cos(t/180*pi+beta_rad))./cos(t/180*pi), 5/2*D+2*P-C*cos(t/180*pi+beta_rad), 'r--', 'linewidth', 3);
plot3(t, (3*D+2*P-C*cos(t/180*pi+beta_rad))./cos(t/180*pi), 3*D+2*P-C*cos(t/180*pi+beta_rad), 'r', 'linewidth', 3);
plot3(t, (0-C*cos(t/180*pi-beta_rad))./cos(t/180*pi), 0-C*cos(t/180*pi-beta_rad), 'g', 'linewidth', 3);
plot3(t, (D/2-C*cos(t/180*pi-beta_rad))./cos(t/180*pi), D/2-C*cos(t/180*pi-beta_rad), 'g--', 'linewidth', 3);
plot3(t, (D-C*cos(t/180*pi-beta_rad))./cos(t/180*pi), D-C*cos(t/180*pi-beta_rad), 'g', 'linewidth', 3);
plot3(t, (D+P-C*cos(t/180*pi-beta_rad))./cos(t/180*pi), D+P-C*cos(t/180*pi-beta_rad), 'g', 'linewidth', 3);
plot3(t, (3/2*D+P-C*cos(t/180*pi-beta_rad))./cos(t/180*pi), 3/2*D+P-C*cos(t/180*pi-beta_rad), 'g--', 'linewidth', 3);
plot3(t, (2*D+P-C*cos(t/180*pi-beta_rad))./cos(t/180*pi), 2*D+P-C*cos(t/180*pi-beta_rad), 'g', 'linewidth', 3);
plot3(t, (2*D+2*P-C*cos(t/180*pi-beta_rad))./cos(t/180*pi), 2*D+2*P-C*cos(t/180*pi-beta_rad), 'g', 'linewidth', 3);
plot3(t, (5/2*D+2*P-C*cos(t/180*pi-beta_rad))./cos(t/180*pi), 5/2*D+2*P-C*cos(t/180*pi-beta_rad), 'g--', 'linewidth', 3);
plot3(t, (3*D+2*P-C*cos(t/180*pi-beta_rad))./cos(t/180*pi), 3*D+2*P-C*cos(t/180*pi-beta_rad), 'g', 'linewidth', 3);

x = -80:10:80;
y = 0.15:1.0:8.15;
[xx,yy] = meshgrid(x,y);
zz = yy./cos(xx/180*pi);
quiver3(xx,zeros(length(y),length(x)),yy,zeros(length(y),length(x)),ones(length(y),length(x)),zeros(length(y),length(x)), 1, 'k','linewidth', 2);

xlim([-80 80]);zlim([0 8.3]);ylim([0 50]);

xlabel('Orientation(degrees)');
zlabel('$\overline{CoM}$ (cm)','interpreter','latex');
ylabel('Active steplength');
set(gca,'Fontsize',35);
view(45,45);

%%
Basin_space_up = zeros(160/resolution_angle + 1, (P + D)/resolution_pos + 1, 1);
Basin_space_down = zeros(160/resolution_angle + 1, (P + D)/resolution_pos + 1, 1);
flag1 = 0; flag2 = 0; %Temporary variables
for x = 1:161
    for y = 1:84
%         flag1 = abs(round(Theta_k_plus(x+10,y,20)/pi*180));
        flag1 = color_space(x+10,y,1);
        for t = 1:83
%             if(abs(round(Theta_k_plus(x+10,mod(y+t,84)+1,20)/pi*180)) == flag1)
            if(color_space(x+10, mod(round(y + t*cos((x-81)/180*pi)),84)+1,1) == flag1)
                Basin_space_up(x,y) = t;
            else
                break;
            end
        end
    end
end

for x = 1:161
    for y = 1:84
%         flag2 = abs(round(Theta_k_plus(x+10,y,20)/pi*180));
        flag2 = color_space(x+10,y,1);
        for t = 1:83
%             if(abs(round(Theta_k_plus(x+10,mod(y-t,84)+1,20)/pi*180)) == flag2)
            if(color_space(x+10, mod(round(y - t*cos((x-81)/180*pi)),84)+1,1) == flag2)
                Basin_space_down(x,y) = t;
            else
                break;
            end
        end
    end
end
x=-80:80;y=0:83;
[xx,yy]=meshgrid(x,y);
figure(2); hold on;
surf(xx,yy,Basin_space_up.');
% surf(xx,yy,Basin_space_down.');
xlim([-80 80]);ylim([0 83]);zlim([0 83]);
xlabel('Orientation(degrees)');
ylabel('$\overline{CoM}$ (cm)','interpreter','latex');

figure(3); hold on;
% surf(xx,yy,Basin_space_up.');
surf(xx,yy,Basin_space_down.');
xlim([-80 80]);ylim([0 83]);zlim([0 83]);
xlabel('Orientation(degrees)');
ylabel('$\overline{CoM}$ (cm)','interpreter','latex');

figure(4); hold on;
% surf(xx,yy,Basin_space_up.');
surf(xx,yy,min(Basin_space_down.',Basin_space_up.'));
xlim([-80 80]);ylim([0 83]);zlim([0 83]);
xlabel('Orientation(degrees)');
ylabel('$\overline{CoM}$ (cm)','interpreter','latex');