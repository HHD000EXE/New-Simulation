x = 0:5:75; y = 0:5:90; x1 = 0:5:80; y1 = -20:5:90;
t1 = zeros(1,16);t2 = zeros(1,16);t3 = zeros(1,16);t11 = zeros(1,16);t22 = zeros(1,16);t33 = zeros(1,16);
%% Experiment part
h = figure(1); hold on; grid on;
xlim([0 80]); ylim([-10 90]);

% b1 = [0 0 0 0 0 0 0 37 37 37 37 37 37 37 90 90];
% b2 = [0 0 0 37 37 37 37 37 37 37 90 90 90 90 90 90];
% b3 = [0 0 0 0 37 37 37 37 37 37 37 37 37 90 90 90];
% h1=plot(x + 5/4, b1, 'ko','MarkerFaceColor','g','MarkerSize',10,'linewidth',2);
% plot(x + 2*5/4, b2, 'ko','MarkerFaceColor','g','MarkerSize',10,'linewidth',2);
% plot(x + 3*5/4, b3, 'ko','MarkerFaceColor','g','MarkerSize',10,'linewidth',2);

% tt1 = [15 15 15 15 15 15 15 15 15 15 67 67 67 67 67 67];
% tt2 = [15 15 15 15 15 47 47 47 47 47 47 47 47 47 47 90];
% tt3 = [15 15 15 15 15 15 15 15 15 15 67 67 67 67 67 67];
% osi = [6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6];
% h1=errorbar(x + 5/4,tt1,osi,'bo','MarkerSize',10,'MarkerFaceColor','g','linewidth',2);
% errorbar(x + 2*5/4,tt2,osi,'bo','MarkerSize',10,'MarkerFaceColor','g','linewidth',2);
% errorbar(x + 3*5/4,tt3,osi,'bo','MarkerSize',10,'MarkerFaceColor','g','linewidth',2);

tt1 = [0 0 0 0 0 56 0 0 56 56 56 56 56 90 90 90];
tt2 = [0 0 0 0 0 0 0 56 56 56 56 56 56 56 56 90];
tt3 = [0 0 0 0 0 0 56 0 0 56 56 56 56 56 56 90];
osi1 = [7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7];
osi2 = [7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7];
osi3 = [7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7];
h1=errorbar(x + 5/4,tt1,osi1,'bo','MarkerSize',10,'MarkerFaceColor','g','linewidth',2);
errorbar(x + 2*5/4,tt2,osi2,'bo','MarkerSize',10,'MarkerFaceColor','g','linewidth',2);
errorbar(x + 3*5/4,tt3,osi3,'bo','MarkerSize',10,'MarkerFaceColor','g','linewidth',2);

%% Bounding_1
for angle = 0:5:75
figure(1);hold on;
A = xlsread(strcat(strcat('D:\MATLAB\New_Simulation\New_data_final\Basin_Pace_degree_',num2str(angle)),'_position_1_(P+D).csv'));
xx=A(:,[6:6])-70;
zz=A(:,[8:8]);
for i = 1 : length(xx)
        if(xx(i,1)>180)
            cut_e(1,1) = i-300;
            break;
        end
        if(i==length(xx))
            cut_e(1,1) = i-300;
        end
end
cut_s(1,1) = 400;
a=A(cut_s(1, 1):cut_e(1, 1),[4:4]);
t1(1,angle/5+1) = mean_sp(a); t11(1,angle/5+1) = std_sp(a);
end
%% Bounding_2
for angle = 0:5:75
figure(1);hold on;
A = xlsread(strcat(strcat('D:\MATLAB\New_Simulation\New_data_final\Basin_Pace_degree_',num2str(angle)),'_position_4_3_(P+D).csv'));
xx=A(:,[6:6])-70;
zz=A(:,[8:8]);
for i = 1 : length(xx)
        if(xx(i,1)>180)
            cut_e(1,1) = i-300;
            break;
        end
        if(i==length(xx))
            cut_e(1,1) = i-300;
        end
end
cut_s(1,1) = 400;
a=A(cut_s(1, 1):cut_e(1, 1),[4:4]);
t2(1,angle/5+1) = mean_sp(a); t22(1,angle/5+1) = std_sp(a);
end
%% Bounding_3
for angle = 0:5:75
figure(1);hold on;
A = xlsread(strcat(strcat('D:\MATLAB\New_Simulation\New_data_final\Basin_Pace_degree_',num2str(angle)),'_position_5_3_(P+D).csv'));
xx=A(:,[6:6])-70;
zz=A(:,[8:8]);
for i = 1 : length(xx)
        if(xx(i,1)>180)
            cut_e(1,1) = i-300;
            break;
        end
        if(i==length(xx))
            cut_e(1,1) = i-300;
        end
end
cut_s(1,1) = 400;
a=A(cut_s(1, 1):cut_e(1, 1),[4:4]);
t3(1,angle/5+1) = mean_sp(a); t33(1,angle/5+1) = std_sp(a);
end
%%
h2=errorbar(x + 5/4,t1,t11/2,'ko','MarkerSize',10,'MarkerFaceColor','r','linewidth',2); 
errorbar(x + 2*5/4,t2,t22/2,'ko','MarkerSize',10,'MarkerFaceColor','r','linewidth',2); 
errorbar(x + 3*5/4,t3,t33/2,'ko','MarkerSize',10,'MarkerFaceColor','r','linewidth',2); 

% plot([0,80],[-15,-15],'g--','linewidth',2); %Only for trotting gait

ll=legend([h1, h2],'Model prediction','Experiment results','Location','NorthEastOutside'); set(ll,'fontsize',16);
xlabel("Initial orientation(°)",'fontsize',24); ylabel("Steady orientation(°)",'fontsize',24);xticks(x);yticks(y);
set(gca,'fontsize',24);

