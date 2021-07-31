stop=20;t=1:stop;D=4.8;P=3.5;LogNum=14;
s1=zeros(1,stop);
s2=zeros(1,stop);
% s3=zeros(1,stop);
% s4=zeros(1,stop);
MarkerEdgeColors =jet(91);
for x=1:5:71
    for y=1%:5:66
        figure(3);hold on;
        s1(1,:)=X_lateral(x,y,t);
        s2(1,:)=X_k_plus(x,y,t);
%         s3(1,:)=Theta_k_plus(x,y,t);
%         s4(1,:)=x;
        plot(s1,s2,'Color', MarkerEdgeColors(x,:),'linewidth',3);
%         plot3(s1,s2,s3,'color',[1 0 0],'linewidth',1);
%         plot3(s1,s2,s4,'color',[0.6 0 0],'linewidth',1);
%         plot3(s1,s2,s3,'color',[1.0-x/90.0 0+(x-30)^2/3600.0 0+x/90.0],'linewidth',1);
%         plot3(s1,s2,s4,'color',[0.5 0.5 0.5],'linewidth',1);
%         patch(s1,s2,s3,s4,'edgecolor','flat','facecolor','none');
%         patch(s1,s2,s4,s1,'edgecolor','flat','facecolor','none');
    end
end

[edge,slip]=ObstacleShow(LogNum,LogDiameter,LogSpacing); % initial obstacle field
for ii=0:LogNum
    figure(3);hold on;
    xlim([-100 300]);
    ylim([-30 150]);
    zlim([-40 100]);
    xL = get(gca,'XLim');
    plot([-300,1500],[ii*(P+D)+0,ii*(P+D)+0],'r','linewidth',0.5);
    plot([-300,1500],[ii*(P+D)+D/2,ii*(P+D)+D/2],'k','linewidth',0.5);
    plot([-300,1500],[ii*(P+D)+D,ii*(P+D)+D],'b','linewidth',0.5);
    x=[-300 1500 1500 -300];y=[ii*(P+D)+0 ii*(P+D)+0 ii*(P+D)+D/2 ii*(P+D)+D/2];fill(x,y,'r','FaceAlpha',0.1,'Edgecolor','none');
    x=[-300 1500 1500 -300];y=[ii*(P+D)+D/2 ii*(P+D)+D/2 ii*(P+D)+D ii*(P+D)+D];fill(x,y,'b','FaceAlpha',0.1,'Edgecolor','none');
%     x=[-300 1500 1500 -300];y=[ii*(P+D)+0 ii*(P+D)+0 ii*(P+D)+D/2 ii*(P+D)+D/2]; z=[-40 -40 -40 -40]; fill3(x,y,z,'r','FaceAlpha',0.1);
%     x=[-300 1500 1500 -300];y=[ii*(P+D)+D/2 ii*(P+D)+D/2 ii*(P+D)+D ii*(P+D)+D];z=[-40 -40 -40 -40]; fill3(x,y,z,'b','FaceAlpha',0.1);
end

% grid on;set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
xlabel('Horizontal position(cm)','fontsize',26);ylabel('Fore-aft position(cm)','fontsize',26);%zlabel('Robot angle(бу)','fontsize',18);
colormap('jet');
caxis([0,90]);
colorbar;
set(gca,'Fontsize',20)
% view(3);
