function PlotPose_frame_pacing(figind, frame, xCoM, zCoM,theta, color, HalfBodyDiag, beta, LegPair)
% visualizes the robot pose
%for debugging:
% frame=kk;
% xCoM = xCoM_pre(kk);
% zCoM = zCoM_pre(kk);
% xleg = xleg_pre(kk,:); 
% zleg = zleg_pre(kk,:);
% IndL = ActIndLeft(kk);
% IndR = ActIndRight(kk);
    xleg = [0,0,0,0];
    zleg = [0,0,0,0];
    
    zleg(1) = zCoM + HalfBodyDiag*cos(beta-theta);
    zleg(2) = zCoM + HalfBodyDiag*cos(beta+theta);
    zleg(3) = zCoM - HalfBodyDiag*cos(beta-theta);
    zleg(4) = zCoM - HalfBodyDiag*cos(beta+theta);

    xleg(1) = xCoM - HalfBodyDiag*cos(pi/2+theta-beta);
    xleg(2) = xCoM + HalfBodyDiag*cos(pi/2-theta-beta);
    xleg(3) = xCoM + HalfBodyDiag*cos(pi/2+theta-beta);
    xleg(4) = xCoM - HalfBodyDiag*cos(pi/2-theta-beta);


    xv=[xleg(4) xleg(3) xleg(2) xleg(1) xleg(4)];
    zv=[zleg(4) zleg(3) zleg(2) zleg(1) zleg(4)];

%     figure(figind);%hold all;
    if color==1
    plot(xv+frame,zv,'k-');hold on;
    else
    plot(xv+frame,zv,'g-');hold on;
    end
    
    %highlight activated leg
    if LegPair==1 %LF, RF
%         plot(xleg(IndL)+frame,zleg(IndL),'r*','markersize',8,'linewidth',1.5);hold on;
%         plot(xleg(IndR)+frame,zleg(IndR),'c*','markersize',8,'linewidth',1.5);hold on;
        plot(xleg(1)+frame,zleg(1),'r*','markersize',8,'linewidth',1.5);hold on;
        plot(xleg(4)+frame,zleg(4),'c*','markersize',8,'linewidth',1.5);hold on;
    else %LB, RB
%         plot(xleg(IndL)+frame,zleg(IndL),'mo','markersize',8,'linewidth',1.5);hold on;
%         plot(xleg(IndR)+frame,zleg(IndR),'bo','markersize',8,'linewidth',1.5);hold on;
        plot(xleg(2)+frame,zleg(2),'mo','markersize',8,'linewidth',1.5);hold on;
        plot(xleg(3)+frame,zleg(3),'bo','markersize',8,'linewidth',1.5);hold on;
    end
    
    %highlight CoM
%     plot(xCoM+frame, zCoM, 'k.','markersize',12,'linewidth',3);hold on;
    plot(xCoM+frame, zCoM, 'k.','markersize',12,'linewidth',3);hold on;
    xlabel('Lateral position(Cm)','fontsize',18);
    ylabel('Fore-aft position(Cm)','fontsize',18);
    axis image;
%     xlim([-2 frame+2]);
%     ylim([-2 2]);
end