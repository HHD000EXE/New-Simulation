figure(1);grid on; hold on;

plot3(pi,pi,0,'marker','o','color','y','markersize',18);
plot3(pi,pi*3/4,pi*1/4,'marker','o','color','k','markersize',18);
plot3(pi,pi*2/4,pi*2/4,'marker','o','color','k','markersize',18);
plot3(pi,pi*1/4,pi*3/4,'marker','o','color','k','markersize',18);

plot3(pi,0,pi,'marker','o','color','c','markersize',18);
plot3(pi*3/4,pi*1/4,pi,'marker','o','color','k','markersize',18);
plot3(pi*2/4,pi*2/4,pi,'marker','o','color','k','markersize',18);
plot3(pi*1/4,pi*3/4,pi,'marker','o','color','k','markersize',18);

plot3(0,pi,pi,'marker','o','color','b','markersize',18);
plot3(pi*1/4,pi,pi*3/4,'marker','o','color','k','markersize',18);
plot3(pi*2/4,pi,pi*2/4,'marker','o','color','k','markersize',18);
plot3(pi*3/4,pi,pi*1/4,'marker','o','color','k','markersize',18);

plot3([pi,pi],[pi,0],[0,pi],'k','linewidth',1.5);
plot3([pi,0],[0,pi],[pi,pi],'k','linewidth',1.5);
plot3([0,pi],[pi,pi],[pi,0],'k','linewidth',1.5);

set(gca,'XTick',[0:0.5*pi:2*pi]);
set(gca,'xtickLabel',{'0','0.5\pi','\pi','1.5\pi','2\pi'});
set(gca,'YTick',[0:0.5*pi:2*pi]);
set(gca,'YtickLabel',{'0','0.5\pi','\pi','1.5\pi','2\pi'});
set(gca,'ZTick',[0:0.5*pi:2*pi]);
set(gca,'ZtickLabel',{'0','0.5\pi','\pi','1.5\pi','2\pi'});
h1=xlabel('$\phi_1$(rad)','fontsize',26);set(h1,'Interpreter','latex');
h2=ylabel('$\phi_2$(rad)','fontsize',26);set(h2,'Interpreter','latex');
h3=zlabel('$\phi_3$(rad)','fontsize',26);set(h3,'Interpreter','latex');
xlim([0,2*pi]);ylim([0,2*pi]);zlim([0,2*pi]);
set(gca,'fontsize',24);
view(45,45);