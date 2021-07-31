figure(1);grid off;hold on;
xlim([0 4*pi]);ylim([0 10]);

set(gca,'XTick',[0:0.5*pi:4*pi]);
set(gca,'xtickLabel',{'0','0.5\pi','\pi','1.5\pi','2\pi','2.5\pi','3\pi','3.5\pi','4\pi'});
set(gca,'YTick',[2:2:8]);
set(gca,'YtickLabel',{'RF','LF','LB','RB'});
xlabel('Relative phase(rad)');
set(gca,'fontsize',24);