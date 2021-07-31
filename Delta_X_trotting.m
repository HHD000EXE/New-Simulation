x1=0:90;
C=8.6;
D=4.5;
P=4.5;
Theta=32;
delta=atan(0.5/0.7)/pi*180;
y1=2*C*cos((x1+delta)/180*pi);
y2=2*C*cos((x1-delta)/180*pi);
y3=28/2.54*cos(x1/180*pi);
figure(1);hold on;
axis([0 90 -10 20]);
plot(x1,y1,'g');
plot(x1,y2,'r');
plot(x1,y3,'m','linewidth',1.5);
plot([Theta,Theta],[-10,20],'c','linewidth',2);

plot([0,90],[0,0],'r--','linewidth',1);
plot([0,90],[D/2,D/2],'k','linewidth',2);
plot([0,90],[D,D],'b--','linewidth',1);
x=[0 90 90 0];y=[0 0 D/2 D/2];fill(x,y,'r','FaceAlpha',0.3);
x=[0 90 90 0];y=[D/2 D/2 D D];fill(x,y,'b','FaceAlpha',0.3);

plot([0,90],[D+P+0,D+P+0],'r--','linewidth',1);
plot([0,90],[D+P+D/2,D+P+D/2],'k','linewidth',2);
plot([0,90],[D+P+D,D+P+D],'b--','linewidth',1);
x=[0 90 90 0];y=[D+P+0 D+P+0 D+P+D/2 D+P+D/2];fill(x,y,'r','FaceAlpha',0.3);
x=[0 90 90 0];y=[D+P+D/2 D+P+D/2 D+P+D D+P+D];fill(x,y,'b','FaceAlpha',0.3);

plot([0,90],[2*(D+P)+0,2*(D+P)+0],'r--','linewidth',1);
plot([0,90],[2*(D+P)+D/2,2*(D+P)+D/2],'k','linewidth',2);
plot([0,90],[2*(D+P)+D,2*(D+P)+D],'b--','linewidth',1);
x=[0 90 90 0];y=[2*(D+P)+0 2*(D+P)+0 2*(D+P)+D/2 2*(D+P)+D/2];fill(x,y,'r','FaceAlpha',0.3);
x=[0 90 90 0];y=[2*(D+P)+D/2 2*(D+P)+D/2 2*(D+P)+D 2*(D+P)+D];fill(x,y,'b','FaceAlpha',0.3);

plot([0,90],[-D/2,-D/2],'k','linewidth',2);
plot([0,90],[-D,-D],'b--','linewidth',1);
plot([0,90],[-(D+P+0),-(D+P+0)],'r--','linewidth',1);
plot([0,90],[-(D+P+0)-D/2,-(D+P+0)-D/2],'k','linewidth',2);
% x=[0 90 90 0];y=[0 0 -D/2 -D/2];fill(x,y,'r','FaceAlpha',0.3);
% x=[0 90 90 0];y=[-D/2 -D/2 -D -D];fill(x,y,'b','FaceAlpha',0.3);

xlabel('Robot angle(бу)','fontsize',18);
ylabel('\DeltaX(inche)','fontsize',18);
legend('\DeltaX_{2}','\DeltaX_{1}','S*cos(\theta)','Steady Angle');

figure(2);hold on;
axis([0 90 0 P+D]);
y1=mod(abs(2*C*cos((x1+delta)/180*pi)),P+D);
y2=mod(2*C*cos((x1-delta)/180*pi),P+D);
y3=mod(28/2.54*cos(x1/180*pi),P+D);
plot([0,90],[0,0],'r--','linewidth',1);
plot([0,90],[D/2,D/2],'k','linewidth',2);
plot([0,90],[D,D],'b--','linewidth',1);
plot([0,90],[D+P+0,D+P+0],'r--','linewidth',1);
x=[0 90 90 0];y=[0 0 D/2 D/2];fill(x,y,'r','FaceAlpha',0.1);
x=[0 90 90 0];y=[D/2 D/2 D D];fill(x,y,'b','FaceAlpha',0.1);
plot(x1,y1,'g');
plot(x1,y2,'r');
plot(x1,y3,'m','linewidth',1.5);
plot([Theta,Theta],[-10,20],'c','linewidth',2);
legend('\DeltaX_{2}|_{P+D}','\DeltaX_{1}|_{P+D}','S*cos(\theta)|_{P+D}','Steady Angle');