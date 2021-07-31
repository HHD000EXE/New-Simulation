function Angel3D(steady_state,resolution_angle,resolution_pos,f)
% Relationship among InputAngel and InputPosition and OutputAngel
figure(f)
y=0:resolution_angle:90;
x=0.9:resolution_pos/100*2.54*10:1.8;
[xx,yy]=meshgrid(x,y);
surf(xx,yy,steady_state)
xlabel('InitialPosition');ylabel('InitialAngle');zlabel('Output');
h=colorbar;
set(get(h,'label'),'string','Output');
end