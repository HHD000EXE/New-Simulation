z = zeros(100,100);x = linspace(1,1,100);y = linspace(1,1,100);
for j = 2:100
    for i = 2:100
        if (rand(1) < 0.5)
            z(j,i) = (z(j,i-1) + z(j-1,i) + z(j-1,i-1))./3 + 0.1;
        else
            z(j,i) = (z(j,i-1) + z(j-1,i) + z(j-1,i-1))./3 - 0.1;
        end
    end
end
[xx,yy]=meshgrid(x,y);
mesh(xx,yy,z);

[X,Y,Z]=peaks(100); grid off;
Z = Z.*z;grid off;
mesh(X,Y,Z);grid off;
zlim([-5 10]);
xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis');