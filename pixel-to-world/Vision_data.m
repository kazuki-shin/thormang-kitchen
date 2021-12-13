clc; clear;

load vision_data.dat;

xr = vision_data(1,:);
yr = vision_data(2,:);
xe = vision_data(3,:);
ye = vision_data(4,:);

err = sqrt((xr-xe).^2+(yr-ye).^2);
std(err);

[xq,yq] = meshgrid(0.30:.01:0.70, -0.20:.01:0.20);
vq = griddata(xr,yr,err,xq,yq);

mesh(xq,yq,vq)
hold on
plot3(xr,yr,err,'o')
xlim([0.30 0.70])
ylim([-0.20 0.20])
xlabel('x (m)');
ylabel('y (m)');
zlabel('error (m)');










