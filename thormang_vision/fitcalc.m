X = fit(cases(:,3:4), cases(:,1), 'poly33')
Y = fit(cases(:,3:4), cases(:,2), 'poly33')
%%
xfit = X(cases(:,3),cases(:,4));
yfit = Y(cases(:,3),cases(:,4));
%%
subplot(2,2,1)
for i=1:3
    plot(cases(5*i-4:5*i,3),cases(5*i-4:5*i,2),"LineWidth",4)
    hold on
end
legend("x=400mm","x=500mm","x=600mm")
xlabel("u (pixels)")
ylabel("y (mm)")
subplot(2,2,2)
for i=1:3
    plot(cases(5*i-4:5*i,4),cases(5*i-4:5*i,2),"LineWidth",4)
    hold on
end
legend("x=400mm","x=500mm","x=600mm")
xlabel("v (pixels)")
ylabel("y (mm)")
subplot(2,2,3)
for i=1:5
    plot(cases([i,i+5,i+10],3),cases([i,i+5,i+10],1),"LineWidth",4)
    hold on
end
legend("y=-150mm","y=-100mm","y=0mm","y=100mm","y=150mm")
xlabel("u (pixels)")
ylabel("x (mm)")
subplot(2,2,4)
for i=1:5
    plot(cases([i,i+5,i+10],4),cases([i,i+5,i+10],1),"LineWidth",4)
    hold on
end
legend("y=-150mm","y=-100mm","y=0mm","y=100mm","y=150mm")
xlabel("v (pixels)")
ylabel("x (mm)")

sgtitle("Relationship between Pixel and World Coordinates")
%%
scatter(yfit,xfit,"r")
hold on
scatter(cases(:,2),cases(:,1),"b")
axis equal
%%
ex = (cases(:,1)-xfit);
ey = (cases(:,2)-yfit);

ex_average = mean(abs(ex));
ey_average = mean(abs(ey));

ex_rms = rms(ex);
ey_rms = rms(ey);

[ex_average,ex_rms,ey_average,ey_rms]