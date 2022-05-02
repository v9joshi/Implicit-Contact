% Animate solution
figure(101)
lineHandle = plot([params.pivotX, x_k(1)],[params.pivotY, y_k(1)]);
hold on
dotHandle = plot(x_k(1),y_k(1), 'ko');
plot([-0.2,1.2],[0,0],'r-')% ground
hold off
ylim([0,1.2])
axis 'equal'
set(gcf,'color','w')

for currTime = 1:length(x_k)
    lineHandle.XData = [params.pivotX, x_k(currTime)];
    lineHandle.YData = [params.pivotY, y_k(currTime)];
    hold on
    dotHandle.XData = x_k(currTime);
    dotHandle.YData = y_k(currTime);
    hold off   
    ylim([-1.2,1.2])
    xlim([-1.2,1.2])
    axis 'square'
    drawnow
    pause(0.01)
end