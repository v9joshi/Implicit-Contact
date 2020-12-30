% Determine y_k(end)
gridPointList = [10,20,30,40,50,60,70,80,85,90,95,100];

% Load all the files one at a time and extract some info
for i = 1:length(gridPointList)
    % Make the file name given the number of grid points, then load it.
    numPoints = gridPointList(i);
    datafileName = ['Solution_', num2str(numPoints), 'gridPoints.mat'];
    load(datafileName);
   
    % Determine the final y - coordinate
    y = optimalPinput(3*numPoints);   
    x = optimalPinput(1*numPoints);   
    ydot = optimalPinput(4*numPoints);   
    xdot = optimalPinput(2*numPoints);   
    g = 9.81;
    energy(i) = (ydot^2 + xdot^2 + g*y);
end

%% Plot the data
figure(4)
plot(-log(1./gridPointList(1:end-1)), log(abs(energy(1:end-1) - energy(end))),'-o')

xlabel('log(1/N)');
ylabel('log(energyN(end) - energy100(end))')