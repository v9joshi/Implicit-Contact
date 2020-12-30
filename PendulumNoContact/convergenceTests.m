% Determine y_k(end)
gridPointList = [10,20,40,80,160,320];

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

diffEnergy = diff(energy);
Ratios = diffEnergy(2:end)./diffEnergy(1:end-1);


%% Plot the data
figure(4)
plot(Ratios, 'o-')

xlabel('log(1/N)');