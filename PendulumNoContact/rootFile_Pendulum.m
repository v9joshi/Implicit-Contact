% attempt at writing a collocation + contact implicit optimization

clear all; close all; clc;
% Problem
% A bob on pendulum falls and collides with the ground
% Parameters for the optimization
params.mass = 1;
params.g = 9.81;

%% Initialize with small number of grid points
load Solution_320gridPoints.mat

numPoints = 320;

x_k = optimalPinput(1:numPoints,1);
x_dot_k = optimalPinput(numPoints + 1:2*numPoints,1);
y_k = optimalPinput(2*numPoints + 1:3*numPoints,1);
y_dot_k = optimalPinput(3*numPoints + 1:4*numPoints,1);
stringF_k = optimalPinput(5*numPoints + 1:6*numPoints,1);

slackString_k = optimalPinput(7*numPoints + 1:8*numPoints,1);

%% Continue onwards
params.numPoints = 500;

totalTime = 2;
params.diffTime = totalTime/params.numPoints;

% pivot point
params.pivotX = 0.5;
params.pivotY = 1.2;
params.stringLength = 0.9;

% initial conditions
params.x0 = 0;
params.xdot0 = 0;
params.y0 = 0.8;
params.ydot0 = 0;

% set up a bunch of state variables
numPoints0 = params.numPoints;
x_k0 = interp1(linspace(0,2,numPoints), x_k, linspace(0,2,numPoints0))';%zeros(params.numPoints,1);
x_dot_k0 = interp1(linspace(0,2,numPoints), x_dot_k, linspace(0,2,numPoints0))';%zeros(params.numPoints,1);
y_k0 = interp1(linspace(0,2,numPoints), y_k, linspace(0,2,numPoints0))';%zeros(params.numPoints,1);
y_dot_k0 = interp1(linspace(0,2,numPoints), y_dot_k, linspace(0,2,numPoints0))';%zeros(params.numPoints,1);

stringF_k0 = interp1(linspace(0,2,numPoints), stringF_k, linspace(0,2,numPoints0))';%zeros(params.numPoints,1);

% Ability to violate complimentarity constraints
slackString_k0 = interp1(linspace(0,2,numPoints), slackString_k, linspace(0,2,numPoints0))';%zeros(params.numPoints,1);

% Assemble initial guess
Pinput0 = [x_k0; x_dot_k0; y_k0; y_dot_k0; stringF_k0; slackString_k0];

% Assemble limits into arrays
LB = []; 
UB =  [];

% Inequalities and equalities
Aineq = [];
bineq = [];

Aeq = [];
beq = [];

%% setup SNOPT
% Prob.user.params = params;
% [cineq_temp,ceq_temp] = consFile_Pendulum(Pinput0,Prob);
%  
% Aineq = []; Bineq_LB = []; Bineq_UB = []; 
% 
% c_LB = [-Inf*ones(size(cineq_temp)); 0*ones(size(ceq_temp))];
% c_UB = [0*ones(size(cineq_temp)); 0*ones(size(ceq_temp))];
% fLowBnd = 0;
% 
% Prob = ...
%     conAssign(@objFile_Pendulum, [], [], [], LB, UB, 'Point Mass Simulation', ...
%     Pinput0, [], fLowBnd, ...
%     Aineq, Bineq_LB, Bineq_UB, ...
%     @conscons_Pendulum, [], [], [], c_LB, c_UB, ...
%     [], [], [], []);
%  
Prob.user.params = params;
% 
% Prob.SOL.optPar(35) = 1000; % major iterations limit
% Prob.SOL.optPar(36) = 20000; % minor iterations limit (in the QP)
% 
% Prob.SOL.optPar(41) = 1e-12;
% Prob.Solver.Tomlab = 'snopt'; 
%  
% global funevalcounter; 
% funevalcounter = 0; 
% options = [];

% %% Run optimization
% presult = tomRun('snopt', Prob, 1);
% optimalPinput = presult.x_k;
% 
numPoints = params.numPoints;

%% Setup fmincon

objFun = @(pinput) objFile_Pendulum(pinput, Prob);
conFun = @(pinput) consFile_Pendulum(pinput, Prob);

[x_result, fVal] = fmincon(objFun, Pinput0, Aineq, bineq, Aeq, beq, LB, UB, conFun);

%% unpack the states 
% x_k = presult.x_k(1:numPoints,1);
% x_dot_k = presult.x_k(numPoints + 1:2*numPoints,1);
% y_k = presult.x_k(2*numPoints + 1:3*numPoints,1);
% y_dot_k = presult.x_k(3*numPoints + 1:4*numPoints,1);
% stringF_k = presult.x_k(4*numPoints + 1:5*numPoints,1);
% 
% slackString_k = presult.x_k(5*numPoints + 1:6*numPoints,1);

x_k = x_result(1:numPoints,1);
x_dot_k = x_result(numPoints + 1:2*numPoints,1);
y_k = x_result(2*numPoints + 1:3*numPoints,1);
y_dot_k = x_result(3*numPoints + 1:4*numPoints,1);
stringF_k = x_result(4*numPoints + 1:5*numPoints,1);

slackString_k = x_result(5*numPoints + 1:6*numPoints,1);


%% Plot the trajectory
figure(1)
plot([params.pivotX*ones(numPoints-1,1), x_k(1:end-1)]',[params.pivotY*ones(numPoints-1,1), y_k(1:end-1)]');
hold on
plot(x_k(1:end-1),y_k(1:end-1), 'k');
hold off
ylim([0,1.2])
axis 'equal'

xlabel('Horizontal')
ylabel('Vertical')

% Plot the complementarity constraint
xPivot = params.pivotX;
yPivot = params.pivotY;
stringLength = params.stringLength;

pivotToMassDist = sqrt((x_k - xPivot).^2 + (y_k - yPivot).^2);
figure(2)
plot(stringF_k(1:end-1).*(stringLength - pivotToMassDist(2:end)))
hold off

xlabel('knot point')
ylabel('complimentarity constraint')

% Plot string length vs string tension
figure(3)
plot(stringLength - pivotToMassDist, stringF_k, '-')
hold off

xlabel('Position')
ylabel('Force')

legend('String tension')
