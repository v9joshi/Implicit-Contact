% attempt at writing a collocation + contact implicit optimization

clear all; close all; clc;
% Problem
% A 3D biped walks from some initial state to some final state
% while minimizing sum of forces squared
% Parameters for the optimization
params.mass = 1;
params.g = 9.81;

params.numPoints = 100;

totalTime = 2;
params.diffTime = totalTime/params.numPoints;

params.LegLength = 1;

% Initial foot position
params.foot0X = 0;
params.foot0Y = 0;
params.foot0Z = 0;

% initial state
params.x0 = 0;
params.xdot0 = 0;
params.y0 = params.LegLength;
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
Bineq = [];

Aeq = [];
Beq = [];

%% setup SNOPT
Prob.user.params = params;
[cineq_temp,ceq_temp] = consFile_Pendulum(Pinput0,Prob);
 
Aineq = []; Bineq_LB = []; Bineq_UB = []; 

c_LB = [-Inf*ones(size(cineq_temp)); 0*ones(size(ceq_temp))];
c_UB = [0*ones(size(cineq_temp)); 0*ones(size(ceq_temp))];
fLowBnd = 0;

Prob = ...
    conAssign(@objFile_Pendulum, [], [], [], LB, UB, 'Point Mass Simulation', ...
    Pinput0, [], fLowBnd, ...
    Aineq, Bineq_LB, Bineq_UB, ...
    @conscons_Pendulum, [], [], [], c_LB, c_UB, ...
    [], [], [], []);
 
Prob.user.params = params;

Prob.SOL.optPar(35) = 1000; % major iterations limit
Prob.SOL.optPar(36) = 20000; % minor iterations limit (in the QP)

Prob.SOL.optPar(41) = 1e-12;
Prob.Solver.Tomlab = 'snopt'; 
 
global funevalcounter; 
funevalcounter = 0; 
options = [];

%% Run optimization
presult = tomRun('snopt', Prob, 1);
optimalPinput = presult.x_k;

numPoints = params.numPoints;

% unpack the states 
x_k = presult.x_k(1:numPoints,1);
x_dot_k = presult.x_k(numPoints + 1:2*numPoints,1);
y_k = presult.x_k(2*numPoints + 1:3*numPoints,1);
y_dot_k = presult.x_k(3*numPoints + 1:4*numPoints,1);
stringF_k = presult.x_k(4*numPoints + 1:5*numPoints,1);

slackString_k = presult.x_k(5*numPoints + 1:6*numPoints,1);

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

legend('String tension')

% Plot string length vs string tension
figure(3)
plot(stringLength - pivotToMassDist, stringF_k, 'o')
hold off

xlabel('Position')
ylabel('Force')

legend('String tension')
