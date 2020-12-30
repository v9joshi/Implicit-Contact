% attempt at writing a collocation + contact implicit optimization

% Problem
% Launch a ball from a cannon and have it hit a target.
% Minimize the launch energy.
% Ball is allowed to bounce as many times as it wants.
% Collisions are elastic? Is there any way to not have this 
% beyond defining a particular coeff of restitution?

% Parameters for the optimization
params.mass = 1;
params.g = 9.81;

params.numPoints = 20;

totalTime = 2;
params.diffTime = totalTime/params.numPoints;

% initial conditions
params.x0 = 0;
params.xdot0 = 5;

params.y0 = 1;
params.ydot0 = 5;


% set up a bunch of state variables
x_k = zeros(params.numPoints,1);
x_dot_k = zeros(params.numPoints,1);
y_k = zeros(params.numPoints,1);
y_dot_k = zeros(params.numPoints,1);

contactF_y_k = zeros(params.numPoints,1);

Pinput0 = [x_k; x_dot_k; y_k; x_dot_k; contactF_y_k];

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
[cineq_temp,ceq_temp] = consFile_PointMass(Pinput0,Prob);
 
Aineq = []; Bineq_LB = []; Bineq_UB = []; 

c_LB = [-Inf*ones(size(cineq_temp)); 0*ones(size(ceq_temp))];
c_UB = [0*ones(size(cineq_temp)); 0*ones(size(ceq_temp))];
fLowBnd = 0;

Prob = ...
    conAssign(@objFile_PointMass, [], [], [], LB, UB, 'Point Mass Simulation', ...
    Pinput0, [], fLowBnd, ...
    Aineq, Bineq_LB, Bineq_UB, ...
    @conscons_PointMass, [], [], [], c_LB, c_UB, ...
    [], [], [], []);
 
Prob.user.params = params;
 
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
contactF_y_k = presult.x_k(4*numPoints + 1:5*numPoints,1);

% Plot the trajectory
figure(1)
plot(x_k(1:end-1), y_k(1:end-1));
axis 'equal'

xlabel('Horizontal')
ylabel('Vertical')

% Plot the complementarity constraint
figure(2)
plot(contactF_y_k(1:end-1).*y_k(2:end))
axis 'equal'

xlabel('knot point')
ylabel('complimentarity constraint')
