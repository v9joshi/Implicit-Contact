% obejctive function for point mass with some contact
function [cineq, ceq] = consFile_PointMass(pinput, prob)

% How many knot points are there?
numPoints = prob.user.params.numPoints;

% What is the time step
diffTime = prob.user.params.diffTime;

% What are the physical parameters
mass = prob.user.params.mass;
g = prob.user.params.g;

% What are the initial states?
x0 = prob.user.params.x0;
x_dot_0 = prob.user.params.xdot0;

y0 = prob.user.params.y0;
y_dot_0 = prob.user.params.ydot0;

% Unpack the input variables
x_k = pinput(1:numPoints,1);
x_dot_k = pinput(numPoints + 1:2*numPoints,1);
y_k = pinput(2*numPoints + 1:3*numPoints,1);
y_dot_k = pinput(3*numPoints + 1:4*numPoints,1);

contactF_y_k = pinput(4*numPoints + 1:5*numPoints,1);

% Set up some constraints
% Horizontal dynamics
constraint_X_K = -x_k(2:end) + x_k(1:end-1) + diffTime*0.5*(x_dot_k(2:end) + x_dot_k(1:end-1));
constraint_X_dot_K = -x_dot_k(2:end) + x_dot_k(1:end-1);

% Vertical dynamics
constraint_Y_K = -y_k(2:end) + y_k(1:end-1) + diffTime*0.5*(y_dot_k(2:end) + y_dot_k(1:end-1));
constraint_Y_dot_K = -y_dot_k(2:end) + y_dot_k(1:end-1) + diffTime*(0.5*(contactF_y_k(2:end) + contactF_y_k(1:end-1)) - mass*g)/mass;

% Contact constraints
constraint_NoContactNoForce = contactF_y_k(1:end-1).*y_k(2:end);
constraint_PositivePosition = -y_k;
constraint_PositiveForce = -contactF_y_k;

% Constraint initial state
constraint_InitialState = [x_k(1)- x0; x_dot_k(1)- x_dot_0; y_k(1) - y0; y_dot_k(1) - y_dot_0];

% combine all the equality constraints
ceq = [constraint_InitialState; constraint_X_K; constraint_X_dot_K; constraint_Y_K; constraint_Y_dot_K; constraint_NoContactNoForce];
cineq = [constraint_PositivePosition; constraint_PositiveForce];
end