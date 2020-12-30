% obejctive function for point mass with some contact
function [cineq, ceq] = consFile_Pendulum(pinput, prob)

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

% What is the pivot location
xPivot = prob.user.params.pivotX;
yPivot = prob.user.params.pivotY;

stringLength = prob.user.params.stringLength;

% Unpack the input variables
x_k = pinput(1:numPoints,1);
x_dot_k = pinput(numPoints + 1:2*numPoints,1);
y_k = pinput(2*numPoints + 1:3*numPoints,1);
y_dot_k = pinput(3*numPoints + 1:4*numPoints,1);

stringF_k = pinput(4*numPoints + 1:5*numPoints,1);

slackString_k = pinput(5*numPoints + 1:6*numPoints,1);

% determine the distance from pivot to mass
pivotToMassDist = sqrt((x_k - xPivot).^2 + (y_k - yPivot).^2);

% Set up some constraints
% Horizontal dynamics should be followed
horizontalStringForce = stringF_k.*(xPivot - x_k)/stringLength;
totalHorizontalForce = horizontalStringForce;

constraint_X_K = -x_k(2:end) + x_k(1:end-1) + diffTime*0.5*(x_dot_k(2:end) + x_dot_k(1:end-1));
constraint_X_dot_K = -x_dot_k(2:end) + x_dot_k(1:end-1)  + diffTime*(0.5*(totalHorizontalForce(2:end) + totalHorizontalForce(1:end-1))/mass);

% Vertical dynamics should be followed
verticalStringForce = stringF_k.*(yPivot - y_k)/stringLength;
totalVerticalForce = verticalStringForce - mass*g;

constraint_Y_K = -y_k(2:end) + y_k(1:end-1) + diffTime*0.5*(y_dot_k(2:end) + y_dot_k(1:end-1));
constraint_Y_dot_K = -y_dot_k(2:end) + y_dot_k(1:end-1) + diffTime*(0.5*(totalVerticalForce(2:end) + totalVerticalForce(1:end-1))/mass);

% string constraints
constraint_NotTautNoForce = stringF_k(1:end-1).*(stringLength - pivotToMassDist(2:end)) - slackString_k(1:end-1); % tension only when string is taut
constraint_StringInTension = -stringF_k; % String can't be in compression
constraint_StringLength = -(stringLength - pivotToMassDist); % Pivot to mass distance can't be more than string length
constraint_TensionSlackPositive = -slackString_k; % Slack variables should be positive

% Constraint initial state
constraint_InitialState = [x_k(1)- x0; x_dot_k(1)- x_dot_0; y_k(1) - y0; y_dot_k(1) - y_dot_0]; % Satisfy the initial condition

% combine all the equality constraints
ceq = [constraint_InitialState; constraint_X_K; constraint_X_dot_K; constraint_Y_K; constraint_Y_dot_K];

% combine all the inequality constraints
cineq = [constraint_StringInTension; constraint_StringLength;
         constraint_TensionSlackPositive;
         constraint_NotTautNoForce];
end