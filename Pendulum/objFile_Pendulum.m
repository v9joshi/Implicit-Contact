% objective function for point mass with some contact
function costVal = objFile_Pendulum(pinput, prob)
    % Number of collocation points
    numPoints = prob.user.params.numPoints;

    % The slack variables in the system
    slackContact_k = pinput(6*numPoints + 1:7*numPoints,1);
    slackString_k = pinput(7*numPoints + 1:8*numPoints,1);

    % Minimize the slack variables for the contact contraint and the 
    % pendulum string constraint
    costVal = (sum(slackContact_k) + sum(slackString_k));
end