% obejctive function for point mass with some contact

function costVal = objFile_Pendulum(pinput, prob)

numPoints = prob.user.params.numPoints;

slackContact_k = pinput(6*numPoints + 1:7*numPoints,1);
slackString_k = pinput(7*numPoints + 1:8*numPoints,1);

costVal = (sum(slackContact_k) + sum(slackString_k));

end