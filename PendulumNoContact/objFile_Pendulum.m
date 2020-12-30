% obejctive function for point mass with some contact

function costVal = objFile_Pendulum(pinput, prob)

numPoints = prob.user.params.numPoints;

slackString_k = pinput(5*numPoints + 1:6*numPoints,1);

costVal = sum(slackString_k);

end