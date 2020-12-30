% This function is required by snopt to bundle the equalities and
% inequalities together
function C = conscons_PointMass(pinput,Prob)

[cineq,ceq] = consFile_PointMass(pinput,Prob);
C = [cineq; ceq];
end