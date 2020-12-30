% This function is required by snopt to bundle the equalities and
% inequalities together
function C = conscons_Pendulum(pinput,Prob)

[cineq,ceq] = consFile_Pendulum(pinput,Prob);
C = [cineq; ceq];
end