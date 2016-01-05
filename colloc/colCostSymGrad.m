function [cost,costG] = colCostSymGrad(prob,x)

% cost is time - just last element
cost = prob.size.nElems*x(prob.size.nVars);
costG = ([zeros(prob.size.nVars-1,1);prob.size.nElems]);

end