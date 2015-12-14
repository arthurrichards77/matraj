function cost = colCost(prob,x)

% cost is time - just last element
cost = prob.size.nElems*x(prob.size.nVars);

end