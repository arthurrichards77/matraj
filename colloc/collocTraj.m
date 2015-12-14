function [xvals,yvals,xdots,ydots,xDdots,yDdots] = collocTraj(prob,x)

% extract x and y outputs at collocation points
xcs = x(1:2:(prob.size.nVars-1)); % last one is time - don't use it
ycs = x(2:2:(prob.size.nVars-1));
% and the time
t = x(prob.size.nVars);

% get values
xvals = kron(eye(prob.size.nElems),prob.colloc.evalMatrix)*xcs;
yvals = kron(eye(prob.size.nElems),prob.colloc.evalMatrix)*ycs;

% get derivatives
xdots = kron(eye(prob.size.nElems),prob.colloc.diffEvalMatrix)*xcs; % trouble here with "/t"
ydots = kron(eye(prob.size.nElems),prob.colloc.diffEvalMatrix)*ycs;

% and second derivatives
xDdots = kron(eye(prob.size.nElems),prob.colloc.dDiffEvalMatrix)*xcs; % trouble here with "/t"
yDdots = kron(eye(prob.size.nElems),prob.colloc.dDiffEvalMatrix)*ycs;

end

