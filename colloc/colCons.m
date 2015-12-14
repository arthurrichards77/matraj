function [C,Ceq] = colCons(prob,x)
%
% C <= 0
% Ceq == 0

% extract x and y outputs at collocation points
xcs = x(1:2:(prob.size.nVars-1)); % last one is time - don't use it
ycs = x(2:2:(prob.size.nVars-1));
% and the time, PER ELEMENT, all assumed equal
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

% speed, squared, w.r.t normalized time
spdsqs = xdots.*xdots + ydots.*ydots;

% max speed constraints
C_spd = spdsqs - (0.25*prob.lims.maxSpeed*prob.lims.maxSpeed*ones(numel(spdsqs),1)*(t.*t));

% curvature constraints
% NOTE - this experession is VERY sensitive to numerical issues.  Don't try
% and re-arrange to get rid of the divide.
C_crv = ((xdots.*yDdots - ydots.*xDdots).*(xdots.*yDdots - ydots.*xDdots)./(spdsqs.*spdsqs.*spdsqs)) - (prob.lims.maxTurn*prob.lims.maxTurn);

% compile ineqs
C = [C_spd; C_crv];
%C = C_spd;

% initial constraints
Ceq_init = [xvals(1);yvals(1);xdots(1);ydots(1)]-[prob.bcs.initZ; prob.bcs.initZdot*t/2];

% terminal constraints
Ceq_term = [xvals(end);yvals(end);xdots(end);ydots(end)]-[prob.bcs.termZ;prob.bcs.termZdot*t/2];
%Ceq_term = [xvals(end);yvals(end)]-[prob.bcs.termZ];

% compile
Ceq = [Ceq_init; Ceq_term];

% add continuity constraints if needed
if prob.size.nElems>1,
    for ee=1:(prob.size.nElems-1),
        Ceq_cts = [xvals(ee*prob.colloc.nConstr) - xvals(ee*prob.colloc.nConstr+1);
                   yvals(ee*prob.colloc.nConstr) - yvals(ee*prob.colloc.nConstr+1);
                   xdots(ee*prob.colloc.nConstr) - xdots(ee*prob.colloc.nConstr+1);
                   ydots(ee*prob.colloc.nConstr) - ydots(ee*prob.colloc.nConstr+1)];
        Ceq = [Ceq; Ceq_cts];
    end
end

end