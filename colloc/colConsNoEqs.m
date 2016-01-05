function [C,Ceq] = colConsNoEqs(prob,x)
%
% C <= 0
% Ceq == 0

% extract x and y outputs at collocation points
%xcs = x(1:2:(prob.size.nVars-1)); % last one is time - don't use it
%ycs = x(2:2:(prob.size.nVars-1));
% and the time, PER ELEMENT, all assumed equal
t = x(prob.size.nVars);

% get values
%xvals = prob.mats.bigEval*xcs;
%yvals = prob.mats.bigEval*ycs;

% get derivatives
xdots = prob.mats.bigDiffX*x; % trouble here with "/t"
ydots = prob.mats.bigDiffY*x;

% and second derivatives
xDdots = prob.mats.bigDblDiffX*x; % trouble here with "/t"
yDdots = prob.mats.bigDblDiffY*x;

% speed, squared, w.r.t normalized time
spdsqs = xdots.*xdots + ydots.*ydots;

% max speed constraints
C_spd = spdsqs - ((0.25*prob.lims.maxSpeed*prob.lims.maxSpeed*ones(numel(spdsqs),1))*(t.*t));

% curvature constraints
% NOTE - this experession is VERY sensitive to numerical issues.  Don't try
% and re-arrange to get rid of the divide.
crv1 = (xdots.*yDdots - ydots.*xDdots);
crv2 = crv1.*crv1; % this one makes little difference by power or by multiply
%crv2 = crv1.^2;
%crv3 = spdsqs.*spdsqs.*spdsqs;
crv3 = spdsqs.^3; % much faster done as a power
C_crv = (crv2./crv3) - (prob.lims.maxTurn*prob.lims.maxTurn);

% compile ineqs
C = [C_spd; C_crv];
%C = C_spd;

Ceq = [];

end