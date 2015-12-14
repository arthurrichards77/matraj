function [prob,x] = colloc

prob = collocSetup

options = optimoptions('fmincon','MaxFunEvals',50000,'MaxIter',10000);

%x = fmincon(@(x)colCost(prob,x),prob.x0,[],[],[],[],[],[],@(x)colCons(prob,x),options);

x = fmincon(@(x)colCost2(prob,x),prob.x0,[],[],[],[],[],[],@(x)colCons2(prob,x),options);

% extract x and y outputs at collocation points
xcs = x(1:2:(prob.size.nVars-1)); % last one is time - don't use it
ycs = x(2:2:(prob.size.nVars-1));
% and the time
t = x(prob.size.nVars);

% get values
xvals = kron(eye(prob.size.nElems),prob.colloc.evalMatrix)*xcs;
yvals = kron(eye(prob.size.nElems),prob.colloc.evalMatrix)*ycs;

plot(xvals,yvals,'.b-')
axis equal

end

function [prob]=collocSetup

%% %%%%%%%%%%%%%% set problem data %%%%%%%%%%%%%%%%

% max speed
maxSpeed = 2.0;

% max turn rate
maxTurn = 1;

% initial configuration (x,y,theta)
initConfig = [0 0 0*3*pi/4];

% final configuration
termConfig = [10 10 -1*pi/4];

% convert to z (output) and z-dot form
initZ = initConfig(:,1:2)';
initZdot = 0.9*maxSpeed*[cos(initConfig(:,3)) sin(initConfig(:,3))]';
termZ = termConfig(:,1:2)';
termZdot = 0.9*maxSpeed*[cos(termConfig(:,3)) sin(termConfig(:,3))]';

% number of outputs
nOutput = numel(initZ);

%% %%%%%%%%%%%%%% set-up collocation method %%%%%%%%%%%%%%%%

% number of elements
nElems = 3;

% degree of polynominal = number of colloc points
nColloc = 4;

% collocation points - t is generalized time, [-1,1]
tColloc = cos((0:(nColloc-1))*pi/(nColloc-1));
% make sure they increase from -1 to 1
tColloc = sort(tColloc);

% number of collocation points
nColloc = length(tColloc);

% matrix of powers
powerMatrix = (ones(nColloc,1)*(0:(nColloc-1)));

% matrix of colloc points
collocMatrix = (tColloc'*ones(1,nColloc));

% vandermonde matrix
vanDerMonde = collocMatrix.^powerMatrix;

% invert the thing
invVanDerMonde = inv(vanDerMonde);

% differential v-d-m matrix
diffVander = powerMatrix.*(collocMatrix.^(powerMatrix-1));

% differentiator matrix
diffMatrix = diffVander*invVanDerMonde;

% and for double differentiation
dDiffVander = powerMatrix.*(powerMatrix-1).*(collocMatrix.^(powerMatrix-2));
dDiffMatrix = dDiffVander*invVanDerMonde;

%% %% construct matrix to evaluate values at endpoints etc

% constraint points
tConstr = linspace(-1,1,17);

% number of constraint points
nConstr = length(tConstr);

% matrix of powers
conPowerMatrix = (ones(nConstr,1)*(0:(nColloc-1)));

% matrix of constraint evaluation points
constrMatrix = (tConstr'*ones(1,nColloc));

% vandermonde matrix
conVanDerMonde = constrMatrix.^conPowerMatrix;

% constraint evalution matrix
evalMatrix = conVanDerMonde*invVanDerMonde;

% derivative evaluation matrices
diffEvalMatrix = (conPowerMatrix.*[zeros(nConstr,1) conVanDerMonde(:,1:end-1)])*invVanDerMonde;
dDiffEvalMatrix = (conPowerMatrix.*(conPowerMatrix-1).*[zeros(nConstr,2) conVanDerMonde(:,1:end-2)])*invVanDerMonde;

%% let's bung everything into a structure

prob.lims.maxSpeed = maxSpeed;
prob.lims.maxTurn = maxTurn;

prob.bcs.initZ = initZ;
prob.bcs.initZdot = initZdot;
prob.bcs.termZ = termZ;
prob.bcs.termZdot = termZdot;

prob.colloc.nConstr = nConstr;
prob.colloc.nColloc = nColloc;
prob.colloc.evalMatrix = evalMatrix;
prob.colloc.diffEvalMatrix = diffEvalMatrix;
prob.colloc.dDiffEvalMatrix = dDiffEvalMatrix;

prob.size.nElems = nElems;
prob.size.nColloc = nColloc;
prob.size.nOutput = nOutput;
prob.size.nVars = nElems*nColloc*nOutput+1; % extra one for time

%% initial guess
xs0 = linspace(prob.bcs.initZ(1),prob.bcs.termZ(1),0.5*(prob.size.nVars-1));
ys0 = linspace(prob.bcs.initZ(2),prob.bcs.termZ(2),0.5*(prob.size.nVars-1));
zs0 = [reshape([xs0;ys0],prob.size.nVars-1,1)];
t0 = (1.2*norm(prob.bcs.initZ-prob.bcs.termZ)/prob.lims.maxSpeed)/prob.size.nElems;

%prob.x0 = [zeros(prob.size.nVars-1,1); 1];
prob.x0 = [zs0;t0];

end

%%%% with autodiff

function [cost,grd] = colCost2(prob,x)

xv = DecVar(x);
op = colCost(prob,xv);

cost=value(op);
grd=grad(op);

end

function [C,Ceq,CG,CGeq] = colCons2(prob,x)

xv = DecVar(x);
[Co,Ceqo] = colCons(prob,xv);

C=value(Co);
CG=grad(Co);
Ceq=value(Ceqo);
CGeq=grad(Ceqo);

end

%%%% cost and constraint functions

function cost = colCost(prob,x)

% cost is time - just last element
cost = prob.size.nElems*x(prob.size.nVars);

end

function [C,Ceq] = colCons(prob,x)
%
% C <= 0
% Ceq == 0

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

% speed, squared
spdsqs = xdots.*xdots + ydots.*ydots

% max speed constraints
C_spd = spdsqs - (0.25*prob.lims.maxSpeed*prob.lims.maxSpeed*ones(numel(spdsqs),1)*(t.*t));

% curvature constraints
C_crv = (xdots.*yDdots - ydots.*xDdots).*(xdots.*yDdots - ydots.*xDdots) - prob.lims.maxTurn*prob.lims.maxTurn*(spdsqs.*spdsqs.*spdsqs);

% compile ineqs
C = [C_spd; C_crv];

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