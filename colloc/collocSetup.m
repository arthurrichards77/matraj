function [prob]=collocSetup

%% %%%%%%%%%%%%%% set problem data %%%%%%%%%%%%%%%%

% max speed
maxSpeed = 1.0;

% max turn rate
maxTurn = 4.0;

% initial configuration (x,y,theta)
initConfig = [0 0 -1*pi/4];

% final configuration
termConfig = [1 1 0*pi/4];

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
tConstr = linspace(-1,1,7);

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
prob.colloc.tConstr = tConstr;
prob.colloc.tColloc = tColloc;
prob.colloc.evalMatrix = evalMatrix;
prob.colloc.diffEvalMatrix = diffEvalMatrix;
prob.colloc.dDiffEvalMatrix = dDiffEvalMatrix;

prob.size.nElems = nElems;
prob.size.nColloc = nColloc;
prob.size.nOutput = nOutput;
prob.size.nVars = nElems*nColloc*nOutput+1; % extra one for time

%% initial guess

% total time
T0 = (1.2*norm(prob.bcs.initZ-prob.bcs.termZ)/prob.lims.maxSpeed);

% time, per element
t0 = T0/prob.size.nElems;

% times for collocation points
ts = (1+prob.colloc.tColloc)*0.5;
if prob.size.nElems>1,
    for ee=2:prob.size.nElems,
        ts = [ts, ((ee-1)+(1+prob.colloc.tColloc)*0.5)];
    end
    ts = ts/prob.size.nElems
end

deltaZ = prob.bcs.termZ-prob.bcs.initZ;

xs0 = prob.bcs.initZ(1) + deltaZ(1)*ts;
ys0 = prob.bcs.initZ(2) + deltaZ(2)*ts;

zs0 = [reshape([xs0;ys0],prob.size.nVars-1,1)];

%prob.x0 = [zeros(prob.size.nVars-1,1); 1];
prob.x0 = [zs0;t0];

end