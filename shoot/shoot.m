function [res, xres] = shoot
close all

prob = shootSetup();

options = optimset('fmincon');
options = optimset(options,'MaxFunEvals',10000);

% run the optimizer
tic
res = fmincon(@(x)shoot1Cost(prob,x),prob.x0,[],[],[],[],prob.LB,prob.UB,@(x)shoot1Cons(prob,x),options);
tShoot = toc

xres = shoot1(prob,res,10);
xres0 = shoot1(prob,prob.x0,10);

% plot the path
figure
plot(xres(1,:),xres(2,:),'.b-',...
     prob.bcs.xInit(1),prob.bcs.xInit(2),'ms',...
     prob.bcs.xTerm(1),prob.bcs.xTerm(2),'mx')
% plot(xres(1,:),xres(2,:),'.b-',...
%      xres0(1,:),xres0(2,:),'.g-',...
%      prob.bcs.xInit(1),prob.bcs.xInit(2),'ms',...
%      prob.bcs.xTerm(1),prob.bcs.xTerm(2),'mx')
axis equal

% plot time histories of control
figure
subplot 211
stairs(linspace(0,res(1),prob.steps.nSteps),res(2:2:end),'LineWidth',2)
axis([0 res(1) 0 prob.lims.maxSpeed*1.2])
hold on
plot([0 res(1)],[1 1]*prob.lims.maxSpeed,'r--')
ylabel('Speed')
subplot 212
stairs(linspace(0,res(1),prob.steps.nSteps),res(3:2:end),'LineWidth',2)
axis([0 res(1) prob.lims.maxTurn*[-1.2 1.2]])
hold on
plot([0 res(1)],[1 1]*prob.lims.maxTurn,'r--')
plot([0 res(1)],-[1 1]*prob.lims.maxTurn,'r--')
ylabel('Turn')
xlabel('Time')

end

function prob = shootSetup

prob.steps.nSteps = 8;

prob.lims.maxTurn = 3.0;
prob.lims.maxSpeed = 1.0;

prob.bcs.xInit = [0;0;0*pi/3];
prob.bcs.xTerm = [1.0;1.0;-1*pi/4];
prob.bcs.xTerm = [2.0;1.0;-2*pi/4];

% initial guess of time
T0 = 1.5;

% initialise with turn to right angle but in wrong place
turn0 = (prob.bcs.xTerm(3)-prob.bcs.xInit(3))/((prob.lims.maxSpeed)*T0);

% decision variable starts with time for whole
prob.x0 = [T0;
           repmat([prob.lims.maxSpeed;
             turn0], prob.steps.nSteps, 1)];

prob.UB = [inf;
           repmat([prob.lims.maxSpeed;
                  prob.lims.maxTurn], prob.steps.nSteps, 1)];
prob.LB = [0;
           repmat([ 0;
                  -prob.lims.maxTurn], prob.steps.nSteps, 1)];

end

function J = shoot1Cost(prob,x)

% time
J = x(1);

end

function [C,Ceq] = shoot1Cons(prob,x)

xs = shoot1(prob,x);

C = [];

% obstacle loc
%xo = [2;0];
%Ro = 0.5;
%ds = [xs(1,:)-xo(1);xs(2,:)-xo(2)];
%Co = Ro*Ro - sum(ds.*ds)';
%C = [C; Co];

Ceq = [xs(1:2,end) - prob.bcs.xTerm(1:2);
       xs(3,end) - prob.bcs.xTerm(3)];
%       cos(xs(3,end)) - cos(prob.bcs.xTerm(3));
%       sin(xs(3,end)) - sin(prob.bcs.xTerm(3))];

end

function xs = shoot1(prob,x,nDec)

%optional decimation for smoother views
if ~exist('nDec'),
    nDec = 1;
end

dt = x(1)/prob.steps.nSteps;
us = x(2:end);
mydt = dt/nDec;
mysteps = prob.steps.nSteps*nDec;

xs(:,1) = prob.bcs.xInit;
for kk=1:mysteps,
    myku = floor((kk-1)/nDec);
    xs(:,kk+1) = f_rk4(xs(:,kk),us(2*myku+(1:2)),mydt);
end

end

function xnext = f_rk4(x,u,dt)

k1 = f(x,u);
k2 = f(x+0.5*dt*k1,u);
k3 = f(x+0.5*dt*k2,u);
k4 = f(x+dt*k3,u);
xnext = x+dt*(k1+2*k2+2*k3+k4)/6;

end

function xdot = f(x,u)

xdot = [u(1)*cos(x(3));
    u(1)*sin(x(3));
    u(1)*u(2)];

end