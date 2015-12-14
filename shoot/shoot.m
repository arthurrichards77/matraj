function [res, xres] = shoot

prob = shootSetup();

UB = repmat([prob.lims.maxSpeed;
             prob.lims.maxTurn], prob.steps.nSteps, 1);
LB = repmat([0;
             -prob.lims.maxTurn], prob.steps.nSteps, 1);

res = fmincon(@(x)shoot1Cost(prob,x),prob.x0,[],[],[],[],LB,UB,@(x)shoot1Cons(prob,x));

xres = shoot1(prob,res);

plot(xres(1,:),xres(2,:),'.b-',...
     prob.bcs.xInit(1),prob.bcs.xInit(2),'gs',...
     prob.bcs.xTerm(1),prob.bcs.xTerm(2),'gx')
axis equal

end

function prob = shootSetup

prob.steps.nSteps = 20;
prob.steps.dt = 0.1;

prob.lims.maxTurn = 2.0;
prob.lims.maxSpeed = 4.0;

prob.bcs.xInit = [0;0;-0*pi/3];
prob.bcs.xTerm = [4.0;0.0;0*pi/6];

prob.x0 = repmat([0.5*prob.lims.maxSpeed;
             0*prob.lims.maxTurn], prob.steps.nSteps, 1);

end

function J = shoot1Cost(prob,x)

J = prob.steps.dt*sum(x(1:2:end));

end

function [C,Ceq] = shoot1Cons(prob,x)

xs = shoot1(prob,x);

% obstacle loc
xo = [2;0];
Ro = 0.5;
ds = [xs(1,:)-xo(1);xs(2,:)-xo(2)];

Co = Ro*Ro - sum(ds.*ds)';

C = [Co];
Ceq = [xs(1:2,end) - prob.bcs.xTerm(1:2);
       xs(3,end) - prob.bcs.xTerm(3)];
%       cos(xs(3,end)) - cos(prob.bcs.xTerm(3));
%       sin(xs(3,end)) - sin(prob.bcs.xTerm(3))];

end

function xs = shoot1(prob,us)

xs(:,1) = prob.bcs.xInit;
for kk=1:prob.steps.nSteps,
    xs(:,kk+1) = f_rk4(xs(:,kk),us(2*(kk-1)+(1:2)),prob.steps.dt);
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