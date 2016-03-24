function [res, xres] = multishoot

prob = shootSetup();

options = optimset('fmincon');
options = optimset(options,'MaxFunEvals',10000);

res = fmincon(@(x)shoot1Cost(prob,x),prob.x0,[],[],[],[],prob.LB,prob.UB,@(x)shoot1Cons(prob,x),options);

xres = reshapex(prob,res);
xres0 = reshapex(prob,prob.x0);

plot(xres(1,:),xres(2,:),'.b-',...
     xres0(1,:),xres0(2,:),'.g-',...
     prob.bcs.xInit(1),prob.bcs.xInit(2),'gs',...
     prob.bcs.xTerm(1),prob.bcs.xTerm(2),'gx')
axis equal

end

function prob = shootSetup

prob.steps.nSteps = 12;
prob.steps.dt = 0.05;

prob.lims.maxTurn = 2.0;
prob.lims.maxSpeed = 4.0;

prob.bcs.xInit = [0;0;0*pi/3];
prob.bcs.xTerm = [1.0;1.0;3*pi/6];

prob.x0 = [linspace(prob.bcs.xInit(1), prob.bcs.xTerm(1), prob.steps.nSteps);
           linspace(prob.bcs.xInit(2), prob.bcs.xTerm(2), prob.steps.nSteps);
           linspace(prob.bcs.xInit(3), prob.bcs.xTerm(3), prob.steps.nSteps);
           repmat([0.5*prob.lims.maxSpeed;
                      0*prob.lims.maxTurn], 1, prob.steps.nSteps);];
prob.x0 = reshape(prob.x0,5*prob.steps.nSteps,1);
prob.x0 = prob.x0(4:end);
         
prob.UB = repmat([inf;
                  inf;
                  inf;
                  prob.lims.maxSpeed;
                  prob.lims.maxTurn], prob.steps.nSteps, 1);
prob.UB = prob.UB(4:end);

prob.LB = repmat([ -inf;
                  -inf;
                  -inf;
                  0;
                  -prob.lims.maxTurn], prob.steps.nSteps, 1);
prob.LB = prob.LB(4:end);

end

function J = shoot1Cost(prob,x)

% distance, by penalizing speed * time
J = prob.steps.dt*sum(x(1:5:end));

end

function [C,Ceq] = shoot1Cons(prob,x)

xs = reshapex(prob,x);

C = [];

Ceq = [];
for kk=2:(prob.steps.nSteps+1),
    Ceq = [Ceq; 
           xs(1:3,kk) - f_rk4(xs(1:3,kk-1),xs(4:5,kk-1),prob.steps.dt)];
end
% Ceq = [Ceq; 
%        prob.bcs.xTerm - f_rk4(xs(1:3,end),xs(4:5,end),prob.steps.dt)];
    
end

function xs = reshapex(prob,x)

xs = reshape([prob.bcs.xInit; x; prob.bcs.xTerm; 0; 0],5,prob.steps.nSteps+1);

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