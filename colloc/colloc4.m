% using automatic differentiation
% but with no nonlinear equalities

% ensure autodiff on path
if ~exist('DecVar','class'),
    addpath('../autodiff')
end

prob = collocSetup;

% options = optimoptions('fmincon','MaxFunEvals',50000,'MaxIter',10000);
% tic
% [x,Jopt,flag,op] = fmincon(@(x)colCost(prob,x),prob.x0,[],[],[],[],[],[],@(x)colCons(prob,x),options)
% t1 = toc

% get the equality constraints
[C,Ceq,CG,CGeq]=colCons2(prob,0*prob.x0);

% version with autodiff expressions
options = optimset('fmincon');
options = optimset(options,'MaxFunEvals',50000,'MaxIter',10000);
options = optimset(options,'GradObj','on','GradConstr','on');
%options = optimoptions('fmincon','MaxFunEvals',50000,'MaxIter',10000,'GradObj','on','GradConstr','on','DerivativeCheck','on');
tic
[x,Jopt,flag,op] = fmincon(@(x)colCost2(prob,x),prob.x0,[],[],CGeq',-Ceq,[],[],@(x)colCons4(prob,x),options)
t2 = toc

[xvals,yvals,xdots,ydots,xDdots,yDdots] = collocTraj(prob,x);
[x0vals,y0vals] = collocTraj(prob,prob.x0);
plot(xvals,yvals,'.b-', ...
     x0vals,y0vals,'.g-', ...
     x(1:2:end-1),x(2:2:end-1),'xr:')
axis equal
