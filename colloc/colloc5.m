% this version uses only nonlinear inequalities
% the equalities are all expressed as matrix equality constraints
% now using symbolc gradient from Maple

prob = collocSetup;

%options = optimoptions('fmincon','MaxFunEvals',50000,'MaxIter',10000);
options = optimset('fmincon');
options = optimset(options,'MaxFunEvals',50000,'MaxIter',10000);
options = optimset(options,'GradObj','on','GradConstr','on');
%options = optimset(options,'DerivativeCheck','on');

% get the equality constraints
[C,Ceq,CG,CGeq]=colCons2(prob,0*prob.x0);

tic
[x,Jopt,flag,op] = fmincon(@(x)colCostSymGrad(prob,x),prob.x0,[],[],CGeq',-Ceq,[],[],@(x)colConsNoEqsSymGrad(prob,x),options)
t3=toc

%x = fmincon(@(x)colCost2(prob,x),prob.x0,[],[],[],[],[],[],@(x)colCons2(prob,x),options);

[xvals,yvals,xdots,ydots,xDdots,yDdots] = collocTraj(prob,x);
[x0vals,y0vals] = collocTraj(prob,prob.x0);
plot(xvals,yvals,'.b-', ...
     x0vals,y0vals,'.g-', ...
     x(1:2:end-1),x(2:2:end-1),'xr:')
axis equal
