% ensure autodiff on path
if ~exist('DecVar','class'),
    addpath('../autodiff')
end

prob = collocSetup;

%options = optimoptions('fmincon','MaxFunEvals',50000,'MaxIter',10000);
%options = optimoptions('fmincon','MaxFunEvals',50000,'MaxIter',10000,'GradObj','on','GradConstr','on');
options = optimoptions('fmincon','MaxFunEvals',50000,'MaxIter',10000,'GradObj','on','GradConstr','on','DerivativeCheck','on');

% version with autodiff expressions
[x,Jopt,flag,op] = fmincon(@(x)colCost2(prob,x),prob.x0,[],[],[],[],[],[],@(x)colCons2(prob,x),options)

[xvals,yvals,xdots,ydots,xDdots,yDdots] = collocTraj(prob,x);
[x0vals,y0vals] = collocTraj(prob,prob.x0);
plot(xvals,yvals,'.b-', ...
     x0vals,y0vals,'.g-', ...
     x(1:2:end-1),x(2:2:end-1),'xr:')
axis equal
