function [cost,grd] = colCost2(prob,x)

xv = DecVar(x);
op = colCost(prob,xv);

cost=value(op);
grd=grad(op);

end