function [C,Ceq,CG,CGeq] = colCons2(prob,x)

xv = DecVar(x);
[Co,Ceqo] = colCons(prob,xv);

C=value(Co);
CG=grad(Co);
Ceq=value(Ceqo);
CGeq=grad(Ceqo);

end