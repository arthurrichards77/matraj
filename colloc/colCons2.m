function [C,Ceq,CG,CGeq] = colCons2(prob,x)

xv = DecVar(x);
[Co,Ceqo] = colCons(prob,xv);

C=value(Co);
CG=grad(Co)';

Ceq=value(Ceqo);
CGeq=grad(Ceqo)';

% grab
%C2 = C;
%Ceq2 = Ceq;

% overwrite to see what's going on
%[C,Ceq] = colCons(prob,x);

%C2-C
%Ceq2-Ceq

end