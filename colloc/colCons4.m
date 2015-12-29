function [C,Ceq,CG,CGeq] = colCons4(prob,x)

xv = DecVar(x);
[Co,~] = colConsNoEqs(prob,xv);

C=value(Co);
CG=sparse(grad(Co)');

Ceq=[];
CGeq=[];

% grab
%C2 = C;
%Ceq2 = Ceq;

% overwrite to see what's going on
%[C,Ceq] = colCons(prob,x);

%C2-C
%Ceq2-Ceq

end