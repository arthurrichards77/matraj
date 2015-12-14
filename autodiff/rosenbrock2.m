function [f,g] = rosenbrock2(x)

z = DecVar(x);

w = rosenbrock(z);

f=value(w);
g=grad(w);