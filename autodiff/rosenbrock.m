function f = rosenbrock(u)

x = u(1);
y = u(2);

f=(1-x).*(1-x)+100*(y-x.*x).*(y-x.*x);