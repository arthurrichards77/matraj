% starting point
x0 = [2;-3];

% basic
[X1,F1,E1,OP1] = fminunc(@rosenbrock,x0)

% and with autodiff cleverness

[X2,F2,E2,OP2] = fminunc(@rosenbrock2,x0)

options = optimoptions('fminunc','GradObj','on');
[X3,F3,E3,OP3] = fminunc(@rosenbrock2,x0,options)