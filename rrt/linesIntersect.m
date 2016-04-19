function [flag] = linesIntersect(a1,a2,b1,b2)
%
% [flag] = linesIntersect(a1,a2,b1,b2)
%
% all inputs should be 2 x 1 vectors
%
% flag = 2 if they strictly intersect
%        1 if they intersect at an endpoint
%        0 otherwise
%
% check if a1-a2 and b1-b2 intersect
%

% tolerance
tol = 1e-9;

dx = a2(1) - a1(1);
dy = a2(2) - a1(2);
da = b2(1) - b1(1);
db = b2(2) - b1(2);

if (da * dy - db * dx) == 0,
  flag = 0;
else,
  lam = (dx * (b1(2) - a1(2)) + dy * (a1(1) - b1(1))) / (da * dy - db * dx);
  gam = (da * (a1(2) - b1(2)) + db * (b1(1) - a1(1))) / (db * dx - da * dy);
  % test 
  if (lam>=tol)&&(lam<=1-tol)&&(gam>=tol)&&(gam<=1-tol),
    flag = 2;
  elseif (lam>=0)&&(lam<=1)&&(gam>=0)&&(gam<=1),
    flag = 1;
  else
    flag = 0;
  end
end
