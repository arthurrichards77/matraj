function flag = isVisible(x1,x2,walls)
%
% flag = isVisible(x1,x2,walls)
%
% x1, x2 are 2x1 points
% Each row of walls is [xa ya xb yb]
%
% flag=1 if visible OK
%

% number of walls
nWalls = size(walls,1);

% initially OK
flag=1;
% now check against each wall
for ww=1:nWalls,
    if linesIntersect(x1,x2,walls(ww,1:2)',walls(ww,3:4)')>0,
        % they intersect - fail
        flag=0;
        % stop wall check here - only need one failure
        break
    end
end