% simple straight line RRT for fun
% v2 - each expansion is the same length
% v3 - tree from both start and finish
% v4 - just expand each tree, don't try to connect to nearest
% v5 - keeps going instead of stopping as soon as 1st path found

close all
clear all

% initial and target positions
xF = [-14.5;-4.5];
x0 = [14.5;4.5];

% distance for each expansion
dMove = 0.3;

% operating box [xLo yLo xHi yHi]
worldBox = [-15 -5 15 5];

% finishing tolerance
dTol = dMove;

% wall locations [x1 y1 x2 y2]
wallsPots = [-1 1 1 1;
    -1 -1 1 -1;
    -1 1 -1 -1;
    1 1 1 0;
    0 0 1 0;
    -4 1 -2 1;
    -4 -1 -2 -1;
    -4 0 -4 -1;
    -2 -1 -2 1;
    -4 0 -3 0];

%walls = [-2 -1 -2 1];

wallsRand = (2*rand(40,4)-1)*[15 0 15 0;
                          0 5  0 5;
                          0 0  5 0;
                          0 0  0 5];

wallsZigzag = [(-10:5:10)' 3.1*sin((-10:5:10)'*pi/10)+3 (-10:5:10)' 3.1*sin((-10:5:10)'*pi/10)-3];

walls = wallsRand;
%walls = wallsZigzag;

% number of walls
nWalls = size(walls,1);

% initialise node and arc list
nodesS=x0;
arcsS=[1;1];
costsS=0;

% initialise tree from target
nodesF=xF;
arcsF=[1;1];
costsF=0;

% running progress plot
plotFlag=1;
%figure
plot(walls(:,[1 3])',walls(:,[2 4])','r','LineWidth',2)
axis equal
axis(worldBox([1 3 2 4]))
hold on
% placeholder for incumbent plot
hPath = plot(x0(1),x0(2),'k.','LineWidth',2);
incumb = inf;
pathCount=0;

% loop
for ii=1:200000,
    
    % default - not finished
    finFlag=0;
    
    % new node to aim for
    targNode = worldBox(1:2)'+rand(2,1).*(worldBox(3:4)'-worldBox(1:2)');
    
    % expand S tree towards it
    [newNode,closestNode,nodesS,arcsS,costsS] = expandTree(targNode,nodesS,arcsS,costsS,walls,dMove);
    
    % if node was added
    if ~isempty(newNode),
        
        % plot progress
        if exist('plotFlag','var'),
            if plotFlag>0,
                plot([newNode(1) nodesS(1,closestNode)],[newNode(2) nodesS(2,closestNode)],'g','LineWidth',2)
            end
        end
        
        % find closest distance to other tree
        [minDist,minNode]=distFromTree(newNode,nodesF);
        
        % check for end - first see if close
        if minDist<dTol,
            % then check visibility
            if isVisible(newNode,nodesF(:,minNode),walls)==1,
            %if isVisibleMex(newNode,nodesF(:,minNode),walls')==1,
                % end nodes are newest and closest on other tree
                endNodeF = minNode;
                endNodeS = size(nodesS,2);
                pathLength = costsS(endNodeS)+costsF(endNodeF)+minDist;
                % made it
                finFlag=1;
            end
        end
        
    end
    
    % expand F tree towards it
    [newNode,closestNode,nodesF,arcsF,costsF] = expandTree(targNode,nodesF,arcsF,costsF,walls,dMove);
    
    % if node was added
    if ~isempty(newNode),
        
        % plot progress
        if exist('plotFlag','var'),
            if plotFlag>0,
                plot([newNode(1) nodesF(1,closestNode)],[newNode(2) nodesF(2,closestNode)],'c','LineWidth',2)
                title(sprintf('Iter %i',ii))
                drawnow
            end
        end
        
        
        
        % find closest distance to other tree
        [minDist,minNode]=distFromTree(newNode,nodesS);
        
        % check for end - first see if close
        if minDist<dTol,
            % then check visibility
            if isVisible(newNode,nodesS(:,minNode),walls)==1,
            %if isVisibleMex(newNode,nodesS(:,minNode),walls')==1,
                % end nodes are newest and closest on other tree
                endNodeS = minNode;
                endNodeF = size(nodesF,2);
                pathLength = costsS(endNodeS)+costsF(endNodeF)+minDist;
                % made it
                finFlag=1;
            end
        end
        
    end
    
    if finFlag==1,
        
        %break
        
        if pathLength<incumb,
            incumb=pathLength;
            
            delete(hPath)
            
            % path through end tree
            nodePathF = pathToNode(arcsF,endNodeF);
            
            % path through start tree
            nodePathS = pathToNode(arcsS,endNodeS);
            
            % combine and plot
            pathNodes = [nodesS(:,nodePathS(end:-1:1)) nodesF(:,nodePathF)];
            hPath=plot(pathNodes(1,:),pathNodes(2,:),'k','LineWidth',2);
            
            pathCount = pathCount+1;
            if pathCount>2,
                break
            end
            
        end
        
    end
    
end

pause(2)
return

% final plot
cla
hold on
plot([nodesS(1,arcsS(1,:));nodesS(1,arcsS(2,:))],[nodesS(2,arcsS(1,:));nodesS(2,arcsS(2,:))],'g')
plot([nodesF(1,arcsF(1,:));nodesF(1,arcsF(2,:))],[nodesF(2,arcsF(1,:));nodesF(2,arcsF(2,:))],'c')
plot(walls(:,[1 3])',walls(:,[2 4])','r')

% stop here if failed to find it
if finFlag==0,
    return
end

% prepare to plot path
hold on

% path through end tree
nodePathF = pathToNode(arcsF,endNodeF);
plot(nodesF(1,nodePathF),nodesF(2,nodePathF),'k')

% path through start tree
nodePathS = pathToNode(arcsS,endNodeS);
plot(nodesS(1,nodePathS),nodesS(2,nodePathS),'k')

% link the two
plot([nodesS(1,endNodeS) nodesF(1,endNodeF)],[nodesS(2,endNodeS) nodesF(2,endNodeF)],'k')
