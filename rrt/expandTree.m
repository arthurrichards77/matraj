function [nearNode,closestNode,newNodes,newArcs,newCosts] = expandTree(newNode,nodes,arcs,costs,walls,dMove)
%#eml
%
% [nearNode,closestNode,newNodes,newArcs] = expandTree(newNode,nodes,arcs,walls)
%
% existing tree is nodes (2 x N) and arcs (2 x M)
% walls - see "isVisible" for format
% newNode is new point to try and aim for
% dMove is distance to move
%
% returns nearNode (2 x 1) the actual location connected
% closestNode (1x1) index of closest node in the tree
% (both above blank if not able to connect)
% newNodes and newArcs are updated node and arc lists

% default returns
nearNode = [];
closestNode=[];
newNodes = nodes;
newArcs = arcs;
newCosts = costs;

% find nearest node on tree
[minDist,minNode]=distFromTree(newNode,nodes);

% move is shortest of 
dMove = min([dMove minDist]);

% "near" node is in direction from closest to new one
tryNode = nodes(:,minNode) + dMove*(newNode-nodes(:,minNode))/minDist;

% if visible, add a node
%if isVisibleMex(nodes(:,minNode),tryNode,walls')==1,
if isVisible(nodes(:,minNode),tryNode,walls)==1,
    % node counter
    nNodes = size(nodes,2);
    % add node
    newNodes(:,nNodes+1) = tryNode;
    % record its cost
    newCosts(nNodes+1) = costs(minNode)+dMove;
    % connect to closest
    newArcs(:,nNodes) = [minNode;nNodes+1];
    % return points
    nearNode = tryNode;
    closestNode = minNode;
end