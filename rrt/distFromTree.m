function [minDist,minNode]=distFromTree(newNode,nodes)
%
% [minDist,minNode]=distFromTree(newNode,nodes)
%
% nodes is 2 x N list of node locations
% newNode is 2x1 node location
% minDist is closest distance to any node in nodes
% minNode is index of that node
%

% number of nodes
nNodes = size(nodes,2);
% start with relative positions
deltas = newNode*ones(1,nNodes) - nodes;
% convert to distances, squared
dists = sum(deltas.*deltas);
% try one-norm
%dists = sum(abs(deltas));
% find nearest
[minDist,minNode]=min(dists);
% take square root for accurate distance
minDist=sqrt(minDist);