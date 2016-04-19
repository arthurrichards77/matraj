function nodeList = pathToNode(arcs,destNode)

% current node starts as destination
thisNode = destNode;

% now work back through to goal from finish
for aa=1:size(arcs,2),
    % store (path runs backswards)
    nodeList(aa)=thisNode;
    % stop if got to 1, i.e. the start point
    if thisNode==1,
        break
    else,
        % find the arc leading to current node
        parentArc = find(arcs(2,:)==thisNode);
        % parent node of that arc is next node
        thisNode=arcs(1,parentArc);
    end
end