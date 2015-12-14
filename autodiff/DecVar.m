classdef DecVar < OptExp
   
    methods
        function obj = DecVar(x)
            obj@OptExp(x,speye(numel(x)));
        end
    end
    
end