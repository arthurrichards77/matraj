classdef DecVar < OptExp
   
    methods
        function obj = DecVar(x)
            obj@OptExp(x,eye(numel(x)));
        end
    end
    
end