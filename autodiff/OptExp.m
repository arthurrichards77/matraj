classdef OptExp
    properties
        vals
        sens
    end
    methods
        % constructor
        function obj = OptExp(vals,sens)
            % need a sensitivity row for every value
            assert(size(sens,1)==numel(vals))
            obj.vals = vals;
            obj.sens = sens;
        end
        
        % methods for evaluation of value and gradient
        function y = value(self)
            y = self.vals;
        end
        function dydx = grad(self)
            dydx = self.sens;
        end
                
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % numerical operations (not overloading, yet)
        function y = scale(self,k)
                % multiply by scalar constant
                y = OptExp(k*self.vals,k*self.sens);            
        end
        
        function y = offset(self,k)
            % add constant
            y = OptExp(self.vals+k,self.sens);
        end
        
        function y = sum(obj1,obj2)
            % add two expressions
            y = OptExp(obj1.vals+obj2.vals, ...
                obj1.sens+obj2.sens);
        end
        
        function y = subtract(obj1,obj2)
            % subtract two expressions obj1-obj2
            y = OptExp(obj1.vals-obj2.vals,...
                obj1.sens-obj2.sens);
        end
        
        function y = mult(obj1,obj2)
            % multiply two expressions, elementwise
            y = OptExp(obj1.vals.*obj2.vals, ...
                diag(obj1.vals)*obj2.sens + diag(obj2.vals)*obj1.sens);
        end
        
        function y = stack2(obj1,obj2)
            % stack two expressions (vertical concatenation)
            y = OptExp([obj1.vals;obj2.vals], ...
                [obj1.sens; obj2.sens]);
        end
                
        function [y] = stackn(a,b,varargin)
            % stack more than two (vertical concatenation)
            y = stack2(a,b);
            n = nargin-2;
            for kk=1:n,
                y = stack2(y,varargin{kk});
            end
        end
        
        function y = elems(self,ind)
            % extract elements
            y = OptExp(self.vals(ind), ...
                self.sens(ind,:));
        end
        
        function y = div(u,v)
            % divide two expressions
            val = (u.vals)./(v.vals);
            sns = u.sens; % placeholder
            for kk = 1:numel(val),
                sns(kk,:) = (v.vals(kk)*u.sens(kk,:) - u.vals(kk)*v.sens(kk,:))/(v.vals(kk)*v.vals(kk));
            end
            y = OptExp(val,sns);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % trying overloading
        
        % a.*b
        function y = times(obj1,obj2)
            % only valid for multiplying two expressions
            if isa(obj1,'OptExp')&&isa(obj2,'OptExp'),
                y = mult(obj1,obj2);
            else
                error('This type of multiplication not supported')
            end
        end
        
        % a./b
        function y = rdivide(u,v)
            % only valid for multiplying two expressions
            if isa(u,'OptExp')&&isa(v,'OptExp'),
                y = div(u,v);
            else
                error('This type of division not supported')
            end
        end
        
        % a*b
        function y = mtimes(obj1,obj2)
            % only valid for premult by constant
            if ~isa(obj1,'OptExp')&&isa(obj2,'OptExp'),
                y = scale(obj2,obj1);
            else
                error('This type of multiplication not supported')
            end
        end
        
        % a+b
        function y = plus(obj1,obj2)
            % need to detect different options
            if isa(obj1,'OptExp')&&isa(obj2,'OptExp'),
                y = sum(obj1,obj2);
            elseif isa(obj1,'OptExp'),
                y = offset(obj1,obj2);
            elseif isa(obj2,'OptExp'),
                y = offset(obj2,obj1);
            else
                error('This type of multiplication not supported')
            end
        end
        
        % a-b
        function y = minus(obj1,obj2)
            % need to detect different options
            if isa(obj1,'OptExp')&&isa(obj2,'OptExp'),
                y = subtract(obj1,obj2);
            elseif isa(obj1,'OptExp'),
                y = offset(obj1,-obj2);
            elseif isa(obj2,'OptExp'),
                y = offset(scale(obj2,-1),obj1);
            else
                error('This type of multiplication not supported')
            end
        end
        
        % a(i)
        function y = subsref(self,S)            
            switch S(1).type,
                case '()'
                    assert(numel(S.subs)==1)
                    y = elems(self,S.subs{1});
                otherwise
                    error('Cannot subscript this way.')
            end
        end
        
        % [a;b]
        function y = vertcat(a,b,varargin)
            y = stackn(a,b,varargin{:});
        end
        
        % number of elements
        function n = numel(self)
            n = numel(self.vals);
        end
    end
end