classdef SerialManipulator < OpenKinematicChain
    properties
        
    end
    
    methods
        
        function obj = SerialManipulator(name,rho,DH_c,m,cm,I,g)
            
            N = numel(rho);
            parent_idx = zeros(1,N);
            for k = 1:N,
                parent_idx(k) = k-1;
            end
            DH_p = zeros(N,4);
            
            obj = obj@OpenKinematicChain(name,parent_idx,rho,DH_p,DH_c,m,cm,I,g);
            
        end
        
    end
    
end