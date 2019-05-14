function [A,r] = buildproblem(states,factor)

K = numel(factor);

% A = zeros(num_poses_factors*pose_size+num_motion_factors*motion_size+num_lmk_factors*lmk_size, num_poses_states*pose_size+num_lmk_states*lmk_sizes);
A = zeros(25,17);
r = zeros(25, 1);

row = 1;

for k = 1:K
  
   y = factor{k}.measurement;
   W = (factor{k}.covariance) ^-1;
   Wt2 = chol(W, 'upper');
   
   factor{k}.type;
   
    switch factor{k}.type
       
        case 'motion'
            i = factor{k}.index(1);             % robot id
            j = factor{k}.index(2);             % lmk id
            rob1 = states{1+ i};
            rob2 = states{1+ j};
            [e, J_e_rob1, J_e_rob2] = error_move(rob1.value, rob2.value, y);
            
            rows = [row : (row + numel(y) -1)];
            
            r(rows) = Wt2*e;
            A(rows ,rob1.range) = Wt2*J_e_rob1;
            A(rows ,rob2.range) = Wt2*J_e_rob2;
            
               
        case 'lmk'
            i = factor{k}.index(1);
            j = factor{k}.index(2);
            rob = states{1+ i};
            lmk = states{1+ j};
            [e, J_e_rob, J_e_lmk] = error_observe(rob.value, lmk.value, y);
            
            rows = [row : (row + numel(y) -1)];
            
            r(rows) = Wt2*e;
            A(rows ,rob.range) = Wt2*J_e_rob;
            A(rows ,lmk.range) = Wt2*J_e_lmk;
            
            
        case 'pose'
            i = factor{k}.index(1);
            rob = states{1+ i};
            [e, J_e_rob] = error_pose(rob.value, y);
            
            rows = [row : (row + numel(y) -1)];
            
            r(rows) = Wt2*e;
            A(rows ,rob.range) = Wt2*J_e_rob;
            
    
    end
            
    row = row + numel(y);
    
end
end