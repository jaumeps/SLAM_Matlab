function [e, J_e_rob1, J_e_rob2] = error_move(rob1, rob2, drobmeas)

% in:
%  rob1: pose of robot at time 1
%  rob1: pose of robot at time 1
%  drob: motion measurement between times 1 and 2
% 
% out:
%   e: expected motion error (per modificar l'error)
%   J_e_rob1: Jacobian of error wrt. robot1
%   J_e_rob2: Jacobian of error with respect to rob2

% expected measurement

[drobexp, J_drobexp_rob1, J_drobexp_rob2] = betweenFrames2D(rob1, rob2);

% expectation error

e = drobexp - drobmeas;
J_e_drobexp = eye(3);

% Chain rule
J_e_rob1 = J_e_drobexp * J_drobexp_rob1;
J_e_rob2 = J_e_drobexp * J_drobexp_rob2;

end

