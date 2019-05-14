function [y, J_y_rob, J_y_lmk] = observe(rob, lmk)

% transform to robot frame

[lmkrob, J_lmkrob_rob, J_lmkrob_lmk] = toFrame2D(rob, lmk);

% transform to polar coordinates

[y, J_y_lmkrob] = c2p(lmkrob);

% chain rule for Jacobians

J_y_rob = J_y_lmkrob * J_lmkrob_rob;

J_y_lmk = J_y_lmkrob * J_lmkrob_lmk;


end

function f()
%%
syms x y th pc py real
r = [x;y;th];
p = [px;py];

[yy, J_yy_r, J_yy_p] = observe(r,p);
simplify(jacobian(yy,r) - J_yy_r)
simplify(jacobian(yy,p) - J_yy_p)

end