% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error
  X_i = v2t(x1);
  X_j = v2t(x2);
  Z_ij = v2t(z);
  R_ij = Z_ij(1:2,1:2);
  R_i = X_i(1:2,1:2);
  ti = x1(1:2);
  tj = x2(1:2);
  e = t2v(inv(Z_ij)*(inv(X_i)*X_j));

  R_i_dot = [-sin(x1(3)) -cos(x1(3)); cos(x1(3)) -sin(x1(3))];
  A = [[-R_ij'*R_i' R_ij'*R_i_dot'*(tj-ti)];[0 0 -1]];
  B = [[R_ij'*R_i' [0;0]];[0 0 1]];
end;
