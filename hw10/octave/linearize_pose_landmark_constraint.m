% Compute the error of a pose-landmark constraint
% x 3x1 vector (x,y,theta) of the robot pose
% l 2x1 vector (x,y) of the landmark
% z 2x1 vector (x,y) of the measurement, the position of the landmark in
%   the coordinate frame of the robot given by the vector x
%
% Output
% e 2x1 error of the constraint
% A 2x3 Jacobian wrt x
% B 2x2 Jacobian wrt l
function [e, A, B] = linearize_pose_landmark_constraint(x, l, z)

  % TODO compute the error and the Jacobians of the error
  X_i = v2t(x);
  R_i = X_i(1:2,1:2);
  t_i = X_i(1:2,3);
  e = R_i'*(l-t_i) - z;

  R_i_dot = [-sin(x(3)) -cos(x(3)); cos(x(3)) -sin(x(3))];
  A = [-R_i' R_i_dot'*(l-t_i)];
  B = R_i';


end;
