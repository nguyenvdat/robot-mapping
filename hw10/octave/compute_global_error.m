% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);
  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)

    x1 = v2t(g.x(edge.fromIdx:edge.fromIdx+2));  % the first robot pose
    x2 = v2t(g.x(edge.toIdx:edge.toIdx+2));      % the second robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    Z_ij = v2t(edge.measurement);
    om = edge.information;
    e_ij = t2v(inv(Z_ij)*(inv(x1)*x2));
    Fx = Fx + e_ij'*om*e_ij;


  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    X_i = v2t(x);
    z_il = edge.measurement;
    om = edge.information;
    R_i = X_i(1:2,1:2);
    t_i = X_i(1:2,3);
    e_il = R_i'*(l-t_i) - z_il;
    Fx = Fx + e_il'*om*e_il;


  end

end
