function particles = correction_step(particles, z)

% Weight the particles according to the current map of the particle
% and the landmark observations z.
% z: struct array containing the landmark observations.
% Each observation z(j) has an id z(j).id, a range z(j).range, and a bearing z(j).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.

% Number of particles
numParticles = length(particles);

% Number of measurements in this time step
m = size(z, 2);

% TODO: Construct the sensor noise matrix Q_t (2 x 2)
Q_t = 0.1*eye(2);

% process each particle
for i = 1:numParticles
  robot = particles(i).pose;
  % process each measurement
  for j = 1:m
    % Get the id of the landmark corresponding to the j-th observation
    % particles(i).landmarks(l) is the EKF for this landmark
    l = z(j).id;

    % The (2x2) EKF of the landmark is given by
    % its mean particles(i).landmarks(l).mu
    % and by its covariance particles(i).landmarks(l).sigma

    % If the landmark is observed for the first time:
    if (particles(i).landmarks(l).observed == false)

      % TODO: Initialize its position based on the measurement and the current robot pose:
      A = z(j).range*z(j).range;
      b = normalize_angle(z(j).bearing + robot(3));
      B = tan(b);
      dy_square = A*B*B/(1+B*B);
      dx_square = A/(1+B*B);
      if (b <= pi && b > pi/2)
        dy = sqrt(dy_square);
        dx = -sqrt(dx_square);
      elseif (b <= pi/2 && b > 0)
        dy = sqrt(dy_square);
        dx = sqrt(dx_square);
      elseif (b <= 0 && b > -pi/2)
        dy = -sqrt(dy_square);
        dx = sqrt(dx_square);
      else
        dy = -sqrt(dy_square);
        dx = -sqrt(dx_square);
      end
      particles(i).landmarks(l).mu = [dx;dy] + robot(1:2);
      % get the Jacobian with respect to the landmark position
      [h, H] = measurement_model(particles(i), z(j));

      % TODO: initialize the EKF for this landmark
      particles(i).landmarks(l).sigma = inv(H)*Q_t*inv(H)';

      % Indicate that this landmark has been observed
      particles(i).landmarks(l).observed = true;

    else
      sigma_l = particles(i).landmarks(l).sigma;
      mu_l = particles(i).landmarks(l).mu;
      % get the expected measurement
      [expectedZ, H] = measurement_model(particles(i), z(j));

      % TODO: compute the measurement covariance
      Q = H*sigma_l*H' + Q_t;

      % TODO: calculate the Kalman gain
      K = sigma_l*H'/Q;

      % TODO: compute the error between the z and expectedZ (remember to normalize the angle)
      z_error = [z(j).range;z(j).bearing] - expectedZ;
      z_error(2) = normalize_angle(z_error(2));

      % TODO: update the mean and covariance of the EKF for this landmark
      particles(i).landmarks(l).mu = mu_l + K*z_error;
      particles(i).landmarks(l).sigma = sigma_l - K*H*sigma_l;

      % TODO: compute the likelihood of this observation, multiply with the former weight
      %       to account for observing several features in one time step
      z_llh = 1/sqrt(det(2*pi*Q))*exp(-1/2*z_error'/Q*z_error);
      particles(i).weight = particles(i).weight * z_llh;

    end

  end % measurement loop
end % particle loop

end
