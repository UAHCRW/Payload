function C_new=Find_Rat_Mat(C_old, gyro_vals, dt)
%given inputs from the IMU, update the rotation matrix
% Inputs:
%   C_old, rotation matrix from previous time step
%   gyro_vals, gyroscope readings with biases removed, 1x3 xyz format
%           units of rad/s
%   dt, time step, in s
%Output:
%   C_new: updated rotation matrix

sigma=norm(gyro_vals*dt); %norm is just the vector norm, sqrt (x^2+y^2+z^2)

% used negative angles instead of transposing in the interest
% of time
B=[0, w(3)*dt, -w(2)*dt; ...
  -w(3)*dt, 0,w(1)*dt;...
   w(2)*dt,-w(1)*dt,0];
%then can find the matrix to multiply C_old by
Update_Mat=eye(3)+(sin(sigma)/sigma)*B+((1-cos(sigma))/sigma^2)*B^2;
%eye is just the 3x3 identity matrix
C_new=C_old*Update_Mat;

