function new_vals=Euler_integration(old_vals, slope, dt)
%finds the next value using Euler's method
% Inputs:
%   vals, old position or velocity values, 1x3 format,xyz
%   slope, acceleration or velocity in this case, format xyz
%   dt, time step, in s
%Output:
%   new_vals: updated values

new_vals(:)=old_vals(:)+slope(:)*dt;