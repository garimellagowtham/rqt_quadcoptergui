function [TrajectoryCost,xs, constants] = EvaluateTrajectory(system_params, us, ts, zs, constants)
%Evaluate the Cost and Dynamics
%system_params is 5x1 vector [Control Gain, External Forcex, ExternalForcey,r_x, p_x]
%constants is struct with members: Cost Matrices R,P, mass(m), initial
%system_params guess(system_params_mean), z acceleration (z_offset)
N = size(us,2);
%Create x0 from zs:
x0 = zeros(6,1);
x0(1:3) = zs(:,1);%1st position
x0(4:6) = (zs(:,2) - zs(:,1))/(ts(2) - ts(1));
xs = zeros(6,N+1);
if nargout >= 2
    xs(:,1) = x0;
end
TrajectoryCost = 0;
constants.throttle = zeros(N+1,1);
constants.throttle(1) = system_params(1)*us(5,1);
for i=1:N
   [x1, constants] = quadcopter_dynamics(i, x0, (ts(2) - ts(1)), us(:,i), system_params, constants);
   if nargout >= 2
    xs(:,i+1) = x1;
   end
   [TrajectoryCost] = TrajectoryCost + Cost(zs(:,i+1), x1, ts(2) - ts(1), constants);
   x0 = x1;
end
error_parameter = (system_params - constants.system_params_mean);
TrajectoryCost = TrajectoryCost + 0.5*error_parameter'*constants.P*error_parameter;
end

function [L] = Cost(z, x, h,constants)
error_sensor = (z-x(1:3));
L = 0.5*h*(error_sensor'*constants.R*error_sensor);
end

function [xnew, constants] = quadcopter_dynamics(i, xold, h, u, system_params, constants)
xnew = zeros(6,1);
constants.throttle(i+1) = constants.throttle(i) + h*system_params(2)*(system_params(1)*u(5) - constants.throttle(i));
%Plot Throttle
R_offset = rpy2mat([0, system_params(6), system_params(5)]);%Yaw Pitch Roll
R = quat2mat([u(4); u(1); u(2); u(3)]);
xnew(4:6) = xold(4:6) + (h/constants.mass)*R*R_offset*[0;0;1]*(constants.throttle(i+1)) + h*[system_params(3); system_params(4); constants.z_offset];
vel_avg = (xnew(4:6) + xold(4:6))/2;
xnew(1:3) = xold(1:3) + h*vel_avg;
end