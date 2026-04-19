function [ve, pdot, omega] = velocityKinematics(robot, q, qdot)
% VELOCITYKINEMATICS Forward differential kinematics.
%
% Inputs:
%   robot : robot struct
%   q     : nx1 or 1xn joint configuration
%   qdot  : nx1 or 1xn joint velocity
%
% Outputs:
%   ve    : 6x1 end-effector twist [linear; angular]
%   pdot  : 3x1 linear velocity
%   omega : 3x1 angular velocity

    q = q(:);
    qdot = qdot(:);

    if numel(q) ~= robot.n || numel(qdot) ~= robot.n
        error('q and qdot must each have n elements.');
    end

    J = geometricJacobian(robot, q);
    ve = J * qdot;

    pdot = ve(1:3);
    omega = ve(4:6);
end