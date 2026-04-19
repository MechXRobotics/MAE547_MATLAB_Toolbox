function [qdot, J, info] = inverseVelocityKinematics(robot, q, ve_des, method, damping)
% INVERSEVELOCITYKINEMATICS Inverse differential kinematics.
%
% Inputs:
%   robot   : robot struct
%   q       : nx1 joint configuration
%   ve_des  : 6x1 desired end-effector twist [v; omega]
%   method  : 'pinv', 'dls', or 'transpose'
%   damping : damping factor for DLS, e.g. 1e-2
%
% Outputs:
%   qdot    : nx1 joint velocity command
%   J       : 6xn Jacobian
%   info    : struct with conditioning data

    if nargin < 4 || isempty(method)
        method = 'pinv';
    end
    if nargin < 5 || isempty(damping)
        damping = 1e-2;
    end

    q = q(:);
    ve_des = ve_des(:);

    if numel(q) ~= robot.n
        error('q must have n elements.');
    end
    if numel(ve_des) ~= 6
        error('ve_des must have 6 elements.');
    end

    J = geometricJacobian(robot, q);

    switch lower(method)
        case 'pinv'
            qdot = pinv(J) * ve_des;

        case 'dls'
            % Damped least-squares
            qdot = J.' * ((J*J.' + (damping^2)*eye(6)) \ ve_des);

        case 'transpose'
            alpha = 0.1;
            qdot = alpha * J.' * ve_des;

        otherwise
            error('Unknown method. Use ''pinv'', ''dls'', or ''transpose''.');
    end

    s = svd(J);
    info.singularValues = s;
    info.rank = rank(J);
    if isempty(s)
        info.condApprox = Inf;
    elseif min(s) < 1e-12
        info.condApprox = Inf;
    else
        info.condApprox = max(s)/min(s);
    end
end