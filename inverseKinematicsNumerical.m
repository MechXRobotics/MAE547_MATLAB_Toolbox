function [q_sol, success, history] = inverseKinematicsNumerical(robot, pd, q0, opts)
% INVERSEKINEMATICSNUMERICAL Numerical position-only inverse kinematics.
%
% Inputs:
%   robot : robot struct
%   pd    : 3x1 desired end-effector position
%   q0    : nx1 initial guess
%   opts  : struct with optional fields:
%           .maxIter   (default 200)
%           .tol       (default 1e-5)
%           .alpha     (default 0.5)
%           .method    (default 'pinv')  % 'pinv' or 'dls'
%           .damping   (default 1e-2)
%           .useLimits (default true)
%
% Outputs:
%   q_sol   : nx1 solution
%   success : logical
%   history : struct with iteration history

    if nargin < 4
        opts = struct();
    end

    if ~isfield(opts, 'maxIter'),   opts.maxIter = 200; end
    if ~isfield(opts, 'tol'),       opts.tol = 1e-5; end
    if ~isfield(opts, 'alpha'),     opts.alpha = 0.5; end
    if ~isfield(opts, 'method'),    opts.method = 'pinv'; end
    if ~isfield(opts, 'damping'),   opts.damping = 1e-2; end
    if ~isfield(opts, 'useLimits'), opts.useLimits = true; end

    pd = pd(:);
    q = q0(:);

    if numel(pd) ~= 3
        error('pd must be a 3x1 vector.');
    end
    if numel(q) ~= robot.n
        error('q0 must have n elements.');
    end

    history.q = zeros(robot.n, opts.maxIter + 1);
    history.errNorm = zeros(opts.maxIter + 1, 1);
    history.position = zeros(3, opts.maxIter + 1);

    success = false;

    for k = 1:opts.maxIter
        [T_all, ~] = fkineDH(robot, q);
        T_ee = T_all(:,:,end);
        p = T_ee(1:3,4);

        e = pd - p;
        J = geometricJacobian(robot, q);
        Jp = J(1:3,:);

        switch lower(opts.method)
            case 'pinv'
                dq = opts.alpha * pinv(Jp) * e;

            case 'dls'
                lambda = opts.damping;
                dq = opts.alpha * Jp.' * ((Jp*Jp.' + lambda^2*eye(3)) \ e);

            otherwise
                error('Unknown IK method. Use ''pinv'' or ''dls''.');
        end

        q = q + dq;

        if opts.useLimits && isfield(robot, 'qlim') && ~isempty(robot.qlim)
            for i = 1:robot.n
                q(i) = min(max(q(i), robot.qlim(i,1)), robot.qlim(i,2));
            end
        end

        history.q(:,k) = q;
        history.errNorm(k) = norm(e);
        history.position(:,k) = p;

        if norm(e) < opts.tol
            success = true;
            history.q = history.q(:,1:k);
            history.errNorm = history.errNorm(1:k);
            history.position = history.position(:,1:k);
            q_sol = q;
            return;
        end
    end

    history.q = history.q(:,1:opts.maxIter);
    history.errNorm = history.errNorm(1:opts.maxIter);
    history.position = history.position(:,1:opts.maxIter);
    q_sol = q;
end