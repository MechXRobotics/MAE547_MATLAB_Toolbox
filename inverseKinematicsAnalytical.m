function [q_sol, success, history] = inverseKinematicsAnalytical(robot, xd, q0, opts)
% INVERSEKINEMATICSANALYTICAL
% Textbook-style inverse kinematics using analytical Jacobian and
% operational-space error e = x_d - x_e with ZYZ Euler angles.
%
% xd   = [p_d; phi_d]  (6x1), with phi_d = [varphi; theta; psi]
%
% Supported methods:
%   'inverse' : qdot = J_A^{-1}(xdot_d + K e)
%   'pinv'    : qdot = pinv(J_A)(xdot_d + K e)
%   'dls'     : qdot = J_A' (J_A J_A' + lambda^2 I)^(-1) (xdot_d + K e)

    if nargin < 4
        opts = struct();
    end

    if ~isfield(opts, 'maxIter'),   opts.maxIter = 200; end
    if ~isfield(opts, 'tol'),       opts.tol = 1e-6; end
    if ~isfield(opts, 'dt'),        opts.dt = 0.05; end
    if ~isfield(opts, 'method'),    opts.method = 'dls'; end
    if ~isfield(opts, 'lambda'),    opts.lambda = 1e-2; end
    if ~isfield(opts, 'K'),         opts.K = diag([2 2 2 2 2 2]); end
    if ~isfield(opts, 'xdot_d'),    opts.xdot_d = zeros(6,1); end
    if ~isfield(opts, 'useLimits'), opts.useLimits = true; end

    xd = xd(:);
    q = q0(:);

    if numel(xd) ~= 6
        error('xd must be 6x1: [p_d; phi_d] with ZYZ Euler angles.');
    end
    if numel(q) ~= robot.n
        error('q0 must have n elements.');
    end

    history.q = zeros(robot.n, opts.maxIter + 1);
    history.e = zeros(6, opts.maxIter + 1);
    history.errNorm = zeros(opts.maxIter + 1, 1);

    success = false;

    for k = 1:opts.maxIter
        [JA, xe] = analyticalJacobianZYZ(robot, q);
        e = xd - xe;

        vref = opts.xdot_d + opts.K * e;

        switch lower(opts.method)
            case 'inverse'
                if size(JA,1) ~= size(JA,2)
                    error('JA is not square; use pinv or dls.');
                end
                qdot = JA \ vref;

            case 'pinv'
                qdot = pinv(JA) * vref;

            case 'dls'
                lambda = opts.lambda;
                qdot = JA.' * ((JA*JA.' + lambda^2*eye(size(JA,1))) \ vref);

            otherwise
                error('Unknown method.');
        end

        q = q + opts.dt * qdot;

        if opts.useLimits && isfield(robot, 'qlim')
            for i = 1:robot.n
                q(i) = min(max(q(i), robot.qlim(i,1)), robot.qlim(i,2));
            end
        end

        history.q(:,k) = q;
        history.e(:,k) = e;
        history.errNorm(k) = norm(e);

        if norm(e) < opts.tol
            success = true;
            history.q = history.q(:,1:k);
            history.e = history.e(:,1:k);
            history.errNorm = history.errNorm(1:k);
            q_sol = q;
            return;
        end
    end

    history.q = history.q(:,1:opts.maxIter);
    history.e = history.e(:,1:opts.maxIter);
    history.errNorm = history.errNorm(1:opts.maxIter);
    q_sol = q;
end