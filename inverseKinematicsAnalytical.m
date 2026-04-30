function [q_sol, success, history, ikInfo] = inverseKinematicsAnalytical(robot, xd, q0, opts)
% INVERSEKINEMATICSANALYTICAL
% Iterative full-pose inverse kinematics using the analytical Jacobian.
%
% Inputs:
%   xd = [p_d; phi_d] where phi_d is in ZYZ Euler angles
%
% Outputs:
%   q_sol   : solution joint vector
%   success : convergence flag
%   history : iteration history
%   ikInfo  : symbolic forward-kinematics information used as the underlying
%             pose model for the IK computation

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
    history.xe = zeros(6, opts.maxIter + 1);

    success = false;

    % Symbolic forward kinematics used as the underlying model.
    ikInfo = struct();
    ikInfo.model_type = 'full_pose_analytical_IK';
    ikInfo.symbolic_fk = symbolicFkineDH(robot);
    ikInfo.target_xd = xd;
    ikInfo.method = opts.method;

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
                qdot = JA.' * ((JA * JA.' + lambda^2 * eye(size(JA,1))) \ vref);

            otherwise
                error('Unknown method.');
        end

        q = q + opts.dt * qdot;

        if opts.useLimits && isfield(robot, 'qlim') && ~isempty(robot.qlim)
            for i = 1:robot.n
                q(i) = min(max(q(i), robot.qlim(i,1)), robot.qlim(i,2));
            end
        end

        history.q(:,k) = q;
        history.e(:,k) = e;
        history.errNorm(k) = norm(e);
        history.xe(:,k) = xe;

        if norm(e) < opts.tol
            success = true;
            history.q = history.q(:,1:k);
            history.e = history.e(:,1:k);
            history.errNorm = history.errNorm(1:k);
            history.xe = history.xe(:,1:k);
            q_sol = q;

            ikInfo.q_solution = q_sol;
            ikInfo.success = success;
            ikInfo.final_error = history.errNorm(end);
            return;
        end
    end

    history.q = history.q(:,1:opts.maxIter);
    history.e = history.e(:,1:opts.maxIter);
    history.errNorm = history.errNorm(1:opts.maxIter);
    history.xe = history.xe(:,1:opts.maxIter);

    q_sol = q;
    ikInfo.q_solution = q_sol;
    ikInfo.success = success;
    ikInfo.final_error = history.errNorm(end);
end