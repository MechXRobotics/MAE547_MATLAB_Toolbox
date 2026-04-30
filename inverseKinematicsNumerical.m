function [q_sol, success, history, ikInfo] = inverseKinematicsNumerical(robot, pd, q0, opts)
% INVERSEKINEMATICSNUMERICAL
% Iterative position-only inverse kinematics using the linear part of the
% geometric Jacobian.
%
% Outputs:
%   q_sol   : solution joint vector
%   success : convergence flag
%   history : iteration history
%   ikInfo  : symbolic forward-kinematics information used as the underlying
%             position model for the IK computation

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

    % Symbolic forward kinematics used as the underlying model.
    ikInfo = struct();
    ikInfo.model_type = 'position_only_numerical_IK';
    ikInfo.symbolic_fk = symbolicFkineDH(robot);
    ikInfo.symbolic_position = ikInfo.symbolic_fk.p_ee;
    ikInfo.target_pd = pd;
    ikInfo.method = opts.method;

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
                dq = opts.alpha * Jp.' * ((Jp * Jp.' + lambda^2 * eye(3)) \ e);

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

            ikInfo.q_solution = q_sol;
            ikInfo.success = success;
            ikInfo.final_error = history.errNorm(end);
            return;
        end
    end

    history.q = history.q(:,1:opts.maxIter);
    history.errNorm = history.errNorm(1:opts.maxIter);
    history.position = history.position(:,1:opts.maxIter);

    q_sol = q;
    ikInfo.q_solution = q_sol;
    ikInfo.success = success;
    ikInfo.final_error = history.errNorm(end);
end