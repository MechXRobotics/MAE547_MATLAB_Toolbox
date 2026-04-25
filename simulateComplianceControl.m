function results = simulateComplianceControl(robot, q0, qdot0, xWall, Ke, De, pd, Tsim)
% SIMULATECOMPLIANCECONTROL
%
% Chapter 9 active compliance control with a one-sided virtual wall.
%
% xWall = environment/wall rest position in x
% Ke = environment stiffness [N/m]
% De = environment damping [N*s/m]
% pd = desired Cartesian end-effector position [x; y; z]
%
% Returns results with:
% results.equilibriumCheck.expectedX
% results.equilibriumCheck.expectedFx
% results.equilibriumCheck.xError
% results.equilibriumCheck.fxError

    if nargin < 8 || isempty(Tsim)
        Tsim = 3.0;
    end

    t = linspace(0, Tsim, 501);
    dt = t(2) - t(1);
    N = numel(t);
    n = robot.n;
    q = q0(:);
    qdot = qdot0(:);
    

    % Defaults
    [T0_all, ~] = fkineDH(robot, q0);
    p0 = T0_all(1:3,4,end);

    if nargin < 4 || isempty(xWall)
        xWall = p0(1) + 0.05;
    end
    
    if nargin < 5 || isempty(Ke)
        Ke = 1500;
    end
    
    if nargin < 6 || isempty(De)
        De = 25;
    end

    if nargin < 7 || isempty(pd)
        pd = p0 + [0.20; 0; 0];
    end

    pd_cart = pd(:);

    % Kp_x must be nonzero
    Kp_cart = diag([200, 200, 200]);
    Kd_cart = diag([20,  20,  20]);

    % hold values in case
    qHist = zeros(n, N);
    qdotHist = zeros(n, N);
    pHist = zeros(3, N);
    pdHist = zeros(3, N);
    fHist = zeros(3, N);
    penetrationHist = zeros(1, N);

    for k = 1:N

        % Forward kinematics
        [T_all, ~] = fkineDH(robot, q);
        p = T_all(1:3, 4, end);

        % Jacobian and end effector velocity
        J  = geometricJacobian(robot, q);
        Jp = J(1:3, :);
        pdot_vec = Jp * qdot;


        % If x <= xWall, no touch
        % If x > xWall, wall pushes robot in negative x
        penetration = p(1) - xWall;

        if penetration > 0 && (Ke > 0 || De > 0)
            Fn = Ke * penetration + De * pdot_vec(1);
            Fn = max(Fn, 0);
            Fenv = [-Fn; 0; 0];
        else
            Fenv = zeros(3, 1);
        end

        % dynamics
        B  = massMatrix(robot, q);
        C  = coriolisMatrix(robot, q, qdot);
        gq = gravityVector(robot, q);

        % compliance controller
        p_err = pd_cart - p;
        v_err = -pdot_vec;

        F_task = Kp_cart * p_err + Kd_cart * v_err;

        tauControl = Jp.' * F_task + gq;
        tauEnv = Jp.' * Fenv;

        qdd = B \ (tauControl + tauEnv - C*qdot - gq);

        % Integrate
        qdot_old = qdot;
        qdot = qdot + dt * qdd;
        q = q    + dt * qdot_old;

        qHist(:, k) = q;
        qdotHist(:, k) = qdot;
        pHist(:, k) = p;
        pdHist(:, k) = pd_cart;
        fHist(:, k) = Fenv;
        penetrationHist(k) = max(penetration, 0);
    end

    % update result for print and plot
    results.t = t;
    results.q = qHist;
    results.qdot = qdotHist;
    results.p = pHist;
    results.pd = pdHist;
    results.Fenv = fHist;
    results.xWall = xWall;
    results.Ke = Ke;
    results.De = De;
    results.Kp_cart = Kp_cart;
    results.Kd_cart = Kd_cart;
    results.penetration = penetrationHist;
    results.finalPosition = pHist(:,end);
    results.finalForce = fHist(:,end);
    results.finalQ = qHist(:,end);
    results.finalQdot = qdotHist(:,end);
    results.finalPenetration = penetrationHist(end);

    % check x touch
    Kp_x = Kp_cart(1,1);
    xd_x = pd_cart(1);
    xr = xWall;

    equilibriumCheck = struct();
    equilibriumCheck.valid = false;
    equilibriumCheck.reason = "";

    equilibriumCheck.Kp_x = Kp_x;
    equilibriumCheck.xd_x = xd_x;
    equilibriumCheck.xWall = xr;
    equilibriumCheck.Ke = Ke;

    equilibriumCheck.expectedX = NaN;
    equilibriumCheck.simulatedX = pHist(1,end);
    equilibriumCheck.xError = NaN;

    equilibriumCheck.expectedFx = NaN;
    equilibriumCheck.simulatedFx = fHist(1,end);
    equilibriumCheck.fxError = NaN;

    equilibriumCheck.expectedPenetration = NaN;
    equilibriumCheck.simulatedPenetration = penetrationHist(end);

    if Kp_x > 0 && Ke > 0 && xd_x > xr
        x_expected = (Kp_x * xd_x + Ke * xr) / (Kp_x + Ke);
        Fx_expected = -Ke * (x_expected - xr);

        equilibriumCheck.valid = true;
        equilibriumCheck.reason = "Valid x-contact equilibrium check.";

        equilibriumCheck.expectedX = x_expected;
        equilibriumCheck.simulatedX = pHist(1,end);
        equilibriumCheck.xError = pHist(1,end) - x_expected;

        equilibriumCheck.expectedFx = Fx_expected;
        equilibriumCheck.simulatedFx = fHist(1,end);
        equilibriumCheck.fxError = fHist(1,end) - Fx_expected;

        equilibriumCheck.expectedPenetration = x_expected - xr;
        equilibriumCheck.simulatedPenetration = penetrationHist(end);

    elseif Ke == 0 && De == 0
        equilibriumCheck.reason = "Skipped: Ke = 0 and De = 0, so no environment/contact force is generated.";
    elseif Ke == 0
        equilibriumCheck.reason = "Skipped: Ke = 0, so the spring equilibrium formula is not valid.";
    elseif xd_x <= xr
        equilibriumCheck.reason = "Skipped: desired x is not beyond xWall, so steady x-contact is not expected.";
    else
        equilibriumCheck.reason = "Skipped: equilibrium check conditions were not met.";
    end

    results.equilibriumCheck = equilibriumCheck;

    % Plot
    plotForceControlResults(results, 'Compliance Control: Active Compliance');

end