function results = simulateImpedanceControl(robot, q0, qdot0, xWall, Ke, De, Tsim)
% SIMULATEIMPEDANCECONTROL
% Chapter 9 impedance control with force measurement.
% Inverse dynamics + force compensation + desired Cartesian impedance.

    if nargin < 7 || isempty(Tsim)
        Tsim = 3.0;
    end

    t = linspace(0, Tsim, 501);
    dt = t(2) - t(1);
    tf = Tsim;
    N = numel(t);
    
    n  = robot.n;

    q = q0(:);
    qdot = qdot0(:);

    [T0_all, ~] = fkineDH(robot, q);
    p0 = T0_all(1:3, 4, end);

    if nargin < 4 || isempty(xWall)
        xWall = p0(1) + 0.05;
    end

    if nargin < 5 || isempty(Ke)
        Ke = 1500;
    end

    if nargin < 6 || isempty(De)
        De = 25;
    end

    pd_final = [xWall + 0.05; p0(2) + 0.05; p0(3)];

    Md      = diag([1, 1, 1]);
    Dd      = diag([80, 80, 80]);
    Kp_imp  = diag([500, 500, 500]);

    lambda_dls = 1e-4;

    qHist = zeros(n, N);
    qdotHist = zeros(n, N);
    pHist = zeros(3, N);
    pdHist = zeros(3, N);
    fHist = zeros(3, N);
    tauHist = zeros(n, N);
    penetrationHist = zeros(1, N);

    for k = 1:N
        time = t(k);

        [T_all, ~] = fkineDH(robot, q);
        p = T_all(1:3, 4, end);

        J = geometricJacobian(robot, q);
        Jp = J(1:3, :);
        pdot_vec = Jp * qdot;

        s = min(time / tf, 1);
        sigma = 3*s^2 - 2*s^3;
        sigma_dot = (6*s - 6*s^2) / tf;
        sigma_ddot = (6 - 12*s) / (tf^2);

        if s >= 1
            sigma_dot = 0;
            sigma_ddot = 0;
        end

        dp = pd_final - p0;
        pd = p0 + sigma * dp;
        pd_dot = sigma_dot * dp;
        pd_ddot = sigma_ddot * dp;

        penetration = p(1) - xWall;

        if penetration > 0 && (Ke > 0 || De > 0)
            Fn = Ke * penetration + De * pdot_vec(1);
            Fn = max(Fn, 0);
            Fenv = [-Fn; 0; 0];
        else
            Fenv = zeros(3, 1);
        end

        B = massMatrix(robot, q);
        C = coriolisMatrix(robot, q, qdot);
        gq = gravityVector(robot, q);

        x_err = pd - p;
        xdot_err = pd_dot - pdot_vec;

        xdd_cmd = pd_ddot + Md \ (Dd * xdot_err + Kp_imp * x_err + Fenv);

        qdd_cmd = Jp.' / (Jp * Jp.' + lambda_dls * eye(3)) * xdd_cmd;

        tauEnv = Jp.' * Fenv;
        tauControl = B * qdd_cmd + C * qdot + gq - tauEnv;

        qdd = B \ (tauControl + tauEnv - C*qdot - gq);

        qdot_old = qdot;
        qdot = qdot + dt * qdd;
        q = q + dt * qdot_old;

        qHist(:, k) = q;
        qdotHist(:, k) = qdot;
        pHist(:, k) = p;
        pdHist(:, k) = pd;
        fHist(:, k) = Fenv;
        tauHist(:, k) = tauControl;
        penetrationHist(k) = max(penetration, 0);
    end

    results.t = t;
    results.q = qHist;
    results.qdot = qdotHist;
    results.p = pHist;
    results.pd = pdHist;
    results.Fenv = fHist;
    results.tau = tauHist;

    results.xWall = xWall;
    results.Ke = Ke;
    results.De = De;
    results.Md = Md;
    results.Dd = Dd;
    results.Kp_imp = Kp_imp;
    results.penetration = penetrationHist;

    results.finalPosition = pHist(:,end);
    results.finalForce = fHist(:,end);
    results.finalQ = qHist(:,end);
    results.finalQdot = qdotHist(:,end);
    results.finalPenetration = penetrationHist(end);

    Kp_x = Kp_imp(1,1);
    xd_x = pdHist(1,end);
    xr = xWall;

    equilibriumCheck = struct();
    equilibriumCheck.valid = false;
    equilibriumCheck.reason = "";

    equilibriumCheck.Kp_x = Kp_x;
    equilibriumCheck.xd_x = xd_x;
    equilibriumCheck.xWall = xr;
    equilibriumCheck.Ke = Ke;
    equilibriumCheck.De = De;

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

    plotForceControlResults(results, 'Impedance Control: Inverse Dynamics');

end