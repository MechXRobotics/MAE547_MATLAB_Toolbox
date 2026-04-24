function [qHist, qdotHist, qddHist] = dynamicsPlotting(robot, q, qdot, tauApplied, t, dt)
%DYNAMICSPLOTTING 
%   plotting q, q_dot, q_ddot for a desired input as a function of time

    N = numel(t);

    qSim = q(:);
    qdotSim = qdot(:);
    tauApplied = tauApplied(:);

    qHist = zeros(robot.n, N);
    qdotHist = zeros(robot.n, N);
    qddHist = zeros(robot.n, N);

    qHist(:,1) = qSim;
    qdotHist(:,1) = qdotSim;

    for k = 1:N-1
        qddSim = forwardDynamics(robot, qSim, qdotSim, tauApplied);

        qddHist(:,k) = qddSim;
        qdotSim = qdotSim + qddSim * dt;
        qSim = qSim + qdotSim * dt;

        qdotHist(:,k+1) = qdotSim;
        qHist(:,k+1) = qSim;
    end

    qddHist(:,N) = forwardDynamics(robot, qSim, qdotSim, tauApplied);

    figure('Name', 'Dynamics Simulation Results');

    subplot(3,1,1);
    plot(t, qHist.');
    grid on;
    xlabel('Time (s)');
    ylabel('q');
    title('q(t)');
    legend(compose('q%d', 1:robot.n), 'Location', 'eastoutside');

    subplot(3,1,2);
    plot(t, qdotHist.');
    grid on;
    xlabel('Time (s)');
    ylabel('qdot');
    title('qdot(t)');
    legend(compose('qdot%d', 1:robot.n), 'Location', 'eastoutside');

    subplot(3,1,3);
    plot(t, qddHist.');
    grid on;
    xlabel('Time (s)');
    ylabel('qddot');
    title('qddot(t)');
    legend(compose('qddot%d', 1:robot.n), 'Location', 'eastoutside');

end

