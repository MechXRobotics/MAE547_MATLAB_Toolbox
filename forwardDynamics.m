function [qdd, B, C, gq] = forwardDynamics(robot, q, qdot, tau)
    q = q(:);
    qdot = qdot(:);
    tau = tau(:);

    n = robot.n;
    if numel(q) ~= n || numel(qdot) ~= n || numel(tau) ~= n
        error('q, qdot, and tau must each have n elements.');
    end

    B = massMatrix(robot, q);
    C = coriolisMatrix(robot, q, qdot);
    gq = gravityVector(robot, q);

    qdd = B \ (tau - C * qdot - gq);
end