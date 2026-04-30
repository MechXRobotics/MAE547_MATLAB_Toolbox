function [tau, B, C, gq] = inverseDynamics(robot, q, qdot, qdd)

    q = q(:);
    qdot = qdot(:);
    qdd = qdd(:);

    n = robot.n;
    if numel(q) ~= n || numel(qdot) ~= n || numel(qdd) ~= n
        error('q, qdot, and qdd must each have n elements.');
    end

    B = massMatrix(robot, q);
    C = coriolisMatrix(robot, q, qdot);
    gq = gravityVector(robot, q);

    tau = B * qdd + C * qdot + gq;
end