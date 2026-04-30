function [ve, pdot, omega] = velocityKinematics(robot, q, qdot)
    q = q(:);
    qdot = qdot(:);

    if numel(q) ~= robot.n || numel(qdot) ~= robot.n
        error('q and qdot must each have n elements.');
    end

    J = geometricJacobian(robot, q);
    ve = J * qdot;

    pdot = ve(1:3);
    omega = ve(4:6);
end