function [JA, xe, phi, TA, Tphi] = analyticalJacobianZYZ(robot, q)

    q = q(:);
    if numel(q) ~= robot.n
        error('q must have n elements.');
    end

    [T_all, ~] = fkineDH(robot, q);
    T_ee = T_all(:,:,end);
    R = T_ee(1:3,1:3);
    p = T_ee(1:3,4);

    phi = rotationMatrixToZYZ(R);
    varphi = phi(1);
    theta  = phi(2);

    Tphi = [0, -sin(varphi),  cos(varphi)*sin(theta);
            0,  cos(varphi),  sin(varphi)*sin(theta);
            1,  0,            cos(theta)];

    if abs(det(Tphi)) < 1e-10
        error('Representation singularity for ZYZ Euler angles: T(phi) is singular.');
    end

    TA = [eye(3), zeros(3,3);
          zeros(3,3), Tphi];

    J = geometricJacobian(robot, q);
    JA = TA \ J;

    xe = [p; phi];
end