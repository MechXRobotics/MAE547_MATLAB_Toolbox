function [JA, xe, phi, TA, Tphi] = analyticalJacobianZYZ(robot, q)
% ANALYTICALJACOBIANZYZ
% Compute textbook analytical Jacobian J_A for operational variables
% x_e = [p_e; phi_e], where phi_e are ZYZ Euler angles.
%
% Based on:
%   xdot_e = J_A(q) qdot
%   J = T_A(phi) J_A
% hence
%   J_A = T_A(phi) \ J
%
% Outputs:
%   JA   : 6xn analytical Jacobian
%   xe   : 6x1 operational variable [p; phi]
%   phi  : 3x1 ZYZ Euler angles
%   TA   : 6x6 transformation matrix
%   Tphi : 3x3 orientation-rate mapping matrix

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