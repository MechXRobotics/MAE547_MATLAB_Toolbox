function B = massMatrix(robot, q)
% MASSMATRIX Joint-space inertia matrix B(q).
%
% Uses:
%   B(q) = sum_i [ m_i Jv_i' Jv_i + Jw_i' R_i I_i R_i' Jw_i ]
%
% Assumes inertia rows are stored as:
%   [Ixx Iyy Izz Ixy Ixz Iyz]
% in the link-local frame.

    q = q(:);
    n = robot.n;

    if numel(q) ~= n
        error('q must have n elements.');
    end

    B = zeros(n,n);

    [T_all, ~] = fkineDH(robot, q);

    for i = 1:n
        T_link = T_all(:,:,i+1);
        T_com = T_link * [eye(3), robot.com(i,:).'; 0 0 0 1];

        p_com = T_com(1:3,4);
        R_com = T_com(1:3,1:3);

        [Jv, Jw] = pointJacobian(robot, q, p_com, i, T_all);

        I_body = inertiaRowToMatrix(robot.inertia(i,:));
        I_base = R_com * I_body * R_com.';

        B = B + robot.mass(i) * (Jv.' * Jv) + Jw.' * I_base * Jw;
    end

    % Symmetrize against numerical drift
    B = 0.5 * (B + B.');
end

function [Jv, Jw] = pointJacobian(robot, q, p_target, linkIndex, T_all)
% Jacobian for a point rigidly attached to link linkIndex.

    n = robot.n;
    Jv = zeros(3,n);
    Jw = zeros(3,n);

    for j = 1:n
        if j <= linkIndex
            T_prev = T_all(:,:,j);
            z = T_prev(1:3,3);
            p = T_prev(1:3,4);

            if robot.type(j) == "R"
                Jv(:,j) = cross(z, p_target - p);
                Jw(:,j) = z;
            else
                Jv(:,j) = z;
                Jw(:,j) = [0;0;0];
            end
        end
    end
end

function I = inertiaRowToMatrix(row)
% Convert [Ixx Iyy Izz Ixy Ixz Iyz] to a 3x3 tensor.

    Ixx = row(1);
    Iyy = row(2);
    Izz = row(3);
    Ixy = row(4);
    Ixz = row(5);
    Iyz = row(6);

    I = [Ixx Ixy Ixz;
         Ixy Iyy Iyz;
         Ixz Iyz Izz];
end