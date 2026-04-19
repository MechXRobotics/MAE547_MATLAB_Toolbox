function gq = gravityVector(robot, q)
% GRAVITYVECTOR Gravity generalized force vector g(q).
%
% Computed from potential energy:
%   U(q) = - sum_i m_i * g' * p_com_i
% and g(q) = dU/dq numerically.

    q = q(:);
    n = robot.n;

    if numel(q) ~= n
        error('q must have n elements.');
    end

    h = 1e-6;
    gq = zeros(n,1);

    for k = 1:n
        dq = zeros(n,1);
        dq(k) = h;

        U_plus  = potentialEnergy(robot, q + dq);
        U_minus = potentialEnergy(robot, q - dq);

        gq(k) = (U_plus - U_minus) / (2*h);
    end
end

function U = potentialEnergy(robot, q)
    [T_all, ~] = fkineDH(robot, q);

    U = 0;
    g = robot.gravity(:);

    for i = 1:robot.n
        T_link = T_all(:,:,i+1);
        T_com = T_link * [eye(3), robot.com(i,:).'; 0 0 0 1];
        p_com = T_com(1:3,4);

        % Potential chosen so that gravity force is consistent with g vector
        U = U - robot.mass(i) * g.' * p_com;
    end
end