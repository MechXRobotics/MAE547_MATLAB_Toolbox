function [T_all, p_all, fkInfo] = fkineDH(robot, q)
% FKINEDH
% Numerical forward kinematics using standard DH convention.
%
% Outputs:
%   T_all  : 4x4x(n+1) transforms of base and successive link frames
%   p_all  : (n+1)x3 positions of base and successive frame origins
%   fkInfo : optional struct containing symbolic forward-kinematics data
%
% Notes:
%   - Numerical result is unchanged in spirit from your existing code.
%   - The final tool transform is now applied consistently to both T_all and p_all.
%   - If requested, fkInfo contains the symbolic representation used before
%     numerical substitution.

    q = q(:);
    if numel(q) ~= robot.n
        error('q must have n elements.');
    end

    T_all = zeros(4, 4, robot.n + 1);
    p_all = zeros(robot.n + 1, 3);

    T = robot.base;
    T_all(:,:,1) = T;
    p_all(1,:) = T(1:3,4).';

    for i = 1:robot.n
        if robot.type(i) == "R"
            theta_i = robot.theta(i) + q(i);
            d_i = robot.d(i);
        else
            theta_i = robot.theta(i);
            d_i = robot.d(i) + q(i);
        end

        A_i = dhTransform(robot.a(i), robot.alpha(i), d_i, theta_i);
        T = T * A_i;

        T_all(:,:,i+1) = T;
        p_all(i+1,:) = T(1:3,4).';
    end

    % Apply the tool transform consistently to the final frame.
    T = T * robot.tool;
    T_all(:,:,end) = T;
    p_all(end,:) = T(1:3,4).';

    if nargout >= 3
        fkInfo = symbolicFkineDH(robot);
        fkInfo.q_numeric = q;
        fkInfo.T_ee_numeric = T_all(:,:,end);
        fkInfo.p_ee_numeric = p_all(end,:).';
    end
end

function A = dhTransform(a, alpha, d, theta)
% Standard DH transform:
% A = Rot(z,theta) * Trans(z,d) * Trans(x,a) * Rot(x,alpha)

    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);

    A = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
         0,      sa,     ca,    d;
         0,       0,      0,    1];
end