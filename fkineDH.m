function [T_all, p_all] = fkineDH(robot, q)
    q = q(:);
    if numel(q) ~= robot.n
        error('q must have n elements.');
    end

    T_all = zeros(4,4,robot.n+1);
    p_all = zeros(robot.n+1, 3);

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

    T_all(:,:,end) = T_all(:,:,end) * robot.tool;
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


% FKINEDH Standard DH forward kinematics for serial open-chain robots.
%
% Inputs:
%   robot : struct with fields n, type, a, alpha, d, theta, base, tool
%   q     : nx1 joint vector
%
% Outputs:
%   T_all : 4x4x(n+1), cumulative transforms from base to each frame
%   p_all : (n+1)x3 points of joint/frame origins in base coordinates