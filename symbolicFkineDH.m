function fkSym = symbolicFkineDH(robot)
% SYMBOLICFKINEDH
% Symbolic forward kinematics model using standard DH convention.
%
% Output struct fields:
%   qsym            symbolic joint variables q1...qn
%   A_all           cell array of symbolic link transforms
%   T_frames        cell array of symbolic frame transforms (base included)
%   T_ee            symbolic end-effector transform including tool
%   p_frames        cell array of frame-origin positions
%   p_ee            symbolic end-effector position
%   R_ee            symbolic end-effector rotation matrix
%   available       true if Symbolic Math Toolbox is available
%   note            explanatory text

    fkSym = struct();
    fkSym.available = false;
    fkSym.note = '';

    if ~license('test', 'Symbolic_Toolbox')
        fkSym.note = 'Symbolic Math Toolbox not available. Symbolic FK not generated.';
        return;
    end

    n = robot.n;
    qsym = sym('q', [n 1], 'real');

    a_const     = exactSymArray(robot.a(:).');
    alpha_const = exactSymArray(robot.alpha(:).');
    d_const     = exactSymArray(robot.d(:).');
    th_const    = exactSymArray(robot.theta(:).');

    T_base = exactSymArray(robot.base);
    T_tool = exactSymArray(robot.tool);

    A_all = cell(n, 1);
    T_frames = cell(n + 1, 1);
    p_frames = cell(n + 1, 1);

    T = T_base;
    T_frames{1} = T;
    p_frames{1} = T(1:3,4);

    for i = 1:n
        if robot.type(i) == "R"
            theta_i = th_const(i) + qsym(i);
            d_i = d_const(i);
        else
            theta_i = th_const(i);
            d_i = d_const(i) + qsym(i);
        end

        A_i = simplify(dhTransformSym(a_const(i), alpha_const(i), d_i, theta_i), 'Steps', 50);
        A_all{i} = A_i;

        T = simplify(T * A_i, 'Steps', 100);
        T_frames{i+1} = T;
        p_frames{i+1} = simplify(T(1:3,4), 'Steps', 50);
    end

    T_ee = simplify(T * T_tool, 'Steps', 100);
    p_ee = simplify(T_ee(1:3,4), 'Steps', 50);
    R_ee = simplify(T_ee(1:3,1:3), 'Steps', 50);

    fkSym.available = true;
    fkSym.note = 'Symbolic forward kinematics generated successfully.';
    fkSym.qsym = qsym;
    fkSym.A_all = A_all;
    fkSym.T_frames = T_frames;
    fkSym.T_ee = T_ee;
    fkSym.p_frames = p_frames;
    fkSym.p_ee = p_ee;
    fkSym.R_ee = R_ee;
end

function A = dhTransformSym(a, alpha, d, theta)
% Standard DH transform in symbolic form:
% A = Rot(z,theta) * Trans(z,d) * Trans(x,a) * Rot(x,alpha)

    ct = cos(theta); 
    st = sin(theta);
    ca = cos(alpha); 
    sa = sin(alpha);

    A = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
         0,      sa,     ca,    d;
         0,       0,      0,    1];
end

function S = exactSymArray(A)
% Convert numeric array to clean symbolic decimals.

    S = sym(zeros(size(A)));

    for ii = 1:size(A,1)
        for jj = 1:size(A,2)
            S(ii,jj) = exactSymScalar(A(ii,jj));
        end
    end
end

function s = exactSymScalar(x)
% Convert one numeric scalar into a clean symbolic decimal.

    if ~isscalar(x) || ~isfinite(x)
        error('Input must be a finite scalar.');
    end

    xr = round(x, 12);

    if abs(xr) < 1e-12
        xr = 0;
    end

    s = str2sym(num2str(xr, 12));
end