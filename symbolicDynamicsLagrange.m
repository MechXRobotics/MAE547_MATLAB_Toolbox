function dyn = symbolicDynamicsLagrange(robot)

    if ~license('test', 'Symbolic_Toolbox')
        error('Symbolic Math Toolbox is required for symbolic equations of motion.');
    end

    n = robot.n;

    % symbolic generalized coordinates
    q   = sym('q',   [n 1], 'real');
    qd  = sym('qd',  [n 1], 'real');
    qdd = sym('qdd', [n 1], 'real');

    % numeric constants promoted to symbolic
    a_const     = sym(robot.a(:).');
    alpha_const = sym(robot.alpha(:).');
    d_const     = sym(robot.d(:).');
    th_const    = sym(robot.theta(:).');
    mass_const  = sym(robot.mass(:).');
    grav        = sym(robot.gravity(:));
    com_const   = sym(robot.com);
    iner_const  = sym(robot.inertia);

    % symbolic forward kinematics to each link frame
    T_all = cell(n+1,1);
    T = sym(eye(4));
    T_all{1} = T;

    for i = 1:n
        if robot.type(i) == "R"
            theta_i = th_const(i) + q(i);
            d_i     = d_const(i);
        else
            theta_i = th_const(i);
            d_i     = d_const(i) + q(i);
        end

        A_i = dhTransformSym(a_const(i), alpha_const(i), d_i, theta_i);
        T = simplify(T * A_i, 'Steps', 20);
        T_all{i+1} = T;
    end

    % kinetic and potential energy terms via textbook inertia matrix structure
    B = sym(zeros(n,n));
    U = sym(0);

    for i = 1:n
        T_link = T_all{i+1};
        R_i = T_link(1:3,1:3);

        p_com_h = T_link * [com_const(i,:).'; 1];
        p_com = simplify(p_com_h(1:3), 'Steps', 20);

        % linear Jacobian from COM position
        Jp = jacobian(p_com, q);

        % angular Jacobian
        Jo = sym(zeros(3,n));
        for j = 1:n
            if j <= i
                T_prev = T_all{j};
                z = T_prev(1:3,3);

                if robot.type(j) == "R"
                    Jo(:,j) = z;
                else
                    Jo(:,j) = sym([0;0;0]);
                end
            end
        end

        I_body = inertiaRowToMatrixSym(iner_const(i,:));
        I_base = simplify(R_i * I_body * R_i.', 'Steps', 20);

        B = B + mass_const(i) * (Jp.' * Jp) + Jo.' * I_base * Jo;

        U = U - mass_const(i) * grav.' * p_com;
    end

    B = simplify((B + B.')/2, 'Steps', 50);
    g = simplify(jacobian(U, q).', 'Steps', 50);

    % Christoffel-based C matrix
    C = sym(zeros(n,n));
    for i = 1:n
        for j = 1:n
            cij = sym(0);
            for k = 1:n
                cijk = sym(1/2) * ( ...
                    diff(B(i,j), q(k)) + ...
                    diff(B(i,k), q(j)) - ...
                    diff(B(j,k), q(i)) );
                cij = cij + cijk * qd(k);
            end
            C(i,j) = simplify(cij, 'Steps', 20);
        end
    end

    tau = simplify(B*qdd + C*qd + g, 'Steps', 50);

    eqns = sym(zeros(n,1));
    labels = cell(n,1);
    for i = 1:n
        if robot.type(i) == "R"
            labels{i} = sprintf('tau%d', i);
        else
            labels{i} = sprintf('f%d', i);
        end
        eqns(i) = tau(i);
    end

    dyn.q = q;
    dyn.qd = qd;
    dyn.qdd = qdd;
    dyn.B = B;
    dyn.C = C;
    dyn.g = g;
    dyn.tau = tau;
    dyn.eqns = eqns;
    dyn.labels = labels;
end

function A = dhTransformSym(a, alpha, d, theta)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);

    A = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
         0,      sa,     ca,    d;
         0,       0,      0,    1];
end

function I = inertiaRowToMatrixSym(row)
    Ixx = row(1); Iyy = row(2); Izz = row(3);
    Ixy = row(4); Ixz = row(5); Iyz = row(6);

    I = [Ixx Ixy Ixz;
         Ixy Iyy Iyz;
         Ixz Iyz Izz];
end