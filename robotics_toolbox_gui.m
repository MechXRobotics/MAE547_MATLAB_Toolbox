function robotics_toolbox_gui()
    fig = uifigure('Name', 'MATLAB Robotics Toolbox', 'Position', [60 40 1780 980]);

    gl = uigridlayout(fig, [1 2]);
    gl.ColumnWidth = {590, '1x'};

    leftPanel = uipanel(gl, 'Title', 'Robot Definition and Inputs');

    leftGrid = uigridlayout(leftPanel, [25 2]);
    leftGrid.ColumnWidth = {215, '1x'};
    leftGrid.RowHeight = { ...
        24, ...
        24, ...
        24, ...
        32, ...
        190, ...
        24, ...
        24, ...
        85, ...
        24, ...
        85, ...
        24, ...
        24, ...
        24, ...
        24, ...
        24, ...
        24, ...
        28, ...
        36, 36, 36, 36, 36, 36, 36, ...
        '1x'};
    leftGrid.RowSpacing = 6;
    leftGrid.Padding = [8 8 8 8];

    uilabel(leftGrid, 'Text', 'Number of joints n:');
    nField = uieditfield(leftGrid, 'numeric', 'Value', 3, ...
        'Limits', [1 12], 'RoundFractionalValues', true);

    uilabel(leftGrid, 'Text', 'Units:');
    unitsDrop = uidropdown(leftGrid, 'Items', {'radians','degrees'}, 'Value', 'radians');

    uilabel(leftGrid, 'Text', 'Gravity [gx gy gz]:');
    gravField = uieditfield(leftGrid, 'text', 'Value', '[0 0 -9.81]');

    exampleBtn = uibutton(leftGrid, 'push', 'Text', 'Load Example: 3R Planar');
    exampleBtn.Layout.Column = [1 2];

    dhTable = uitable(leftGrid);
    dhTable.ColumnName = {'type','a','alpha','d','theta','q_home','qlim_min','qlim_max'};
    dhTable.ColumnEditable = true(1,8);
    dhTable.ColumnFormat = {{'R','P'}, 'numeric','numeric','numeric','numeric','numeric','numeric','numeric'};
    dhTable.Data = defaultDHTable(3);
    dhTable.Layout.Column = [1 2];

    uilabel(leftGrid, 'Text', 'Masses [kg]:');
    massesField = uieditfield(leftGrid, 'text', 'Value', '[1.2 0.9 0.5]');

    centerLabel = uilabel(leftGrid, ...
        'Text', 'Link center positions (local frame) Nx3 [x y z]:');
    centerLabel.Layout.Column = [1 2];

    comField = uitextarea(leftGrid, ...
        'Value', {'[0.25 0 0;', ...
                  ' 0.15 0 0;', ...
                  ' 0.10 0 0]'});
    comField.Layout.Column = [1 2];
    comField.FontName = 'Courier New';

    inertiaLabel = uilabel(leftGrid, ...
        'Text', 'Link inertia data Nx6 [Ixx Iyy Izz Ixy Ixz Iyz]:');
    inertiaLabel.Layout.Column = [1 2];

    inertiaField = uitextarea(leftGrid, ...
        'Value', {'[0.01 0.01 0.01 0 0 0;', ...
                  ' 0.008 0.008 0.008 0 0 0;', ...
                  ' 0.003 0.003 0.003 0 0 0]'});
    inertiaField.Layout.Column = [1 2];
    inertiaField.FontName = 'Courier New';

    uilabel(leftGrid, 'Text', 'q:');
    qField = uieditfield(leftGrid, 'text', 'Value', '[0.4 0.5 -0.3]');

    uilabel(leftGrid, 'Text', 'qdot:');
    qdotField = uieditfield(leftGrid, 'text', 'Value', '[0.1 0.2 -0.1]');

    uilabel(leftGrid, 'Text', 'qdd:');
    qddField = uieditfield(leftGrid, 'text', 'Value', '[0.2 -0.1 0.05]');

    uilabel(leftGrid, 'Text', 'tau:');
    tauField = uieditfield(leftGrid, 'text', 'Value', '[1 0.5 0.2]');

    uilabel(leftGrid, 'Text', 'Desired pd [x y z]:');
    pdField = uieditfield(leftGrid, 'text', 'Value', '[0.7 0.2 0]');

    uilabel(leftGrid, 'Text', 'Desired phi_d [ZYZ]:');
    phiField = uieditfield(leftGrid, 'text', 'Value', '[0 0 0]');

    uilabel(leftGrid, 'Text', 'Desired twist ve_d [vx vy vz wx wy wz]:');
    veField = uieditfield(leftGrid, 'text', 'Value', '[0.05 0 0 0 0 0]');

    buildBtn   = uibutton(leftGrid, 'push', 'Text', 'Build Robot');
    fkBtn      = uibutton(leftGrid, 'push', 'Text', '1. Forward Kinematics');
    animBtn    = uibutton(leftGrid, 'push', 'Text', '1a. Animate Home -> q');
    ikBtn      = uibutton(leftGrid, 'push', 'Text', '2. Inverse Kinematics');
    diffBtn    = uibutton(leftGrid, 'push', 'Text', '3. Differential Kinematics');
    invVelBtn  = uibutton(leftGrid, 'push', 'Text', '4. Inverse Velocity Kinematics');
    dynBtn     = uibutton(leftGrid, 'push', 'Text', '5. Dynamics');

    buildBtn.Layout.Column   = 1;
    fkBtn.Layout.Column      = 2;
    animBtn.Layout.Column    = 1;
    ikBtn.Layout.Column      = 2;
    diffBtn.Layout.Column    = 1;
    invVelBtn.Layout.Column  = 2;
    dynBtn.Layout.Column     = 1;

    rightPanel = uipanel(gl, 'Title', 'Visualization and Output');
    rightGrid = uigridlayout(rightPanel, [2 1]);
    rightGrid.RowHeight = {'2x','1x'};

    ax = uiaxes(rightGrid);
    title(ax, 'Robot Visualization');
    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    zlabel(ax, 'Z');
    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 135, 25);

    outArea = uitextarea(rightGrid, 'Editable', 'off');
    outArea.FontName = 'Courier New';
    outArea.Value = {'Ready. Enter inputs and click Build Robot, or load the example.'};

    app.robot = [];
    app.ax = ax;
    app.outArea = outArea;
    app.nField = nField;
    app.unitsDrop = unitsDrop;
    app.dhTable = dhTable;
    app.gravField = gravField;
    app.massesField = massesField;
    app.comField = comField;
    app.inertiaField = inertiaField;
    app.qField = qField;
    app.qdotField = qdotField;
    app.qddField = qddField;
    app.tauField = tauField;
    app.pdField = pdField;
    app.phiField = phiField;
    app.veField = veField;

    nField.ValueChangedFcn = @(src,evt) onNChanged();
    buildBtn.ButtonPushedFcn = @(src,evt) onBuildRobot();
    fkBtn.ButtonPushedFcn = @(src,evt) onForwardKinematics();
    animBtn.ButtonPushedFcn = @(src,evt) onAnimate();
    ikBtn.ButtonPushedFcn = @(src,evt) onInverseKinematics();
    diffBtn.ButtonPushedFcn = @(src,evt) onDifferentialKinematics();
    invVelBtn.ButtonPushedFcn = @(src,evt) onInverseVelocityKinematics();
    dynBtn.ButtonPushedFcn = @(src,evt) onDynamics();
    exampleBtn.ButtonPushedFcn = @(src,evt) onLoadExample();

    clearPlot();

    function clearPlot()
        cla(ax);
        title(ax, 'Robot Visualization');
        xlabel(ax, 'X');
        ylabel(ax, 'Y');
        zlabel(ax, 'Z');
        grid(ax, 'on');
        axis(ax, 'equal');
        view(ax, 135, 25);
        xlim(ax, [-1 1]);
        ylim(ax, [-1 1]);
        zlim(ax, [-1 1]);
    end

    function onLoadExample()
        selection = uiconfirm(fig, ...
            'Loading the example will overwrite the current robot inputs. Continue?', ...
            'Confirm Load Example', ...
            'Options', {'Continue','Cancel'}, ...
            'DefaultOption', 2, ...
            'CancelOption', 2);

        if strcmp(selection, 'Continue')
            load3RPlanarExample();
            onBuildRobot();
        end
    end

    function onNChanged()
        nNew = nField.Value;
        oldData = dhTable.Data;
        oldN = size(oldData, 1);

        newData = defaultDHTable(nNew);
        rowsToCopy = min(oldN, nNew);

        for i = 1:rowsToCopy
            for j = 1:size(oldData,2)
                newData{i,j} = oldData{i,j};
            end
        end

        dhTable.Data = newData;

        qField.Value = mat2str(zeros(1,nNew), 4);
        qdotField.Value = mat2str(zeros(1,nNew), 4);
        qddField.Value = mat2str(zeros(1,nNew), 4);
        tauField.Value = mat2str(zeros(1,nNew), 4);
        massesField.Value = mat2str(ones(1,nNew), 4);
        comField.Value = defaultCOMText(nNew);
        inertiaField.Value = defaultInertiaText(nNew);
        pdField.Value = '[0 0 0]';
        phiField.Value = '[0 0 0]';
        veField.Value = '[0 0 0 0 0 0]';

        clearPlot();
        outArea.Value = {'Inputs resized for new number of joints. Click Build Robot to redraw.'};
    end

    function load3RPlanarExample()
        nField.Value = 3;
        unitsDrop.Value = 'radians';
        gravField.Value = '[0 0 -9.81]';

        dhTable.Data = {
            'R', 0.5, 0, 0, 0, 0, -pi, pi;
            'R', 0.3, 0, 0, 0, 0, -pi, pi;
            'R', 0.2, 0, 0, 0, 0, -pi, pi
        };

        massesField.Value = '[1.2 0.9 0.5]';
        comField.Value = {'[0.25 0 0;', ...
                          ' 0.15 0 0;', ...
                          ' 0.10 0 0]'};
        inertiaField.Value = {'[0.01 0.01 0.01 0 0 0;', ...
                              ' 0.008 0.008 0.008 0 0 0;', ...
                              ' 0.003 0.003 0.003 0 0 0]'};
        qField.Value = '[0.4 0.5 -0.3]';
        qdotField.Value = '[0.1 0.2 -0.1]';
        qddField.Value = '[0.2 -0.1 0.05]';
        tauField.Value = '[1 0.5 0.2]';
        pdField.Value = '[0.7 0.2 0]';
        phiField.Value = '[0 0 0]';
        veField.Value = '[0.05 0 0 0 0 0]';
    end

    function robot = ensureRobot()
        robot = parseRobotFromUI(app);
        app.robot = robot;
    end

    function v = parseVector(field, n, name)
        v = str2num(field.Value); %#ok<ST2NM>
        validateattributes(v, {'numeric'}, {'vector','numel',n}, '', name);
        v = v(:);
    end

    function v = parse3(field, name)
        v = str2num(field.Value); %#ok<ST2NM>
        validateattributes(v, {'numeric'}, {'vector','numel',3}, '', name);
        v = v(:);
    end

    function v = parse6(field, name)
        v = str2num(field.Value); %#ok<ST2NM>
        validateattributes(v, {'numeric'}, {'vector','numel',6}, '', name);
        v = v(:);
    end

    function onBuildRobot()
        try
            robot = ensureRobot();
            q = parseVector(qField, robot.n, 'q');

            [T_all, p_all] = fkineDH(robot, q);

            cla(ax);
            plotRobot(ax, p_all, T_all);
            title(ax, 'Robot Visualization');

            outArea.Value = {'Robot built from current inputs.'};
        catch ME
            outArea.Value = {['Error: ' ME.message]};
        end
    end

    function onForwardKinematics()
        try
            robot = ensureRobot();
            q = parseVector(qField, robot.n, 'q');

            [T_all, p_all, fkInfo] = fkineDH(robot, q);
            T_ee = T_all(:,:,end);
            p_ee = T_ee(1:3,4);

            cla(ax);
            plotRobot(ax, p_all, T_all);
            title(ax, 'Robot Visualization');

            txt = {};
            txt{end+1,1} = '1. FORWARD KINEMATICS';
            txt{end+1,1} = ' ';

            if isPlanar3R(robot)
                a1 = robot.a(1);
                a2 = robot.a(2);
                a3 = robot.a(3);

                txt{end+1,1} = 'Compact FK for 3R planar arm:';
                txt{end+1,1} = sprintf('p_x = %.4g*cos(q1) + %.4g*cos(q1+q2) + %.4g*cos(q1+q2+q3)', a1, a2, a3);
                txt{end+1,1} = sprintf('p_y = %.4g*sin(q1) + %.4g*sin(q1+q2) + %.4g*sin(q1+q2+q3)', a1, a2, a3);
                txt{end+1,1} = 'phi = q1 + q2 + q3';
                txt{end+1,1} = ' ';
            end

            if isfield(fkInfo, 'available') && fkInfo.available
                txt{end+1,1} = 'Symbolic forward-kinematics expression T(q):';
                txt = [txt; symbolicMatrixToText(fkInfo.T_ee, 4)];
                txt{end+1,1} = ' ';

                txt{end+1,1} = 'Symbolic end-effector position p(q):';
                txt = [txt; symbolicVectorToText(fkInfo.p_ee, 4)];
                txt{end+1,1} = ' ';

                txt{end+1,1} = 'Substitute q =';
                txt = [txt; vectorToText(q)];
                txt{end+1,1} = ' ';
            else
                txt{end+1,1} = 'Symbolic forward-kinematics expression unavailable.';
                if isfield(fkInfo, 'note') && ~isempty(fkInfo.note)
                    txt{end+1,1} = fkInfo.note;
                end
                txt{end+1,1} = ' ';
            end

            txt{end+1,1} = 'Substituted numerical transform T(q):';
            txt = [txt; matrixToText(T_ee)];
            txt{end+1,1} = ' ';

            txt{end+1,1} = 'Final answer: end-effector position p_e';
            txt = [txt; vectorToText(p_ee)];

            outArea.Value = txt;
        catch ME
            outArea.Value = {['Error: ' ME.message]};
        end
    end

    function onAnimate()
        try
            robot = ensureRobot();
            qf = parseVector(qField, robot.n, 'q');
            q0 = robot.q_home(:);
            steps = 60;

            for k = 1:steps
                s = (k-1)/(steps-1);
                qk = (1-s)*q0 + s*qf;
                [T_all, p_all] = fkineDH(robot, qk);
                cla(ax);
                plotRobot(ax, p_all, T_all);
                title(ax, sprintf('Animation %.0f%%', 100*s));
                drawnow;
                pause(0.03);
            end

            T_final = T_all(:,:,end);

            txt = {};
            txt{end+1,1} = '1a. ANIMATE HOME -> q';
            txt{end+1,1} = 'Animation completed.';
            txt{end+1,1} = ' ';
            txt{end+1,1} = 'Final end-effector transform T:';
            txt = [txt; matrixToText(T_final)];
            txt{end+1,1} = ' ';
            txt{end+1,1} = 'Final position p_e:';
            txt = [txt; vectorToText(T_final(1:3,4))];

            outArea.Value = txt;
        catch ME
            outArea.Value = {['Error: ' ME.message]};
        end
    end

    function onInverseKinematics()
        try
            robot = ensureRobot();
            q0 = parseVector(qField, robot.n, 'q');

            txt = {};
            txt{end+1,1} = '2. INVERSE KINEMATICS';
            txt{end+1,1} = ' ';

            if robot.n < 6
                pd = parse3(pdField, 'pd');

                opts.maxIter = 250;
                opts.tol = 1e-6;
                opts.alpha = 0.5;
                opts.method = 'dls';
                opts.damping = 1e-2;
                opts.useLimits = true;

                [q_sol, success, history, ikInfo] = inverseKinematicsNumerical(robot, pd, q0, opts);

                if isPlanar3R(robot)
                    a1 = robot.a(1);
                    a2 = robot.a(2);
                    a3 = robot.a(3);

                    txt{end+1,1} = 'Compact FK position model used in IK:';
                    txt{end+1,1} = sprintf('p_x(q) = %.4g*cos(q1) + %.4g*cos(q1+q2) + %.4g*cos(q1+q2+q3)', a1, a2, a3);
                    txt{end+1,1} = sprintf('p_y(q) = %.4g*sin(q1) + %.4g*sin(q1+q2) + %.4g*sin(q1+q2+q3)', a1, a2, a3);
                    txt{end+1,1} = ' ';
                end

                txt{end+1,1} = 'Symbolic forward-kinematics position model p(q):';
                if isfield(ikInfo, 'symbolic_fk') && isfield(ikInfo.symbolic_fk, 'available') && ikInfo.symbolic_fk.available
                    txt = [txt; symbolicVectorToText(ikInfo.symbolic_fk.p_ee, 4)];
                else
                    txt{end+1,1} = 'Unavailable.';
                end
                txt{end+1,1} = ' ';

                txt{end+1,1} = 'Target position p_d:';
                txt = [txt; vectorToText(pd)];
                txt{end+1,1} = ' ';

                txt{end+1,1} = 'Substitute q = q_sol into p(q):';
            else
                pd = parse3(pdField, 'pd');
                phi_d = parse3(phiField, 'phi_d');
                xd = [pd; phi_d];

                opts.maxIter = 250;
                opts.tol = 1e-6;
                opts.dt = 0.05;
                opts.method = 'dls';
                opts.lambda = 1e-2;
                opts.K = diag([2 2 2 2 2 2]);
                opts.xdot_d = zeros(6,1);
                opts.useLimits = true;

                [q_sol, success, history, ikInfo] = inverseKinematicsAnalytical(robot, xd, q0, opts);

                txt{end+1,1} = 'Symbolic forward-kinematics transform T(q):';
                if isfield(ikInfo, 'symbolic_fk') && isfield(ikInfo.symbolic_fk, 'available') && ikInfo.symbolic_fk.available
                    txt = [txt; symbolicMatrixToText(ikInfo.symbolic_fk.T_ee, 4)];
                    txt{end+1,1} = ' ';
                    txt{end+1,1} = 'Symbolic forward-kinematics position p(q):';
                    txt = [txt; symbolicVectorToText(ikInfo.symbolic_fk.p_ee, 4)];
                else
                    txt{end+1,1} = 'Unavailable.';
                end
                txt{end+1,1} = ' ';

                txt{end+1,1} = 'Target x_d = [p_d; phi_d]:';
                txt = [txt; vectorToText(xd)];
                txt{end+1,1} = ' ';

                txt{end+1,1} = 'Substitute q = q_sol into T(q) and p(q):';
            end

            qField.Value = mat2str(q_sol.', 6);

            [T_all, p_all] = fkineDH(robot, q_sol);
            T_sol = T_all(:,:,end);
            p_sol = T_sol(1:3,4);

            cla(ax);
            plotRobot(ax, p_all, T_all);
            title(ax, 'Robot Visualization');

            txt = [txt; vectorToText(q_sol)];
            txt{end+1,1} = ' ';

            txt{end+1,1} = 'Substituted numerical transform T(q_sol):';
            txt = [txt; matrixToText(T_sol)];
            txt{end+1,1} = ' ';

            txt{end+1,1} = 'Substituted numerical position p(q_sol):';
            txt = [txt; vectorToText(p_sol)];
            txt{end+1,1} = ' ';

            if success
                txt{end+1,1} = 'Final answer: success = 1 (converged)';
            else
                txt{end+1,1} = 'Final answer: success = 0 (did not converge within max iterations / tolerance)';
            end
            txt{end+1,1} = ' ';
            txt{end+1,1} = 'Final answer: q solution';
            txt = [txt; vectorToText(q_sol)];
            txt{end+1,1} = ' ';
            txt{end+1,1} = sprintf('Final error norm = %.6e', history.errNorm(end));

            outArea.Value = txt;
        catch ME
            outArea.Value = {['Error: ' ME.message]};
        end
    end

    function onDifferentialKinematics()
        try
            robot = ensureRobot();
            q = parseVector(qField, robot.n, 'q');
            qdot = parseVector(qdotField, robot.n, 'qdot');

            [ve, pdot, omega] = velocityKinematics(robot, q, qdot);

            txt = {};
            txt{end+1,1} = '3. DIFFERENTIAL KINEMATICS';
            txt{end+1,1} = ' ';
            txt{end+1,1} = 'Twist v_e = [pdot; omega]:';
            txt = [txt; vectorToText(ve)];
            txt{end+1,1} = ' ';
            txt{end+1,1} = 'Linear velocity pdot:';
            txt = [txt; vectorToText(pdot)];
            txt{end+1,1} = ' ';
            txt{end+1,1} = 'Angular velocity omega:';
            txt = [txt; vectorToText(omega)];

            txt{end+1,1} = ' ';
            txt{end+1,1} = 'Geometric Jacobian J:';
            J = geometricJacobian(robot, q);
            txt = [txt; matrixToText(J)];

            txt{end+1,1} = ' ';
            txt{end+1,1} = 'Analytical Jacobian reference:';
            try
                [JA, xe, ~, ~, ~] = analyticalJacobianZYZ(robot, q);
                txt{end+1,1} = 'Operational variable x_e = [p_e; phi_e]:';
                txt = [txt; vectorToText(xe)];
                txt{end+1,1} = ' ';
                txt{end+1,1} = 'Analytical Jacobian J_A:';
                txt = [txt; matrixToText(JA)];
            catch MEaj
                txt{end+1,1} = ['Analytical Jacobian unavailable: ' MEaj.message];
            end

            outArea.Value = txt;
        catch ME
            outArea.Value = {['Error: ' ME.message]};
        end
    end

    function onInverseVelocityKinematics()
        try
            robot = ensureRobot();
            q = parseVector(qField, robot.n, 'q');
            ve_des = parse6(veField, 've_d');

            [qdot_cmd, J, info] = inverseVelocityKinematics(robot, q, ve_des, 'dls', 1e-2);
            qdotField.Value = mat2str(qdot_cmd.', 6);

            txt = {};
            txt{end+1,1} = '4. INVERSE VELOCITY KINEMATICS';
            txt{end+1,1} = ' ';
            txt{end+1,1} = 'Desired twist ve_d:';
            txt = [txt; vectorToText(ve_des)];
            txt{end+1,1} = ' ';
            txt{end+1,1} = 'qdot command:';
            txt = [txt; vectorToText(qdot_cmd)];
            txt{end+1,1} = ' ';
            txt{end+1,1} = sprintf('rank(J) = %d', info.rank);
            txt{end+1,1} = sprintf('cond approx = %.6g', info.condApprox);
            txt{end+1,1} = ' ';
            txt{end+1,1} = 'Geometric Jacobian J:';
            txt = [txt; matrixToText(J)];
            outArea.Value = txt;
        catch ME
            outArea.Value = {['Error: ' ME.message]};
        end
    end

    function onDynamics()
        try
            robot = ensureRobot();
            q = parseVector(qField, robot.n, 'q');
            qdot = parseVector(qdotField, robot.n, 'qdot');
            qdd = parseVector(qddField, robot.n, 'qdd');
            tauApplied = parseVector(tauField, robot.n, 'tau');

            dyn = symbolicDynamicsLagrange(robot);

            subsVars = [dyn.q; dyn.qd; dyn.qdd];
            subsVals = [q; qdot; qdd];

            B_num = double(subs(dyn.B, subsVars, subsVals));
            C_num = double(subs(dyn.C, subsVars, subsVals));
            g_num = double(subs(dyn.g, subsVars, subsVals));
            tau_num = double(subs(dyn.tau, subsVars, subsVals));

            qdd_fd = B_num \ (tauApplied - C_num*qdot - g_num);

            txt = {};
            txt{end+1,1} = '5. DYNAMICS';
            txt{end+1,1} = ' ';
            txt{end+1,1} = 'Symbolic model (reduced case):';
            txt{end+1,1} = 'B(q) qdd + C(q,qdot) qdot + g(q) = tau';
            txt{end+1,1} = ' ';
            txt{end+1,1} = '----------------------------------------';
            txt{end+1,1} = ' ';

            txt{end+1,1} = 'Symbolic joint equations of motion:';
            txt{end+1,1} = ' ';

            for i = 1:robot.n
                eqStr = char(vpa(dyn.eqns(i), 6));
                fullEq = sprintf('%s = %s', dyn.labels{i}, eqStr);
                wrappedEq = wrapEquationText(fullEq, 90);
                txt = [txt; wrappedEq]; %#ok<AGROW>
                txt{end+1,1} = ' ';
            end

            txt{end+1,1} = '----------------------------------------';
            txt{end+1,1} = ' ';
            txt{end+1,1} = 'Numerical evaluation at current [q, qdot, qdd]:';
            txt{end+1,1} = 'tau(q,qdot,qdd) =';
            txt = [txt; vectorToText(tau_num)];

            txt{end+1,1} = ' ';
            txt{end+1,1} = '----------------------------------------';
            txt{end+1,1} = ' ';
            txt{end+1,1} = 'Mass matrix B(q):';
            txt = [txt; matrixToText(B_num)];

            txt{end+1,1} = ' ';
            txt{end+1,1} = '----------------------------------------';
            txt{end+1,1} = ' ';
            txt{end+1,1} = 'Coriolis matrix C(q,qdot):';
            txt = [txt; matrixToText(C_num)];

            txt{end+1,1} = ' ';
            txt{end+1,1} = '----------------------------------------';
            txt{end+1,1} = ' ';
            txt{end+1,1} = 'Gravity vector g(q):';
            txt = [txt; vectorToText(g_num)];

            txt{end+1,1} = ' ';
            txt{end+1,1} = '----------------------------------------';
            txt{end+1,1} = ' ';
            txt{end+1,1} = 'Forward dynamics from the same model, using applied tau:';
            txt{end+1,1} = 'qdd = B(q)^(-1) [tau - C(q,qdot)qdot - g(q)]';
            txt = [txt; vectorToText(qdd_fd)];

            outArea.Value = txt;
        catch ME
            outArea.Value = {['Error: ' ME.message]};
        end
    end
end

function data = defaultDHTable(n)
    data = cell(n, 8);
    for i = 1:n
        data{i,1} = 'R';
        data{i,2} = 0.3;
        data{i,3} = 0.0;
        data{i,4} = 0.0;
        data{i,5} = 0.0;
        data{i,6} = 0.0;
        data{i,7} = -pi;
        data{i,8} = pi;
    end
end

function txt = matrixToText(M)
    txt = cell(size(M,1), 1);
    for i = 1:size(M,1)
        s = '';
        for j = 1:size(M,2)
            s = [s, sprintf('% .6f   ', M(i,j))]; %#ok<AGROW>
        end
        txt{i,1} = strtrim(s);
    end
end

function txt = vectorToText(v)
    v = v(:);
    txt = cell(numel(v),1);
    for i = 1:numel(v)
        txt{i,1} = sprintf('% .6f', v(i));
    end
end

function val = defaultCOMText(n)
    val = cell(n,1);
    for i = 1:n
        if i == 1
            val{i} = '[0.1 0 0;';
        elseif i == n
            val{i} = ' 0.1 0 0]';
        else
            val{i} = ' 0.1 0 0;';
        end
    end
end

function val = defaultInertiaText(n)
    val = cell(n,1);
    for i = 1:n
        if i == 1
            val{i} = '[0.01 0.01 0.01 0 0 0;';
        elseif i == n
            val{i} = ' 0.01 0.01 0.01 0 0 0]';
        else
            val{i} = ' 0.01 0.01 0.01 0 0 0;';
        end
    end
end

function wrapped = wrapEquationText(str, maxLen)
    if nargin < 2
        maxLen = 90;
    end

    str = char(str);
    wrapped = {};

    while length(str) > maxLen
        breakIdx = [];

        searchStart = max(2, floor(0.6 * maxLen));
        searchEnd = min(length(str), maxLen);

        for k = searchEnd:-1:searchStart
            if str(k) == '+' || str(k) == '-'
                breakIdx = k - 1;
                break;
            end
        end

        if isempty(breakIdx)
            for k = searchEnd:-1:searchStart
                if str(k) == ' '
                    breakIdx = k;
                    break;
                end
            end
        end

        if isempty(breakIdx)
            breakIdx = maxLen;
        end

        wrapped{end+1,1} = strtrim(str(1:breakIdx)); %#ok<AGROW>
        str = strtrim(str(breakIdx+1:end));
    end

    if ~isempty(str)
        wrapped{end+1,1} = strtrim(str);
    end
end

function txt = symbolicMatrixToText(M, digits)
    if nargin < 2
        digits = 4;
    end

    if ~isa(M, 'sym')
        txt = matrixToText(double(M));
        return;
    end

    M = cleanSymbolicExpression(M);
    M = vpa(M, digits);
    txt = cell(size(M,1), 1);

    for i = 1:size(M,1)
        rowParts = cell(1, size(M,2));
        for j = 1:size(M,2)
            rowParts{j} = char(M(i,j));
        end
        txt{i,1} = strjoin(rowParts, '   ');
    end
end

function txt = symbolicVectorToText(v, digits)
    if nargin < 2
        digits = 4;
    end

    v = v(:);

    if ~isa(v, 'sym')
        txt = vectorToText(double(v));
        return;
    end

    v = cleanSymbolicExpression(v);
    v = vpa(v, digits);
    txt = cell(numel(v), 1);

    for i = 1:numel(v)
        txt{i,1} = char(v(i));
    end
end

function exprOut = cleanSymbolicExpression(exprIn)
    if ~isa(exprIn, 'sym')
        exprOut = exprIn;
        return;
    end

    exprOut = simplify(exprIn, 'Steps', 100);
    exprOut = rewrite(exprOut, 'sincos');
    exprOut = simplify(exprOut, 'Steps', 100);
end

function tf = isPlanar3R(robot)
    tf = robot.n == 3 && ...
         all(robot.type == "R") && ...
         all(abs(robot.alpha) < 1e-12) && ...
         all(abs(robot.d) < 1e-12);
end