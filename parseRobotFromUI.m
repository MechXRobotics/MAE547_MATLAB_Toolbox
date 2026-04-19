function robot = parseRobotFromUI(app)
% PARSEROBOTFROMUI
% Parse GUI data into a validated robot struct.
%
% Expected fields in app:
%   app.dhTable
%   app.unitsDrop
%   app.gravField
%   app.massesField
%   app.comField
%   app.inertiaField
%
% Output robot fields:
%   n, type, a, alpha, d, theta, q_home, qlim
%   gravity, mass, com, inertia
%   base, tool

    % ---------------- DH table ----------------
    data = app.dhTable.Data;
    n = size(data, 1);

    robot.n = n;
    robot.type = strings(n,1);
    robot.a = zeros(n,1);
    robot.alpha = zeros(n,1);
    robot.d = zeros(n,1);
    robot.theta = zeros(n,1);
    robot.q_home = zeros(n,1);
    robot.qlim = zeros(n,2);

    for i = 1:n
        jt = upper(strtrim(string(data{i,1})));
        if ~(jt == "R" || jt == "P")
            error('Joint %d type must be R or P.', i);
        end

        robot.type(i)    = jt;
        robot.a(i)       = parseScalarEntry(data{i,2}, sprintf('a(%d)', i));
        robot.alpha(i)   = parseScalarEntry(data{i,3}, sprintf('alpha(%d)', i));
        robot.d(i)       = parseScalarEntry(data{i,4}, sprintf('d(%d)', i));
        robot.theta(i)   = parseScalarEntry(data{i,5}, sprintf('theta(%d)', i));
        robot.q_home(i)  = parseScalarEntry(data{i,6}, sprintf('q_home(%d)', i));
        robot.qlim(i,1)  = parseScalarEntry(data{i,7}, sprintf('qlim_min(%d)', i));
        robot.qlim(i,2)  = parseScalarEntry(data{i,8}, sprintf('qlim_max(%d)', i));

        if robot.qlim(i,1) > robot.qlim(i,2)
            error('For joint %d, qlim_min must be <= qlim_max.', i);
        end
    end

    % ---------------- Units handling ----------------
    useDegrees = strcmpi(app.unitsDrop.Value, 'degrees');

    if useDegrees
        robot.alpha = deg2rad(robot.alpha);

        for i = 1:n
            if robot.type(i) == "R"
                robot.theta(i) = deg2rad(robot.theta(i));
                robot.q_home(i) = deg2rad(robot.q_home(i));
                robot.qlim(i,:) = deg2rad(robot.qlim(i,:));
            end
        end
    end

    % ---------------- Gravity ----------------
    gravityVec = parseNumericArray(app.gravField.Value, 'gravity');
    if numel(gravityVec) ~= 3
        error('Gravity must be a 3-element vector, e.g. [0 0 -9.81].');
    end
    robot.gravity = gravityVec(:);

    % ---------------- Masses ----------------
    massVec = parseNumericArray(app.massesField.Value, 'mass vector');
    if numel(massVec) ~= n
        error('Mass vector must have exactly n = %d elements.', n);
    end
    robot.mass = massVec(:);

    % ---------------- COMs ----------------
    comMat = parseNumericArray(textAreaToString(app.comField.Value), 'COM matrix');
    if ~isequal(size(comMat), [n 3])
        error('COM matrix must be %d x 3.', n);
    end
    robot.com = comMat;

    % ---------------- Inertias ----------------
    inertiaMat = parseNumericArray(textAreaToString(app.inertiaField.Value), 'inertia matrix');
    if ~isequal(size(inertiaMat), [n 6])
        error(['Inertia matrix must be %d x 6 with rows ', ...
               '[Ixx Iyy Izz Ixy Ixz Iyz].'], n);
    end
    robot.inertia = inertiaMat;

    % ---------------- Base / tool ----------------
    robot.base = eye(4);
    robot.tool = eye(4);

    % ---------------- Optional consistency checks ----------------
    validateDynamicData(robot);
end

function x = parseScalarEntry(v, name)
% Parse one scalar from numeric/table/string entry.

    if isnumeric(v)
        if ~isscalar(v) || ~isfinite(v)
            error('Entry %s must be a finite scalar.', name);
        end
        x = double(v);
        return;
    end

    if isstring(v) || ischar(v)
        s = strtrim(char(v));
        tmp = str2num(s); %#ok<ST2NM>
        if isempty(tmp) || ~isscalar(tmp) || ~isfinite(tmp)
            error('Entry %s must be a finite scalar.', name);
        end
        x = double(tmp);
        return;
    end

    error('Unsupported entry type for %s.', name);
end

function A = parseNumericArray(v, name)
% Parse a numeric vector/matrix from edit field or string.

    if isnumeric(v)
        A = double(v);
        if isempty(A) || any(~isfinite(A(:)))
            error('%s must contain finite numeric values.', name);
        end
        return;
    end

    if isstring(v) || ischar(v)
        s = strtrim(char(v));
        A = str2num(s); %#ok<ST2NM>
        if isempty(A) || any(~isfinite(A(:)))
            error('Could not parse %s.', name);
        end
        return;
    end

    error('Unsupported input type for %s.', name);
end

function s = textAreaToString(v)
% Turn uitextarea Value into one parsable MATLAB string.

    if iscell(v)
        parts = strings(numel(v),1);
        for k = 1:numel(v)
            parts(k) = string(v{k});
        end
        s = strjoin(parts, ' ');
    elseif isstring(v)
        s = strjoin(v, ' ');
    else
        s = char(v);
    end
end

function validateDynamicData(robot)
% Basic consistency checks for dynamic data.

    n = robot.n;

    if numel(robot.mass) ~= n
        error('Mass vector size mismatch.');
    end
    if size(robot.com,1) ~= n || size(robot.com,2) ~= 3
        error('COM matrix size mismatch.');
    end
    if size(robot.inertia,1) ~= n || size(robot.inertia,2) ~= 6
        error('Inertia matrix size mismatch.');
    end

    if any(robot.mass < 0)
        error('Mass values must be nonnegative.');
    end

    diagTerms = robot.inertia(:,1:3);
    if any(diagTerms(:) < 0)
        error('Inertia diagonal terms Ixx, Iyy, Izz must be nonnegative.');
    end
end