function J = geometricJacobian(robot, q)

    q = q(:);
    n = robot.n;

    [T_all, ~] = fkineDH(robot, q);
    T_ee = T_all(:,:,end);
    p_e = T_ee(1:3,4);

    Jv = zeros(3,n);
    Jw = zeros(3,n);

    for i = 1:n
        T_i_minus_1 = T_all(:,:,i);
        z = T_i_minus_1(1:3,3);
        p = T_i_minus_1(1:3,4);

        if robot.type(i) == "R"
            Jv(:,i) = cross(z, p_e - p);
            Jw(:,i) = z;
        else
            Jv(:,i) = z;
            Jw(:,i) = [0;0;0];
        end
    end

    J = [Jv; Jw];
end