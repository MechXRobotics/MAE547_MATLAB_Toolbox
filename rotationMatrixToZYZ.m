function phi = rotationMatrixToZYZ(R)

    validateattributes(R, {'numeric'}, {'size',[3 3]});

    r13 = R(1,3);
    r23 = R(2,3);
    r31 = R(3,1);
    r32 = R(3,2);
    r33 = R(3,3);

    stheta = sqrt(r13^2 + r23^2);

    % Generic case
    if stheta > 1e-12
        phi1   = atan2(r23, r13);
        theta1 = atan2(stheta, r33);
        psi1   = atan2(r32, -r31);
        phi = [phi1; theta1; psi1];
        return;
    end

    % Singular cases: theta = 0 or pi
    if r33 > 0
        % theta = 0, only phi + psi is determined
        theta = 0;
        phi_plus_psi = atan2(R(2,1), R(1,1));
        phi = [phi_plus_psi; theta; 0];
    else
        % theta = pi, only phi - psi is determined
        theta = pi;
        phi_minus_psi = atan2(R(2,1), R(1,1));
        phi = [phi_minus_psi; theta; 0];
    end
end