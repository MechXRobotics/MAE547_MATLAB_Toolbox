function C = coriolisMatrix(robot, q, qdot)
    q = q(:);
    qdot = qdot(:);
    n = robot.n;

    if numel(q) ~= n || numel(qdot) ~= n
        error('q and qdot must have n elements.');
    end

    h = 1e-6;
    dB = zeros(n,n,n); 

    for k = 1:n
        dq = zeros(n,1);
        dq(k) = h;

        B_plus = massMatrix(robot, q + dq);
        B_minus = massMatrix(robot, q - dq);

        dB(:,:,k) = (B_plus - B_minus) / (2*h);
    end

    C = zeros(n,n);

    for i = 1:n
        for j = 1:n
            cij = 0;
            for k = 1:n
                c_ijk = 0.5 * (dB(i,j,k) + dB(i,k,j) - dB(j,k,i));
                cij = cij + c_ijk * qdot(k);
            end
            C(i,j) = cij;
        end
    end
end