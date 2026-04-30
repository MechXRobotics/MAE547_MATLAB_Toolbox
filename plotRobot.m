function plotRobot(ax, p_all, T_all)

    plot3(ax, p_all(:,1), p_all(:,2), p_all(:,3), '-o', ...
        'LineWidth', 2, 'MarkerSize', 6);
    hold(ax, 'on');

    for i = 1:size(T_all,3)
        T = T_all(:,:,i);
        o = T(1:3,4);
        R = T(1:3,1:3);
        s = 0.08;

        quiver3(ax, o(1), o(2), o(3), s*R(1,1), s*R(2,1), s*R(3,1), 0, 'LineWidth', 1.5);
        quiver3(ax, o(1), o(2), o(3), s*R(1,2), s*R(2,2), s*R(3,2), 0, 'LineWidth', 1.5);
        quiver3(ax, o(1), o(2), o(3), s*R(1,3), s*R(2,3), s*R(3,3), 0, 'LineWidth', 1.5);
    end

    hold(ax, 'off');
    grid(ax, 'on');
    axis(ax, 'equal');

    xyz = p_all(:);
    if isempty(xyz) || all(abs(xyz) < 1e-12)
        lim = 1;
    else
        lim = max(0.5, 1.2*max(abs(xyz)));
    end

    xlim(ax, [-lim lim]);
    ylim(ax, [-lim lim]);
    zlim(ax, [-lim lim]);
    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    zlabel(ax, 'Z');
end