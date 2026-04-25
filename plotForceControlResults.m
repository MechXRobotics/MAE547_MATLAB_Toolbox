function plotForceControlResults(results, plotTitle)
% PLOTFORCECONTROLRESULTS
% Plots desired vs actual end-effector position and contact forces.

    t = results.t;

    figure('Name', plotTitle);

    subplot(2,1,1);
    plot(t, results.pd(1,:), '--', 'LineWidth', 1.5);
    hold on;
    plot(t, results.p(1,:), 'LineWidth', 1.5);
    plot(t, results.pd(2,:), '--', 'LineWidth', 1.5);
    plot(t, results.p(2,:), 'LineWidth', 1.5);
    plot(t, results.pd(3,:), '--', 'LineWidth', 1.5);
    plot(t, results.p(3,:), 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Position [m]');
    title([plotTitle ' - Desired vs Actual EE Position']);
    legend('x_d','x','y_d','y','z_d','z');

    subplot(2,1,2);
    plot(t, results.Fenv(1,:), 'LineWidth', 1.5);
    hold on;
    plot(t, results.Fenv(2,:), 'LineWidth', 1.5);
    plot(t, results.Fenv(3,:), 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Contact Force [N]');
    title('End-Effector Contact Forces');
    legend('F_x','F_y','F_z');
end