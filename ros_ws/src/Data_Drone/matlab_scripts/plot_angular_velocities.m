function fig = plot_angular_velocities(db, unit)
    if nargin < 2
        unit = 'rad';
    end

    ang_vel = db.state(:, 11:13);

    if unit == 'deg'
        ang_vel = rad2deg(ang_vel);
    end

    fig = figure;
    subplot(3, 1, 1); hold on; grid on
    plot(db.t, ang_vel(:, 1))
    legend('sim', 'Location', 'best')
    xlabel('Time (s)')
    ylabel(sprintf('w_x (%s/s)', unit))
    xlim([0 db.t(end)])
    title("Roll Rate")

    subplot(3, 1, 2); hold on; grid on
    plot(db.t, ang_vel(:, 2))
    legend('sim', 'Location', 'best')
    xlabel('Time (s)')
    ylabel(sprintf('w_y (%s/s)', unit))
    xlim([0 db.t(end)])
    title("Pitch Rate")

    subplot(3, 1, 3); hold on; grid on
    plot(db.t, ang_vel(:, 3))
    legend('sim', 'Location', 'best')
    xlabel('Time (s)')
    ylabel(sprintf('w_z (%s/s)', unit))
    xlim([0 db.t(end)])
    title("Yaw Rate")

    sgtitle(sprintf('%s Angular Velocities (%s/s)', db.name, unit));
end