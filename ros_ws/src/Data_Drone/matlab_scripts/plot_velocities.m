function fig = plot_velocities(db)
    fig = figure;
    subplot(3, 1, 1); hold on; grid on
    plot(db.t, db.state(:, 4))
    plot(db.t, db.ref(:, 4), '--')
    legend('sim', 'ref')
    xlabel('Time (s)')
    ylabel('vx (m/s)')
    xlim([0 db.t(end)])
    ylims = ylim;
    ymin = min([db.state(:, 4); db.ref(:, 4)]);
    ylim([ymin - 1 * abs(ymin), ylims(2)]);
    title("Velocity X")

    subplot(3, 1, 2); hold on; grid on
    plot(db.t, db.state(:, 5))
    plot(db.t, db.ref(:, 5), '--')
    legend('sim', 'ref')
    xlabel('Time (s)')
    ylabel('vy (m/s)')
    xlim([0 db.t(end)])
    ylims = ylim;
    ymin = min([db.state(:, 4); db.ref(:, 4)]);
    ylim([ymin - 1 * abs(ymin), ylims(2)]);
    title("Velocity Y")

    subplot(3, 1, 3); hold on; grid on
    plot(db.t, db.state(:, 6))
    plot(db.t, db.ref(:, 6), '--')
    legend('sim', 'ref')
    xlabel('Time (s)')
    ylabel('vz (m/s)')
    xlim([0 db.t(end)])
    ylims = ylim;
    ymin = min([db.state(:, 4); db.ref(:, 4)]);
    ylim([ymin - 1 * abs(ymin), ylims(2)]);
    title("Velocity Z")

    sgtitle(sprintf('%s State Velocities', db.name));
end