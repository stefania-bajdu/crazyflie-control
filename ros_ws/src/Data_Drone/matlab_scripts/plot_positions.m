function fig = plot_positions(db)
    fig = figure('Position', [500, 200, 620, 470]);
    subplot(3, 1, 1); hold on; grid on
    plot(db.t, db.state(:, 1))
    plot(db.t, db.ref(:, 1), '--')
    legend('sim', 'ref', 'Location', 'northwest')
    xlabel('Time (s)')
    ylabel('x (m)')
    xlim([0 db.t(end)])
    title("Position X")

    subplot(3, 1, 2); hold on; grid on
    plot(db.t, db.state(:, 2))
    plot(db.t, db.ref(:, 2), '--')
    % legend('sim', 'ref', 'Location', 'best')
    legend('sim', 'ref', 'Location', 'northwest')
    xlabel('Time (s)')
    ylabel('y (m)')
    xlim([0 db.t(end)])
    title("Position Y")

    subplot(3, 1, 3); hold on; grid on
    plot(db.t, db.state(:, 3))
    plot(db.t, db.ref(:, 3), '--')
    % legend('sim', 'ref', 'Location', 'best')
    legend('sim', 'ref', 'Location', 'northwest')
    xlabel('Time (s)')
    ylabel('z (m)')
    xlim([0 db.t(end)])
    title("Position Z")

    sgtitle(sprintf('%s Positions', db.name));
end