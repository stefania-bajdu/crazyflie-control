function fig = plot_angles(db, unit, src)
    if nargin < 2 || isempty(unit)
        unit = 'rad';
    end
    if nargin < 3
        src = 'quat';
    end

    if strcmp(src, 'manual')
        if ~isfield(db, 'eu_ang')
            error('Field "eu_ang" not found in db struct for manual mode.');
        end
        angles = db.eu_ang;
    else
        quats = db.state(:, 7:10); % [qx, qy, qz, qw]
        quats = quats(:, [4 1 2 3]); % rearange to [qw, qx, qy, qz]
        angles = quat2eul(quats);
        angles = angles(:, [3, 2, 1]); % reorder to roll, pitch yaw
    end
    
    controls = db.controls(:, 2:3);

    if unit == 'deg'
        angles = rad2deg(angles);
        controls = rad2deg(controls);
    end

    fig = figure;
    subplot(3, 1, 1); hold on; grid on
    plot(db.t, angles(:, 1))
    plot(db.t, controls(:, 1), '--')
    legend('sim', 'ref', 'Location', 'best')
    xlabel('Time (s)')
    ylabel(sprintf("Roll (%s)", unit))
    xlim([0 db.t(end)])
    title("Roll Angle")

    subplot(3, 1, 2); hold on; grid on
    plot(db.t, angles(:, 2))
    plot(db.t, controls(:, 2), '--')
    legend('sim', 'ref', 'Location', 'best')
    xlabel('Time (s)')
    ylabel(sprintf("Pitch (%s)", unit))
    xlim([0 db.t(end)])
    title("Pitch Angle")

    subplot(3, 1, 3); hold on; grid on
    plot(db.t, angles(:, 3))
    legend('sim', 'Location', 'best')
    xlabel('Time (s)')
    ylabel(sprintf("Yaw (%s)", unit))
    xlim([0 db.t(end)])
    title("Yaw Angle")

    sgtitle(sprintf('%s Angles (%s)', db.name, unit));
end