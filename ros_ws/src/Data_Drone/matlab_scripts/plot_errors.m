function [roll_pitch_errs, fig] = plot_errors(db, err, signal, unit)
    if nargin < 3
        signal = 'pos';
    end
    if nargin < 4
        unit = 'rad';
    end

    switch(signal)
        case 'pos'
            roll_pitch_errs = -1;
            fig = figure;
            subplot(3, 1, 1); hold on; grid on
            plot(db.t, err(:, 1))
            legend('sim', 'Location', 'best')
            xlabel('Time (s)')
            ylabel('Error X (m)')
            xlim([0 db.t(end)])
            title("Position Error X")
        
            subplot(3, 1, 2); hold on; grid on
            plot(db.t, err(:, 2))
            legend('sim', 'Location', 'best')
            xlabel('Time (s)')
            ylabel('Error Y (m)')
            xlim([0 db.t(end)])
            title("Position Error Y")
        
            subplot(3, 1, 3); hold on; grid on
            plot(db.t, err(:, 3))
            legend('sim', 'Location', 'best')
            xlabel('Time (s)')
            ylabel('Error Z (m)')
            xlim([0 db.t(end)])
            title("Position Error Z")
        
            sgtitle(sprintf('%s Position Error', db.name));
        case 'vel'
            roll_pitch_errs = -1;

            fig = figure;
            subplot(3, 1, 1); hold on; grid on
            plot(db.t, err(:, 4))
            legend('sim', 'Location', 'best')
            xlabel('Time (s)')
            ylabel('Error vx (m/s)')
            xlim([0 db.t(end)])
            title("Velocity Error X")
        
            subplot(3, 1, 2); hold on; grid on
            plot(db.t, err(:, 5))
            legend('sim', 'Location', 'best')
            xlabel('Time (s)')
            ylabel('Error vy (m/s)')
            xlim([0 db.t(end)])
            title("Velocity Error Y")
        
            subplot(3, 1, 3); hold on; grid on
            plot(db.t, err(:, 6))
            legend('sim', 'Location', 'best')
            xlabel('Time (s)')
            ylabel('Error vz (m/s)')
            xlim([0 db.t(end)])
            title("Velocity Error Z")
        
            sgtitle(sprintf('%s Velocity Error', db.name));
       case 'ang'
            
            errs = db.eu_ang(:, 1:2) - db.controls(:, 2:3);
            roll_pitch_errs = errs;

            if unit == 'deg'
                errs = rad2deg(errs);
            end

            fig = figure;
            subplot(2, 1, 1); hold on; grid on
            plot(db.t, errs(:, 1))
            legend('sim', 'Location', 'best')
            xlabel('Time (s)')
            ylabel(sprintf('Error Roll (%s)', unit))
            xlim([0 db.t(end)])
            title("Roll Error")
        
            subplot(2, 1, 2); hold on; grid on
            plot(db.t, errs(:, 2))
            legend('sim', 'Location', 'best')
            xlabel('Time (s)')
            ylabel(sprintf('Error Pitch (%s)', unit))
            xlim([0 db.t(end)])
            title("Pitch Error")
        
            sgtitle(sprintf('%s Angle Errors (%s)', db.name, unit));
    end 
end