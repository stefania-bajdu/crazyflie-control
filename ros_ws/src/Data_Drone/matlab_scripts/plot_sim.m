clear; close all; clc;

simid = 6;
file_name = sprintf('mats/sim_%d.mat', simid);
drones = load_drones(file_name);

%%
Nb = length(drones);
errors = cell(Nb, 1);


for i = 1 : Nb
    db = drones{i};

    fig = plot_positions(db); 
    print(fig, "figures/seq_mpc_1_drone_traj_pos_tracking.eps", '-depsc')

    fig = plot_velocities(db);
    print(fig, "figures/seq_mpc_1_drone_traj_vel_tracking.eps", '-depsc')

    fig = plot_angles(db, 'rad', 'manual'); 
    print(fig, "figures/seq_mpc_1_drone_traj_angle_tracking.eps", '-depsc')

    % plot_angular_velocities(db, 'rad');

    fig = plot_controls(db, 'rad');
    print(fig, "figures/seq_mpc_1_drone_traj_controls.eps", '-depsc')

    errors{i}(:, 1:6) = db.state(:, 1:6) - db.ref(:, 1:6);
end

%% Plot errors
pos_err = cell(Nb, 1);
vel_err = cell(Nb, 1);
ang_err = cell(Nb, 1);
for i = 1 : Nb
    plot_errors(drones{i}, errors{i}, 'pos');
    plot_errors(drones{i}, errors{i}, 'vel');
    roll_pitch_errs = plot_errors(drones{i}, errors{i}, 'ang', 'rad');

    pos_err{i} = mean(abs(errors{i}(:, 1:3)), 1);
    vel_err{i} = mean(abs(errors{i}(:, 4:6)), 1);
    ang_err{i} = mean(abs(roll_pitch_errs(:, 1:2)), 1);
end
pos_err
vel_err
ang_err
%%

