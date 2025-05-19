% Load the computation time data
sim_o = 505202504;
sim_p = 505202505;
file_name = sprintf('mats/computation_times_%d_%d.mat', sim_o, sim_p);
load(file_name);

start_idx = 1;
seq_times = seq(start_idx : end);
par_times = par(start_idx : end);


avg_seq = mean(seq_times);
avg_par = mean(par_times);

timesteps = 1:length(par_times);

figure;
hold on;
stem(timesteps(start_idx:end), seq_times(start_idx:end), 'filled', 'DisplayName', 'Sequential');
stem(timesteps(start_idx:end), par_times(start_idx:end), 'filled', 'DisplayName', 'Parallel');

yline(avg_seq, '--r', sprintf('Seq Avg = %.4f s', avg_seq), 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle');
yline(avg_par, '--g', sprintf('Par Avg = %.4f s', avg_par), 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle');

xlabel('Timestep');
ylabel('Computation Time (s)');
title('MPC Solve Time per Step');
legend;
grid on;
hold off;
