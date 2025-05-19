clear; close all; clc;

load('mats/Upos.mat'); 

%%
v1_range = linspace(-3, 3, 30);
v2_range = linspace(-3, 3, 30);
v3_range = linspace(-10, 6, 30);

g = 9.81;
eps_max = deg2rad(10);     
T_min = 0;
T_max = 14.5;                     

[v1, v2, v3] = ndgrid(v1_range, v2_range, v3_range);
points = [v1(:), v2(:), v3(:)];

fl1 = (v1.^2 + v2.^2 + (v3 + g).^2);

% Apply constraints
c1 = ( fl1 <= T_max^2) & (fl1 >= T_min^2);
c2 = ((v1.^2 + v2.^2) ./ fl1) <= sin(eps_max)^2;
c3 = ((v1.^2 + v2.^2) ./ ((v3 + g).^2)) <= tan(eps_max)^2;

feasible = c1 & c2 & c3;
feasible_pts = [v1(feasible), v2(feasible), v3(feasible)];

K_constraints = convhull(feasible_pts);

% Compute the UPOS feasible points
upos_is_feasible = all((A_vc * points') <= b_vc, 1);
upos_feasible_points = points(upos_is_feasible, :);

K_upos = convhull(upos_feasible_points);

figure; hold on;

trisurf(K_constraints, feasible_pts(:, 1), feasible_pts(:, 2), feasible_pts(:, 3), ...
    'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'yellow');

% trisurf(K_upos, upos_feasible_points(:, 1), upos_feasible_points(:, 2), upos_feasible_points(:, 3), ...
%     'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'magenta');

xlabel('v_1'); ylabel('v_2'); zlabel('v_3');
% xlim([-2, 2]); ylim([-2, 2]); zlim([-2, 2]);
title('Feasible Region in the Flat Output Space');
axis equal;
grid on; box on;

camlight headlight;
lighting gouraud;
material dull;
view(3);

%%
highest_height = feasible_pts(end, end, end);
first_point = feasible_pts(1, :, :);
lowest_height = first_point(3);

top_points = feasible_pts(feasible_pts(:, 3) == highest_height, :);
top_points_xy = top_points(:, 1:2);

low_points = feasible_pts(feasible_pts(:, 3) == lowest_height, :);
low_points_xy = low_points(:, 1:2);

[center_high, r_high, high_hull_pts, high_hull] = get_cricle_coord(top_points(:, 1:2));
[center_low, r_low, low_hull_pts, low_hull] = get_cricle_coord(low_points(:, 1:2));

%%
theta = linspace(0, 2*pi, 8);
x_circ = center_high(1) + cos(theta) .* r_high;
y_circ = center_high(2) + sin(theta) .* r_high;
high_circle_poly = [x_circ(:), y_circ(:)];

x_circ = center_low(1) + cos(theta) .* r_low;
y_circ = center_low(2) + sin(theta) .* r_low;
low_circle_poly = [x_circ(:), y_circ(:)];

%%
figure; hold on; grid on;
plot(top_points_xy(:, 1), top_points_xy(:, 2), 'b.');
plot(high_hull_pts(:, 1), high_hull_pts(:, 2), 'k--');
plot(high_circle_poly(:, 1), high_circle_poly(:, 2), 'Color', 'r');
plot(center_high(1), center_high(2), 'rx');
title('Top Hull');
xlabel('x'); ylabel('y');

figure; hold on; grid on;
plot(low_points_xy(:, 1), low_points_xy(:, 2), 'b.');
plot(low_hull_pts(:, 1), low_hull_pts(:, 2), 'k--');
plot(low_circle_poly(:, 1), low_circle_poly(:, 2), 'Color', 'r');
plot(center_low(1), center_low(2), 'rx');
title('Low Hull');
xlabel('x'); ylabel('y');

%%

temp1 = [high_circle_poly, highest_height .* ones(length(high_circle_poly), 1)];
temp2 = [low_circle_poly, lowest_height .* ones(length(low_circle_poly), 1)];
circle_vert = [temp1; temp2];

P = Polyhedron('V', circle_vert);
P_upos = Polyhedron('A', A_vc, 'b', b_vc);

figure; hold on;  grid on;
plot(P, 'color', 'yellow', 'alpha', 0.4);
plot(P_upos, 'color', 'cyan', 'alpha', 0.4);
legend('constraints', 'upos')
title('Polyhedron from Vertices');

P.A





