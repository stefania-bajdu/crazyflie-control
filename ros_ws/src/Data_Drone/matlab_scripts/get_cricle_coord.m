function [best_center, best_radius, hull_pts, k] = get_cricle_coord(xy_points)

    k = convhull(xy_points(:, 1), xy_points(:, 2));
    hull_pts = xy_points(k, :);

    x_range = linspace(min(hull_pts(:, 1)), max(hull_pts(:, 1)), 300);
    y_range = linspace(min(hull_pts(:, 2)), max(hull_pts(:, 2)), 300);
    [X, Y] = meshgrid(x_range, y_range);
    grid_points = [X(:), Y(:)];
    
    in = inpolygon(grid_points(:, 1), grid_points(:, 2), hull_pts(:, 1), hull_pts(:, 2));
    
    interior_pts = grid_points(in, :);
    min_dists = zeros(size(interior_pts, 1), 1);
    
    for i = 1 : length(interior_pts)
        pt = interior_pts(i, :);
        dists = [];
        for j = 1 : length(hull_pts) - 1
            edge_start = hull_pts(j, :);
            edge_end = hull_pts(j + 1, :);
            d = point_to_segment_dist(pt, edge_start, edge_end);
            dists(end + 1) = d;
        end
        min_dists(i) = min(dists);
    end
    
    [best_radius, idx] = max(min_dists);
    best_center = interior_pts(idx, :);
end