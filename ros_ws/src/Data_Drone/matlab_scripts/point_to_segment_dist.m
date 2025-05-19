function d = point_to_segment_dist(p0, p1, p2)
    d12 = p2 - p1;
    d = abs(p0(1)*d12(2) - p0(2)*d12(1) + p2(1)*p1(2) - p2(2)*p1(1));
    d = d / sqrt(d12(1)^2 + d12(2)^2);
end