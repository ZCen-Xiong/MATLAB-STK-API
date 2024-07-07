function direction_vector = an2vec(azimuth_deg,elevation_deg)
azimuth_rad = deg2rad(azimuth_deg);
elevation_rad = deg2rad(elevation_deg);

% 计算方向矢量
x = cos(elevation_rad) * cos(azimuth_rad);
y = cos(elevation_rad) * sin(azimuth_rad);
z = sin(elevation_rad);

% 输出方向矢量
direction_vector = [x, y, z];
end