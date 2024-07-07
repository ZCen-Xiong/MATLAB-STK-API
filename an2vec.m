function direction_vector = an2vec(azimuth_deg,elevation_deg)
azimuth_rad = deg2rad(azimuth_deg);
elevation_rad = deg2rad(elevation_deg);

% ���㷽��ʸ��
x = cos(elevation_rad) * cos(azimuth_rad);
y = cos(elevation_rad) * sin(azimuth_rad);
z = sin(elevation_rad);

% �������ʸ��
direction_vector = [x, y, z];
end