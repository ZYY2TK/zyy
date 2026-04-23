% generate_psins_files.m
% 从 KITTI OXTS 原始数据生成 PSINS 兼容的 imu_data.txt, gps_data.txt, init.txt
% 坐标系转换：从 KITTI (前-左-上) 到 PSINS 默认 (前-右-下)

clear; clc; close all;

%% ==================== 用户设置 ====================
data_dir = 'C:\Users\12091\Desktop\neural_network\devkit_raw_data/';
sequence = '2011_09_26_drive_0009_sync';
% ===================================================

seq_path = fullfile(data_dir, sequence);
oxts_path = fullfile(seq_path, 'oxts');
data_path = fullfile(oxts_path, 'data');

%% 1. 读取所有 OXTS 数据文件（每帧一个 .txt，每行 30 个数字）
fprintf('正在读取 OXTS 数据文件...\n');
file_list = dir(fullfile(data_path, '*.txt'));
n = length(file_list);
if n == 0
    error('未找到数据文件，请检查路径：%s', data_path);
end

% 按文件名数字顺序排序
names = {file_list.name};
num_str = regexp(names, '\d+', 'match');
num_val = zeros(n,1);
for i = 1:n
    num_val(i) = str2double(num_str{i}{1});
end
[~, sort_idx] = sort(num_val);
file_list = file_list(sort_idx);

% 预分配存储矩阵（n 行，30 列）
data_all = zeros(n, 30);

for i = 1:n
    fid = fopen(fullfile(data_path, file_list(i).name), 'r');
    data = fscanf(fid, '%f');
    fclose(fid);
    if length(data) ~= 30
        warning('文件 %s 包含 %d 个数值（应为 30），跳过该帧', file_list(i).name, length(data));
        continue;
    end
    data_all(i, :) = data(:)';
end

% 如果某些帧读取失败，去除全零行
valid_rows = any(data_all, 2);
if ~all(valid_rows)
    fprintf('有 %d 帧数据不完整，已剔除。\n', sum(~valid_rows));
    data_all = data_all(valid_rows, :);
    n = size(data_all,1);
end

%% 2. 读取时间戳文件（用于生成时间列，可选）
ts_file = fullfile(oxts_path, 'timestamps.txt');
if exist(ts_file, 'file')
    fid = fopen(ts_file);
    ts_cell = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    timestamps_str = ts_cell{1};
    % 只取有效帧对应的时间戳（可能 data 文件数量少于时间戳行数）
    if length(timestamps_str) > n
        timestamps_str = timestamps_str(1:n);
    elseif length(timestamps_str) < n
        error('时间戳文件行数少于数据文件数');
    end
    t = zeros(n,1);
    for i = 1:n
        t_str = timestamps_str{i};
        t_parts = sscanf(t_str, '%d-%d-%d %d:%d:%f');
        t(i) = t_parts(4)*3600 + t_parts(5)*60 + t_parts(6); % 秒
    end
    t = t - t(1);   % 相对时间（秒）
else
    warning('timestamps.txt 未找到，使用近似时间（帧索引 * 0.1）');
    t = (0:n-1)' * 0.1;
end

%% 3. 提取各字段（按 readme 中的顺序）
% 字段索引（从1开始）：
%  1: lat,  2: lon,  3: alt,  4: roll,  5: pitch,  6: yaw
%  7: vn,   8: ve,   9: vf,  10: vl,   11: vu
% 12: ax,  13: ay,  14: az,  15: af,  16: al,  17: au
% 18: wx,  19: wy,  20: wz,  21: wf,  22: wl,  23: wu
% 24: posacc, 25: velacc, 26: navstat, 27: numsats, 28: posmode, 29: velmode, 30: orimode

lat = data_all(:,1);
lon = data_all(:,2);
alt = data_all(:,3);
roll = data_all(:,4);    % 原始 roll  (左抬为正)
pitch = data_all(:,5);   % 原始 pitch (头向下为正)
yaw = data_all(:,6);     % 原始 yaw   (0 指东，逆时针)
vn = data_all(:,7);      % 北速
ve = data_all(:,8);      % 东速
vu = data_all(:,11);     % 天速（注意索引11）
wx = data_all(:,18);     % 角速度 x (前向轴)
wy = data_all(:,19);     % 角速度 y (左向轴)
wz = data_all(:,20);     % 角速度 z (天向轴)
ax = data_all(:,12);     % 加速度 x (前向)
ay = data_all(:,13);     % 加速度 y (左向)
az = data_all(:,14);     % 加速度 z (天向)

%% 4. 坐标系转换：从 KITTI (前-左-上) 到 PSINS (前-右-下)
% 角速度：y 取反，z 取反
wx_psins = wx;
wy_psins = -wy;
wz_psins = -wz;

% 加速度：y 取反，z 取反
ax_psins = ax;
ay_psins = -ay;
az_psins = -az;

% 姿态角：roll 取反，pitch 取反，yaw 转换
roll_psins = -roll;
pitch_psins = -pitch;
yaw_psins = pi/2 - yaw;          % 转换为北偏东顺时针，结果范围 [-π, π]

% 可选：将 yaw 调整到 [0, 2π) 或 [-π, π]（PSINS 通常接受任意范围）
yaw_psins = mod(yaw_psins + pi, 2*pi) - pi;  % 归一化到 [-π, π)

% 初始值（第一帧）
init = [lat(1), lon(1), alt(1), roll_psins(1), pitch_psins(1), yaw_psins(1)];

%% 5. 保存为 PSINS 所需文件
% IMU 数据：每行 [t, wx, wy, wz, ax, ay, az]
imu_out = [t, wx_psins, wy_psins, wz_psins, ax_psins, ay_psins, az_psins];
save('imu_data.txt', 'imu_out', '-ascii');

% GPS 数据：每行 [t, lat, lon, alt, vn, ve, vu] （速度已是北-东-天）
gps_out = [t, lat, lon, alt, vn, ve, vu];
save('gps_data.txt', 'gps_out', '-ascii');

% 初始值
save('init.txt', 'init', '-ascii');

fprintf('成功生成 imu_data.txt, gps_data.txt, init.txt\n');