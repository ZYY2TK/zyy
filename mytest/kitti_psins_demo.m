% run_kitti_psins.m
% 使用 KITTI oxts 数据进行 INS/GNSS 松组合导航（适用于 PSINS 新版，如 2024 年后版本）
% 请先运行 psinsinit 初始化工具箱

clear; clc; close all;

%% 1. 设置 KITTI 数据路径（请修改为你的实际路径）
dataPath = 'C:\Users\12091\Desktop\neural_network\devkit_raw_data\2011_09_26_drive_0009_sync\oxts\data\';  % 确保末尾有斜杠

%% 2. 读取所有 oxts 文件
fileList = dir(fullfile(dataPath, '*.txt'));
nFiles = length(fileList);
fprintf('共找到 %d 个 oxts 文件\n', nFiles);

% 预分配数组
lat = zeros(nFiles,1); lon = zeros(nFiles,1); alt = zeros(nFiles,1);
roll = zeros(nFiles,1); pitch = zeros(nFiles,1); yaw = zeros(nFiles,1);
vn = zeros(nFiles,1); ve = zeros(nFiles,1); vu = zeros(nFiles,1);
ax = zeros(nFiles,1); ay = zeros(nFiles,1); az = zeros(nFiles,1);
wx = zeros(nFiles,1); wy = zeros(nFiles,1); wz = zeros(nFiles,1);

% 循环读取每个文件
for i = 1:nFiles
    fid = fopen(fullfile(dataPath, fileList(i).name), 'r');
    data = fscanf(fid, '%f', [30, inf])';
    fclose(fid);
    lat(i)   = data(1);
    lon(i)   = data(2);
    alt(i)   = data(3);
    roll(i)  = data(4);
    pitch(i) = data(5);
    yaw(i)   = data(6);
    vn(i)    = data(7);
    ve(i)    = data(8);
    vu(i)    = data(11);
    ax(i)    = data(12);
    ay(i)    = data(13);
    az(i)    = data(14);
    wx(i)    = data(18);
    wy(i)    = data(19);
    wz(i)    = data(20);
end
fprintf('数据读取完成\n');

%% 3. 转换为 PSINS 格式
dt = 0.1;  % 采样间隔 10 Hz
t = (0:nFiles-1)' * dt;  % 时间轴（相对时间，以第一个文件为 0）

% IMU 数据：角增量 [rad] 和速度增量 [m/s]
imu = [wx, wy, wz, ax, ay, az] * dt;
imu(:,7) = t;  % 最后一列是时间

% GPS 数据：速度 [东, 北, 天] (m/s)，位置 [纬度, 经度, 高度] (rad, rad, m)
gps = [ve, vn, vu, lat*pi/180, lon*pi/180, alt, t];

% 初始姿态、速度、位置（使用第一帧）
att0 = [pitch(1); roll(1); yaw(1)];          % [pitch; roll; yaw] (rad)
vn0  = [ve(1); vn(1); vu(1)];                 % [vE; vN; vU] (m/s)
pos0 = [lat(1)*pi/180; lon(1)*pi/180; alt(1)]; % [lat; lon; hgt] (rad, rad, m)
avp0 = [att0; vn0; pos0];

%% 4. 初始化 SINS
% 注意：PSINS 新版要求 insinit 输入 avp0 和 dt，或 avp0 和 ts（采样间隔）
ins = insinit(avp0, dt);

%% 5. 设置 IMU 误差（若不清楚可设为零，但滤波器仍会估计部分误差）
% 陀螺常值零偏 0.01 deg/h，加表常值零偏 100 ug，角度随机游走 0.001 deg/sqrt(h)，速度随机游走 10 ug/sqrt(Hz)
% 单位转换：PSINS 内部使用 rad, m/s 等，但 imuerrset 会自动处理
imuerr = imuerrset(0.01, 100, 0.001, 10);
% 若不想加入任何误差，可以全设为 0：
% imuerr = imuerrset(0, 0, 0, 0);

%% 6. 初始化 Kalman 滤波器（使用官方预设类型 test_SINS_GPS_153）
% 该类型为 15 状态：姿态误差(3)、速度误差(3)、位置误差(3)、陀螺漂移(3)、加表零偏(3)
% 量测为速度+位置（6维）
kftype = 'test_SINS_GPS_153';
kf = kfinit(ins, imuerr, kftype);

% 若 kfinit 因版本问题无法使用，可采用手动设置（但一般新版都支持）
% 此处保留手动设置的注释备查：
% kf = [];
% kf.Qk = diag([imuerr.web; imuerr.wdb; zeros(9,1)])^2 * dt;
% kf.Rk = diag([0.1;0.1;0.1; 1;1;3])^2;   % 速度噪声 0.1 m/s，位置噪声 1m,1m,3m
% kf.Pxk = diag([0.1*glv.deg;0.1*glv.deg;1*glv.deg; 1;1;1; 10;10;10; 0.01;0.01;0.01; 100;100;100])^2;
% kf.Hk = zeros(6,15);
% kf.Hk(1:3,4:6) = eye(3);
% kf.Hk(4:6,7:9) = eye(3);
% kf = kfinit0(kf, dt);   % 若没有 kfinit0，尝试 kf = kfinit(kf, dt);

fprintf('滤波器初始化完成\n');

%% 7. 主循环：惯导更新 + 滤波更新
avpResult = zeros(nFiles, 10);  % 保存每时刻的 [att; vn; pos; t]
kfResult  = zeros(nFiles, 16);  % 保存前15个状态估计 + 时间（可选）

for i = 1:nFiles
    % 当前时刻的 IMU 数据（角增量+速度增量，6x1列向量）
    wm = imu(i,1:3)';
    vm = imu(i,4:6)';
    imu_sample = [wm; vm];       % 6x1 向量
    t_curr = imu(i,7);
    
    % 惯导更新（新版 insupdate 只接受一个 imu 向量）
    ins = insupdate(ins, imu_sample);
    
    % 量测更新（每步都有 GPS 数据，因为频率相同）
    z = [ins.vn - gps(i,1:3)';   % 速度差 [vE; vN; vU]
         ins.pos - gps(i,4:6)']; % 位置差 [dlat; dlon; dhgt] (rad, rad, m)
    kf = kfupdate(kf, z);
    
    % 将滤波器估计的误差反馈到 ins 中
    ins = kffeedback(ins, kf);
    
    % 保存结果
    avpResult(i,:) = [ins.att; ins.vn; ins.pos; t_curr]';
    kfResult(i,:) = [kf.xk(1:15); t_curr]';   % 只保存状态估计的前15个
end

fprintf('组合导航计算完成\n');

%% 8. 绘图
% 8.1 绘制导航结果（姿态、速度、位置）
figure('Name', 'SINS/GPS 导航结果');
insplot(avpResult);

% 8.2 绘制滤波器状态估计
figure('Name', 'Kalman 滤波状态估计');
kfplot(kfResult);

% 8.3 与 KITTI 提供的参考姿态对比（可选）
ref_att = [pitch, roll, yaw];   % 参考姿态（弧度）
att_err = (avpResult(:,1:3) - ref_att) * glv.deg;  % 转换为度
figure('Name', '姿态误差');
subplot(311); plot(t, att_err(:,1)); ylabel('pitch error (deg)'); grid on;
subplot(312); plot(t, att_err(:,2)); ylabel('roll error (deg)'); grid on;
subplot(313); plot(t, att_err(:,3)); ylabel('yaw error (deg)'); grid on;
xlabel('时间 (s)');

% 8.4 绘制轨迹（经纬度）
figure('Name', '轨迹');
plot(avpResult(:,8), avpResult(:,7));  % 经度 vs 纬度
xlabel('经度 (rad)'); ylabel('纬度 (rad)'); grid on;
axis equal;  % 保持比例
title('组合导航轨迹');