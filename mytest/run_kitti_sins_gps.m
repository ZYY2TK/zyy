%% run_kitti_sins_pure.m
% 纯惯性解算（无GPS更新），用于验证KITTI数据转换是否正确

clear; clc; close all;

%% 第一步：手动添加PSINS路径（请根据您的安装路径修改）
psins_path = 'D:\zyyUse\导航\psins\psins251010';
addpath(genpath(psins_path));

% 初始化PSINS全局变量（如果之前未初始化）
global psinsdef
psinsdef = [];  % 清空，让psinsinit重新创建
psinsinit;      % 运行初始化

%% 加载转换后的KITTI数据
load('mytest/kitti_oxts_for_psins.mat');  % 变量：imu_data, gps_data, t_seconds, pitch_kitti, roll_kitti, yaw_kitti

%% 设置初始导航参数
% 初始姿态（PSINS的att向量为 [pitch; roll; yaw] 弧度）
att0 = [pitch_kitti(1); roll_kitti(1); yaw_kitti(1)];  % 列向量

% 初始速度（北-东-地）
vn0 = gps_data(1, 5:7)';   % 列向量

% 初始位置（纬度、经度、海拔）
pos0 = gps_data(1, 2:4)';   % 列向量

% 合并为avp0向量 (9x1) [att; vn; pos]
avp0 = [att0; vn0; pos0];

%% 计算IMU采样间隔
ts_imu = diff(t_seconds(1:2));  % 单位：秒（KITTI为0.1s）

%% 设置IMU误差参数（这里先忽略误差，设为0）
imuerr = imuerrset(0, 0, 0, 0);  % 无误差，纯理想情况

%% 初始化SINS结构体
ins = insinit(avp0, ts_imu);

%% 准备存储纯惯性解算结果
avp_pure = zeros(length(t_seconds), 10);  % [att, vn, pos, t]

%% 主循环（纯惯性更新，不加入GPS）
timebar = waitbar(0, '纯惯性解算中...');
for k = 1:length(t_seconds)
    % 当前IMU增量
    wm = imu_data(k, 1:3)';
    vm = imu_data(k, 4:6)';
    
    % 时间间隔
    if k == 1
        dt = ts_imu;
    else
        dt = imu_data(k, 7) - imu_data(k-1, 7);
    end
    
    % 纯惯性更新
    ins = insupdate(ins, wm, vm, dt);
    
    % 保存结果
    avp_pure(k, :) = [ins.att', ins.vn', ins.pos', imu_data(k, 7)];
    
    waitbar(k/length(t_seconds), timebar);
end
close(timebar);

%% 绘图对比
% 绘制姿态、速度、位置曲线
insplot(avp_pure);

% 与GPS原始轨迹对比
figure;
plot3(gps_data(:,2), gps_data(:,3), gps_data(:,4), 'b.', 'DisplayName', 'GPS轨迹');
hold on;
plot3(avp_pure(:,7), avp_pure(:,8), avp_pure(:,9), 'r-', 'DisplayName', '纯惯性解算');
xlabel('纬度 (deg)'); ylabel('经度 (deg)'); zlabel('海拔 (m)');
legend; grid on; axis equal;
title('纯惯性解算 vs GPS轨迹');

disp('纯惯性解算完成！');