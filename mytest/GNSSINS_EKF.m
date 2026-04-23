% my_kitti_sins_gps.m
% 功能：使用PSINS工具箱对KITTI数据集进行SINS/GNSS组合导航（EKF）
% 数据文件要求：
%   imu_data.txt : 每行 [t, wx, wy, wz, ax, ay, az] (t:s, 陀螺:rad/s, 加表:m/s², 坐标系:前-右-下)
%   init.txt     : 一行 [lat, lon, alt, roll, pitch, yaw] (lat/lon:°, alt:m, 姿态:rad, 航向从北顺时针)
%   gps_data.txt : 每行 [t, lat, lon, alt, vn, ve, vu] (vn:北速, ve:东速, vu:天速, 单位:m/s)

clear; clc; close all;

%% 1. 初始化PSINS工具箱
glvs;   % base 文件夹

%% 2. 尝试定义滤波器类型（15状态 SINS/GPS 松组合）
try
    psinstypedef(15);   % 数字方式设置15状态（适用于较新版本）
catch
    try
        psinstypedef('15');  % 字符串方式
    catch
        % 如果上述都失败，则手动设置 psinsdef 基本字段
        global psinsdef
        psinsdef = [];
        psinsdef.n = 15;          % 状态维数
        psinsdef.m = 3;            % 量测维数（位置）
        psinsdef.kfinit = 1;       % 使用默认初始化
        psinsdef.typestr = '';     % 留空，后续可能需要
        fprintf('手动设置 psinsdef 结构体。\n');
    end
end

%% 3. 加载用户数据
imu = load('C:\Users\12091\Desktop\neural_network\devkit_raw_data\2011_09_26_drive_0009_sync\imu_data.txt');      % 来自用户数据
init = load('C:\Users\12091\Desktop\neural_network\devkit_raw_data\2011_09_26_drive_0009_sync\init.txt');          % 来自用户数据
gps = load('C:\Users\12091\Desktop\neural_network\devkit_raw_data\2011_09_26_drive_0009_sync\gps_data.txt');       % 来自用户数据


% 提取基本信息
ts = mean(diff(imu(:,1)));        % IMU采样间隔
nn = 1;                            % 子样数（单子样更新）

%% 4. 初始状态设置
lat0 = init(1); lon0 = init(2); alt0 = init(3);
roll0 = init(4); pitch0 = init(5); yaw0 = init(6);

% PSINS 的 insinit 要求姿态向量为 [pitch; roll; yaw] (弧度)
att0 = [pitch0; roll0; yaw0];

% 从第一个 GPS 记录获取初始速度（顺序：东速, 北速, 天速）
vE0 = gps(1,6);   % 东速
vN0 = gps(1,5);   % 北速
vU0 = gps(1,7);   % 天速
vn0 = [vE0; vN0; vU0];

% 初始位置向量 [lat; lon; alt] (度,度,米)
pos0 = [lat0; lon0; alt0];

% 初始化 INS 结构体
ins = insinit(att0, vn0, pos0, ts);   % base 文件夹

%% 5. 设置卡尔曼滤波器参数
davp0 = [0.1; 0.1; 0.5] * glv.min;   % 姿态误差：0.1°（转换为弧度）
dvn0 = [0.01; 0.01; 0.01];            % 速度误差：0.01 m/s
dpos0 = [5; 5; 10];                    % 位置误差：5m 水平，10m 垂直

% IMU 误差参数（需根据IMU手册设定，此处为示例）
imuerr = imuerrset(0.01, 100, 0.001, 1);   % base 文件夹

% 量测噪声：GPS位置误差标准差（水平1m，垂直3m）
rk = poserrset([1;1;3]);   % base 文件夹

% 初始化卡尔曼滤波器结构体
try
    kf = kfinit(ins, davp0, dvn0, dpos0, imuerr, rk);   % base 文件夹
catch ME
    % 如果 kfinit 失败，则手动构建 kf 结构体（备用方案）
    fprintf('kfinit 调用失败，采用手动构建 KF 结构体。\n');
    fprintf('错误信息：%s\n', ME.message);
    
    kf = [];
    kf.n = 15;                      % 状态维数
    % 状态顺序：phi(3), dvn(3), dpos(3), eb(3), db(3)
    
    % --- 系统噪声协方差 Qk ---
    Q = zeros(15);
    % 角度随机游走方差（imuerr.web 是向量，单位 rad/s/√Hz）
    web = imuerr.web;
    Q(1:3,1:3) = diag((web * ts).^2);   % 使用 .^2 逐元素平方
    % 速度随机游走方差（imuerr.wdb 是向量，单位 m/s/√Hz）
    wdb = imuerr.wdb;
    Q(4:6,4:6) = diag((wdb * ts).^2);
    % 陀螺零偏和加表零偏视为随机常数，Q中对应位置保持0
    kf.Qk = Q;
    
    % --- 量测噪声协方差 Rk ---
    % rk 是标准差向量（单位米），转换为方差矩阵
    kf.Rk = diag(rk.^2);   % 使用 .^2 逐元素平方
    
    % --- 初始协方差 Pxk ---
    P0 = zeros(15);
    P0(1:3,1:3) = diag(davp0.^2);
    P0(4:6,4:6) = diag(dvn0.^2);
    P0(7:9,7:9) = diag(dpos0.^2);
    P0(10:12,10:12) = diag(imuerr.eb.^2);
    P0(13:15,13:15) = diag(imuerr.db.^2);
    kf.Pxk = P0;
    
    % --- 量测矩阵 Hk（位置量测对应状态7-9）---
    kf.Hk = zeros(3,15);
    kf.Hk(1:3, 7:9) = eye(3);
    
    % 初始化状态向量为零
    kf.xk = zeros(kf.n, 1);
    
    % 调用 kfinit0 完成其他必要字段的初始化，传入滤波周期 nts
    nts = nn * ts;   % 滤波时间更新步长
    kf = kfinit0(kf, nts);
end

%% 6. 主循环：INS更新 + GPS量测更新
len = size(imu, 1);
avp = zeros(fix(len/nn), 10);   % 存储结果 [att; vn; pos; t]

timebar(nn, len, 'SINS/GPS Processing...');   % base 文件夹
ki = 1;
gps_idx = 1;  % GPS数据索引

for k = 1:nn:len-nn+1
    k1 = k+nn-1;
    % 取当前子样的IMU数据（陀螺和加速度计，共6列）
    wvm = imu(k:k1, 2:7);   % 列顺序：wx,wy,wz,ax,ay,az
    t = imu(k1, 1);          % 当前时间
    
    % INS 更新（机械编排）
    ins = insupdate(ins, wvm);   % base 文件夹
    
    % 检查是否有 GPS 观测（时间对齐）
    while gps_idx <= size(gps,1) && abs(gps(gps_idx,1) - t) < ts/2
        % 提取 GPS 位置观测
        posGPS = gps(gps_idx, 2:4)';   % [lat; lon; alt]
        
        % 卡尔曼滤波量测更新
        kf = kfupdate(kf, ins.pos - posGPS, 'M');   % base 文件夹
        
        % 反馈校正
        [kf, ins] = kffeedback(kf, ins, 1, 'avp');   % base 文件夹
        
        gps_idx = gps_idx + 1;
    end
    
    % 保存结果（每秒记录一次）
    if mod(t, 1) < ts/2
        % 使用 ins.att, ins.vn, ins.pos 分别存储，避免与 ins.avp 重复
        avp(ki,:) = [ins.att(:)', ins.vn(:)', ins.pos(:)', t];
        ki = ki + 1;
    end
    
    timebar;   % 更新进度条
end
avp(ki:end,:) = [];  % 删除未使用的预分配行

%% 7. 结果绘图
insplot(avp);   % base 文件夹

disp('处理完成！');

%% 将 PSINS 结果转换为 ENU 并与 OXTS 对比
% 提取 PSINS 位置（纬度、经度、高度）
lat_psins = avp(:,7);
lon_psins = avp(:,8);
alt_psins = avp(:,9);
t_psins = avp(:,10);

% 以第一帧为参考点（应与 OXTS 可视化参考点一致）
lat0 = lat_psins(1);
lon0 = lon_psins(1);
alt0 = alt_psins(1);

% 近似 ENU 转换
R = 6371000;
x_psins = (lon_psins - lon0) * deg2rad(1) * R * cos(deg2rad(lat0));
y_psins = (lat_psins - lat0) * deg2rad(1) * R;
z_psins = alt_psins - alt0;

% 绘制对比图
figure;
plot(x_psins, y_psins, 'b-', 'LineWidth', 1.5); hold on;
% 如果你有 OXTS 的 ENU 坐标（来自 visualize_kitti_oxts_fixed.m 的 x_enu, y_enu），可以叠加
% 假设你已将 OXTS 的 ENU 坐标保存在工作空间，这里演示加载方式
% load('oxts_enu.mat', 'x_enu', 'y_enu'); % 需要先保存
% plot(x_enu, y_enu, 'r--', 'LineWidth', 1.5);
axis equal; grid on;
xlabel('East (m)'); ylabel('North (m)');
legend('PSINS', 'OXTS (参考)');
title('PSINS vs OXTS Trajectory');