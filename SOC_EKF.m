clc; clear; close all;

% 设定数据集路径列表
data_paths = {
    "E:\BMS\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\25degC\Drive cycles\03-18-17_02.17 25degC_Cycle_1_Pan18650PF.mat",
    "E:\BMS\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\25degC\Drive cycles\03-19-17_03.25 25degC_Cycle_2_Pan18650PF.mat",
    "E:\BMS\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\25degC\Drive cycles\03-19-17_09.07 25degC_Cycle_3_Pan18650PF.mat",
    "E:\BMS\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\25degC\Drive cycles\03-19-17_14.31 25degC_Cycle_4_Pan18650PF.mat",
    "E:\BMS\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\25degC\Drive cycles\03-20-17_01.43 25degC_US06_Pan18650PF.mat",
    "E:\BMS\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\25degC\Drive cycles\03-20-17_05.56 25degC_HWFTa_Pan18650PF.mat",
    "E:\BMS\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\25degC\Drive cycles\03-20-17_19.27 25degC_HWFTb_Pan18650PF.mat",
    "E:\BMS\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\25degC\Drive cycles\03-21-17_00.29 25degC_UDDS_Pan18650PF.mat",
    "E:\BMS\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\25degC\Drive cycles\03-21-17_09.38 25degC_LA92_Pan18650PF.mat",
    "E:\BMS\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\Panasonic 18650PF Data\25degC\Drive cycles\03-21-17_16.27 25degC_NN_Pan18650PF.mat"
};

% 初始化电池参数
global R0 R1 C1 Eta C_N;
R0 = 0.0096;   
R1 = 0.0071;   
C1 = 50;       
Eta = 0.97;    
C_N = 2.9 * 3600; % 电池额定容量 (2.9Ah 转换为库仑)

% EKF 误差协方差矩阵
global P Q R Wk Vk;
P = [1e-8 0; 0 1e-6]; 
Q = 0.01;    
R = 100;    
Wk = [0.01; 0.01];  
Vk = [100; 100];

% 循环处理数据集
for file_idx = 1:length(data_paths)
    file_path = data_paths{file_idx};
    
    % 加载数据
    disp(['正在处理文件: ', file_path]);
    data = load(file_path);
    
    % 获取 `meas` 结构体
    meas = data.meas;
    
    % 提取时间、电压、电流字段
    battery_time = meas.Time;
    battery_vol = meas.Voltage;
    battery_cur = meas.Current;

    len = length(battery_time);

    % 初始化 EKF 变量
    global Uk SOCk Ik;
    Uk = battery_vol(1);
    SOCk = 1.0; % 初始SOC 设为100%
    Ik = battery_cur(1); 
    
    % 结果存储
    SOC_est = nan(len, 1);
    U_est = nan(len, 1);
    Uoc_est = nan(len, 1);
    
    % EKF 计算SOC
    for i = 2:len
        dt = battery_time(i) - battery_time(i-1);
        if dt < 0.001
            dt = 0.001;
        end
        
        Uk_obs = battery_vol(i);
        Ik_obs = battery_cur(i);
        SOCk_obs = SOCk;
        
        [Uk, Ik, SOCk] = soc_estimator_ekf(Uk_obs, Ik_obs, SOCk_obs, dt);
        
        % 计算开路电压
        Uoc_est(i) = Uk + polyval([1.6026 -8.2896 14.8878 -12.0873 5.5781 2.5047], SOCk) + Ik * R0;
        SOC_est(i) = SOCk;
        U_est(i) = Uk;
    end
    
    % 生成文件名
    [~, filename, ~] = fileparts(file_path); % 获取不带路径的文件名
    result_filename = strcat('SOC_估算结果_', filename, '.csv');
    
    % 保存结果
    result_table = table(battery_time, battery_vol, battery_cur, SOC_est, Uoc_est, U_est, ...
        'VariableNames', {'Time_s', 'Voltage_V', 'Current_A', 'SOC_EKF', 'Uoc_EKF', 'Polarization_V'});
    writetable(result_table, result_filename);
    
    disp(['SOC 估算结果已保存至: ', result_filename]);
end

disp('所有数据集处理完成！');

% ============ EKF 估算函数 ============ 
function [Uk_est, Ik_est, SOCk_est] = soc_estimator_ekf(Uk_obs, Ik_obs, SOC_obs, dt)
    global R0 R1 C1 Eta C_N;
    global P Q R Wk Vk;
    global Uk Ik SOCk;
    
    % **状态预测**
    Uk_ = exp(-dt/(R1*C1)) * Uk + R1 * (1 - exp(-dt/(R1*C1))) * Ik;
    SOCk_ = SOCk + (Eta*dt/C_N) * Ik;
    Xk_ = [Uk_; SOCk_];
    Ak = [exp(-dt/(R1*C1)), 0; 0, 1];
    
    % **误差协方差预测**
    P_ = Ak * P * Ak' + Wk * Q * Wk';
    
    % **观测方程**
    Uk_pola = Uk_obs - R0 * Ik_obs - polyval([1.6026 -8.2896 14.8878 -12.0873 5.5781 2.5047], SOC_obs);
    Xk_obs = [Uk_pola; SOC_obs];
    Ck = [1, (8.013*SOCk^4) - (33.1584*SOCk^3) + (44.6634*SOCk^2) - (24.1746*SOCk) + 5.5781];
    
    % **卡尔曼增益计算**
    Kk = P_ * Ck' / (Ck * P_ * Ck' + Vk' * R * Vk);
    
    % **状态更新**
    Xk_est = Xk_ + Kk .* (Xk_obs - Xk_);
    P = (eye(2) - Kk * Ck) * P_;
    
    Uk_est = Xk_est(1);
    Ik_est = Ik_obs;
    SOCk_est = Xk_est(2);
end
