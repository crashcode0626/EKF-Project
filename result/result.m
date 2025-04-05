clc; clear; close all;

%% 参数配置
filePath = 'E:\BMS\result\20250405_135053.csv';
nominal_capacity = 2.9;  % 电池标称容量（Ah）
soc_columns = {'SOC_0','SOC_1','SOC_2','SOC_6','SOC_7','SOC_8'};
voltage_columns = {'Voltage_0','Voltage_1','Voltage_2','Voltage_6','Voltage_7','Voltage_8'};

%% 数据读取（保留原始列名）
try
    data = readtable(filePath, 'VariableNamingRule','preserve');
catch
    data = readtable(filePath);
    origNames = data.Properties.VariableDescriptions;
    data.Properties.VariableNames = regexprep(origNames, '^''|''$', '');
end

%% 列名验证
required_columns = [soc_columns, voltage_columns, {'timestamp','Current','current(A)','AH'}];
missing_cols = setdiff(required_columns, data.Properties.VariableNames);
if ~isempty(missing_cols)
    error('缺失必要列: %s', strjoin(missing_cols, ', '));
end

%% 数据预处理
% 时间处理（转换为相对秒）
time_relative = (data.timestamp - data.timestamp(1)) / 1000; 

% 提取关键数据
soc_data = data{:, soc_columns};
voltages = data{:, voltage_columns};
current_bms = data.Current;
current_meas = data.('current(A)');
AH_data = data.AH;

%% 实际SOC计算
if AH_data(end) <= 0
    error('AH数据异常，末行值必须大于0');
end
SOC_actual = 1 - (AH_data / nominal_capacity);

%% ============== SOC分析 ==============
figure('Name','SOC对比分析','Position',[50 50 1400 800])
for i = 1:6
    subplot(2,3,i)
    plot(time_relative, soc_data(:,i), 'b', 'LineWidth',1.5)
    hold on
    plot(time_relative, SOC_actual, 'r--', 'LineWidth',1.5)
    xlabel('时间 (秒)','FontSize',10)
    ylabel('SOC值','FontSize',10)
    title([soc_columns{i} ' vs 实际SOC'],'FontSize',12)
    legend({'BMS计算','实际值'},'FontSize',8)
    grid on
    ylim([0 1.1])
    set(gca,'FontName','Microsoft YaHei')
end

%% ============== 电压分析 ==============
figure('Name','电池电压监测','Position',[50 50 1400 800])
voltage_range = [min(voltages(:))-50, max(voltages(:))+50];
for i = 1:6
    subplot(2,3,i)
    plot(time_relative, voltages(:,i), 'Color','#0072BD', 'LineWidth',1.5)
    xlabel('时间 (秒)','FontSize',10)
    ylabel('电压 (mV)','FontSize',10)
    title(voltage_columns{i},'FontSize',12)
    grid on
    ylim(voltage_range)
    set(gca,'FontName','Microsoft YaHei')
end

%% ============== 电流分析 ==============
figure('Name','电流测量对比','Position',[50 50 1000 800])

% 电流对比图
subplot(3,1,[1 2])
plot(time_relative, -current_bms, 'b', 'LineWidth',1.5)
hold on
plot(time_relative, current_meas, 'r--', 'LineWidth',1.5)
xlabel('时间 (秒)','FontSize',10)
ylabel('电流 (A)','FontSize',10)
title('电流测量值对比','FontSize',12)
legend({'BMS测量','实际测量'},'Location','best')
grid on
set(gca,'FontName','Microsoft YaHei')

% 电流误差图
subplot(3,1,3)
current_error = -current_bms - current_meas;
plot(time_relative, current_error, 'k', 'LineWidth',1)
xlabel('时间 (秒)','FontSize',10)
ylabel('电流误差 (A)','FontSize',10)
title('电流测量误差','FontSize',12)
grid on
ylim([min(current_error)-0.1 max(current_error)+0.1])
set(gca,'FontName','Microsoft YaHei')

%% ============== 误差统计分析 ==============
fprintf('\n================ SOC误差分析 ================\n')
error_absolute = soc_data - SOC_actual;
for i = 1:6
    MAE = mean(abs(error_absolute(:,i))) * 100;
    MaxError = max(abs(error_absolute(:,i))) * 100;
    RMSE = sqrt(mean(error_absolute(:,i).^2)) * 100;
    
    fprintf('电池 %s:\n', soc_columns{i})
    fprintf('  平均绝对误差: %.2f%%\n', MAE)
    fprintf('  最大绝对误差: %.2f%%\n', MaxError)
    fprintf('  均方根误差: %.2f%%\n\n', RMSE)
end

fprintf('\n================ 电流误差分析 ================\n')
[~,max_err_idx] = max(abs(current_error));
fprintf('平均误差: %.4f A\n', mean(current_error))
fprintf('标准差: %.4f A\n', std(current_error))
fprintf('最大绝对误差: %.4f A (发生在%.1f秒)\n',...
    abs(current_error(max_err_idx)), time_relative(max_err_idx))
fprintf('均方根误差: %.4f A\n', sqrt(mean(current_error.^2)))

%% ============== 剩余电量报告 ==============
remaining_ah = nominal_capacity - AH_data(end);
remaining_percent = remaining_ah / nominal_capacity * 100;
fprintf('\n================ 容量报告 ================\n')
fprintf('标称容量: %.2f Ah\n', nominal_capacity)
fprintf('实际放电: %.2f Ah\n', AH_data(end))
fprintf('剩余电量: %.2f Ah (%.1f%%)\n', remaining_ah, remaining_percent)