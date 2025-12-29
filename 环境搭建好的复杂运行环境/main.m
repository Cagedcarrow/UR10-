clear; clc; close all;

% --- 1. 启动主环境 ---
fprintf('正在启动 UR10 仿真系统...\n');

% 初始化控制器 (挂高 1.4m 是为了能够到地面的桶)
ui = Ur10Controller(1.4, 0.25); 

% 启动主界面
ui.launchGUI();
drawnow; % 关键：强制刷新，确保主窗口完全创建后再继续

% --- 2. 启动手动轨迹测试器 (新功能) ---
% 这会弹出一个独立的小窗口
try
    tester = ManualPathTester(ui);
    fprintf('手动测试器已启动。\n');
    fprintf('>>> 请在小窗口中输入坐标，点击 [Move] 测试可达性。\n');
catch ME
    warning('手动测试器启动失败: %s', ME.message);
end

% --- 提示信息 ---
msgbox({'系统就绪！', ...
        '1. 主窗口：调整布局 (Base/Wall/Bucket)', ...
        '2. 小窗口：输入 XYZ 坐标测试机械臂移动', ...
        '3. 主窗口按钮：分析布局 -> 自动规划'}, '操作指引');