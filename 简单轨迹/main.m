clear; clc; close all;

% 启动集成轨迹研究工具
app = Ur10TrajIntegrator();

fprintf('工具已启动：\n');
fprintf('1. 在左侧面板修改起始 (Start) 和 终止 (End) 坐标。\n');
fprintf('2. 开启 [显示工作空间] 辅助观察。\n');
fprintf('3. 点击 [开始执行规划] 查看机械臂实时运动。\n');