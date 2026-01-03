classdef WorkspaceCloudGenerator < handle
    properties
        RobotModel  % 7自由度机器人模型
        NumSamples = 30000; % 采样点数量 (建议 30000-50000，越多越清晰但越慢)
    end
    
    methods
        function obj = WorkspaceCloudGenerator()
            % 初始化机器人 (确保目录下有 ScoopRobot7DOF.m)
            bot = ScoopRobot7DOF(); 
            obj.RobotModel = bot.Robot;
        end
        
        function run(obj)
            fprintf('正在计算 7自由度工作空间点云 (采样数: %d)...\n', obj.NumSamples);
            fprintf('计算原理：蒙特卡洛随机采样 + 正运动学求解\n');
            
            % 1. 定义 7个关节的采样范围 (根据物理限位)
            % UR10 6轴范围 (通常 ±360，但在倒挂抹泥中我们限制在合理范围内以提高效率)
            % [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3, Shovel]
            qMin = [-pi,   -pi,   -pi,   -2*pi, -2*pi, -2*pi, deg2rad(-90)];
            qMax = [ pi,    0,     pi,    2*pi,  2*pi,  2*pi, deg2rad(45)];
            
            % 2. 生成随机关节配置矩阵 (N x 7)
            % rand(N,1) * (max-min) + min
            qSamples = rand(obj.NumSamples, 7) .* (qMax - qMin) + qMin;
            
            % 3. 正运动学计算 (FK) - 核心求解循环
            tcpPoints = zeros(obj.NumSamples, 3);
            validIdx = false(obj.NumSamples, 1);
            
            % 使用 waitbar 显示进度
            hWait = waitbar(0, '正在生成点云数据...');
            
            for i = 1:obj.NumSamples
                config = qSamples(i, :);
                
                % 简单的自碰撞检测 (可选)：剔除铲子戳到天花板的情况
                % 这里主要依靠正运动学计算末端位置
                try
                    tform = getTransform(obj.RobotModel, config, 'tcp_frame');
                    pos = tform(1:3, 4)';
                    
                    % 过滤掉不合理的点 (例如钻入地下的)
                    if pos(3) >= 0 && pos(3) <= 2.0 
                        tcpPoints(i, :) = pos;
                        validIdx(i) = true;
                    end
                catch
                    continue;
                end
                
                if mod(i, 1000) == 0
                    waitbar(i/obj.NumSamples, hWait);
                end
            end
            close(hWait);
            
            % 提取有效点
            finalPoints = tcpPoints(validIdx, :);
            fprintf('有效点数量: %d / %d\n', size(finalPoints, 1), obj.NumSamples);
            
            % 4. 三维可视化渲染
            obj.visualizePointCloud(finalPoints);
        end
        
        function visualizePointCloud(obj, points)
            fig = figure('Name', 'UR10 工作空间点云', 'Color', 'k', 'Position', [100 100 1000 800]);
            ax = axes('Parent', fig, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
            hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
            
            % 绘制机器人基座位置 (参考点)
            plot3(ax, 0, 0, 1.4, 'r+', 'MarkerSize', 15, 'LineWidth', 3);
            text(0, 0, 1.5, 'Robot Base (1.4m)', 'Color', 'r', 'FontSize', 12);
            
            % 绘制点云
            % 技巧：使用 Z 轴高度作为颜色映射 (Color Map)
            x = points(:, 1);
            y = points(:, 2);
            z = points(:, 3);
            
            % scatter3 比 plot3 更适合点云，支持颜色数据
            scatter3(ax, x, y, z, 5, z, 'filled', 'MarkerFaceAlpha', 0.6);
            
            % 设置视觉效果
            colormap(ax, 'jet'); % 彩虹色：蓝色是地面，红色是高处
            cb = colorbar(ax);
            cb.Color = 'w';
            cb.Label.String = 'Height (m)';
            
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            title('7-DOF Reachable Workspace (Monte Carlo Cloud)', 'Color', 'w', 'FontSize', 14);
            view(45, 30);
            
            % 绘制地面辅助网格
            [gx, gy] = meshgrid(-2:0.5:2, -2:0.5:2);
            mesh(ax, gx, gy, zeros(size(gx)), 'FaceColor', 'none', 'EdgeColor', [0.3 0.3 0.3]);
            
            % 5. 自动旋转展示
            fprintf('按任意键停止旋转...\n');
            fps = 30;
            angle = 0;
            while ishandle(fig)
                view(ax, angle, 20); % 保持俯仰角20度，旋转方位角
                angle = angle + 0.5;
                if angle > 360, angle = 0; end
                drawnow limitrate;
                pause(1/fps);
            end
        end
    end
end