classdef ManualPathTester < handle
    properties
        Controller  % 引用主控制器
        Win         % 测试窗口句柄
        Inputs      % 输入框句柄结构体
        IK          % 逆运动学求解器
    end
    
    methods
        function obj = ManualPathTester(mainController)
            obj.Controller = mainController;
            obj.IK = inverseKinematics('RigidBodyTree', obj.Controller.Robot);
            obj.createGUI();
        end
        
        function createGUI(obj)
            % 创建一个小窗口
            obj.Win = figure('Name', '末端点动控制 (Cartesian Control)', 'NumberTitle', 'off', ...
                'MenuBar', 'none', 'ToolBar', 'none', 'Resize', 'off', ...
                'Color', [0.9 0.9 0.95], 'Position', [100, 300, 350, 450]);
            
            % 标题
            uicontrol(obj.Win, 'Style', 'text', 'String', '输入目标世界坐标 (XYZ & RPY)', ...
                'FontSize', 11, 'FontWeight', 'bold', 'Position', [20, 410, 310, 25], 'BackgroundColor', [0.9 0.9 0.95]);
            
            labels = {'X (m):', 'Y (m):', 'Z (m):', 'Roll (rad):', 'Pitch (rad):', 'Yaw (rad):'};
            tags = {'x', 'y', 'z', 'r', 'p', 'yw'};
            
            % 获取当前机械臂位置作为默认值
            currentTCP = obj.getCurrentTCP();
            defaults = {currentTCP(1), currentTCP(2), currentTCP(3), ...
                        obj.Controller.ShovelAttitude(1), ...
                        obj.Controller.ShovelAttitude(2), ...
                        obj.Controller.ShovelAttitude(3)};
            
            % 创建6个输入框
            for i = 1:6
                yPos = 370 - (i-1)*45;
                % 标签
                uicontrol(obj.Win, 'Style', 'text', 'String', labels{i}, ...
                    'Position', [30, yPos+5, 80, 20], 'HorizontalAlignment', 'right', 'BackgroundColor', [0.9 0.9 0.95]);
                % 输入框
                obj.Inputs.(tags{i}) = uicontrol(obj.Win, 'Style', 'edit', 'String', num2str(defaults{i}, '%.3f'), ...
                    'Position', [120, yPos+8, 150, 25], 'BackgroundColor', 'w');
            end
            
            % 按钮区
            uicontrol(obj.Win, 'Style', 'pushbutton', 'String', '读取当前位置 (Get)', ...
                'Position', [30, 80, 130, 35], 'Callback', @(s,e) obj.readCurrentPos());
                
            uicontrol(obj.Win, 'Style', 'pushbutton', 'String', '移动到目标点 (Move)', ...
                'Position', [180, 80, 140, 35], 'FontWeight', 'bold', 'ForegroundColor', 'b', ...
                'Callback', @(s,e) obj.moveToTarget());
                
            uicontrol(obj.Win, 'Style', 'pushbutton', 'String', '生成直线轨迹 (Line)', ...
                'Position', [30, 30, 290, 35], 'ForegroundColor', [0 0.5 0], ...
                'Callback', @(s,e) obj.runLinearPath());
        end
        
        function readCurrentPos(obj)
            pos = obj.getCurrentTCP();
            att = obj.Controller.ShovelAttitude;
            set(obj.Inputs.x, 'String', num2str(pos(1), '%.3f'));
            set(obj.Inputs.y, 'String', num2str(pos(2), '%.3f'));
            set(obj.Inputs.z, 'String', num2str(pos(3), '%.3f'));
            set(obj.Inputs.r, 'String', num2str(att(1), '%.3f'));
            set(obj.Inputs.p, 'String', num2str(att(2), '%.3f'));
            set(obj.Inputs.yw, 'String', num2str(att(3), '%.3f'));
        end
        
        function moveToTarget(obj)
            try
                x = str2double(get(obj.Inputs.x, 'String'));
                y = str2double(get(obj.Inputs.y, 'String'));
                z = str2double(get(obj.Inputs.z, 'String'));
                r = str2double(get(obj.Inputs.r, 'String'));
                p = str2double(get(obj.Inputs.p, 'String'));
                yw = str2double(get(obj.Inputs.yw, 'String'));
                
                tform = trvec2tform([x, y, z]) * eul2tform([yw, p, r], 'ZYX');
                weights = [0.2 0.2 0.2 1 1 1]; % 姿态权重稍低
                initGuess = obj.Controller.CurrentConfig;
                [config, solInfo] = obj.IK('shovel_link', tform, weights, initGuess);
                
                if strcmp(solInfo.Status, 'success')
                    obj.Controller.CurrentConfig = config;
                    obj.Controller.ShovelAttitude = [r, p, yw];
                    obj.Controller.render();
                    title(obj.Controller.Ax, sprintf('✅ 到达: [%.2f, %.2f, %.2f]', x, y, z), 'Color', 'g');
                else
                    errordlg('该位置无法到达！可能超出工作空间。', 'IK Failed');
                end
            catch ME
                errordlg(ME.message);
            end
        end
        
        function runLinearPath(obj)
            startPos = obj.getCurrentTCP();
            endX = str2double(get(obj.Inputs.x, 'String'));
            endY = str2double(get(obj.Inputs.y, 'String'));
            endZ = str2double(get(obj.Inputs.z, 'String'));
            
            steps = 30;
            trajX = linspace(startPos(1), endX, steps);
            trajY = linspace(startPos(2), endY, steps);
            trajZ = linspace(startPos(3), endZ, steps);
            
            r = str2double(get(obj.Inputs.r, 'String'));
            p = str2double(get(obj.Inputs.p, 'String'));
            yw = str2double(get(obj.Inputs.yw, 'String'));
            
            for i = 1:steps
                tform = trvec2tform([trajX(i), trajY(i), trajZ(i)]) * eul2tform([yw, p, r], 'ZYX');
                [config, info] = obj.IK('shovel_link', tform, [1 1 1 1 1 1], obj.Controller.CurrentConfig);
                if strcmp(info.Status, 'success')
                    obj.Controller.CurrentConfig = config;
                    obj.Controller.render();
                    pause(0.01);
                else
                    fprintf('路径中断。\n'); break;
                end
            end
        end
        
        function pos = getCurrentTCP(obj)
            tform = getTransform(obj.Controller.Robot, obj.Controller.CurrentConfig, 'shovel_link');
            pos = tform(1:3, 4)';
        end
    end
end