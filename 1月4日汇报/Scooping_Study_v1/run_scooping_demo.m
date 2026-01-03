classdef run_scooping_demo < handle
    properties
        RobotObj        % ScoopRobot7DOF 实例
        RobotModel      % rigidBodyTree
        
        SimFig, Ax      % 3D 绘图窗口
        CtrlFig         % 控制面板窗口
        
        % 状态数据
        CurrentConfig   % 当前关节角 (1x7)
        BasePos = [0, 0, 1.4]
        BucketPos = [0.6, 0.4]
        
        % 动态参数
        CurrentShovelLen = 0.30; % 当前铲长
        TrajParams = struct('Radius', 0.25, 'Depth', 0.35, 'EntryAngle', -80);
        
        % 绘图句柄
        hTrajLine
        hBucket
        
        % IK 求解器
        IK
    end
    
    methods
        function obj = run_scooping_demo()
            fprintf('正在启动 UR10 铲泥工艺集成调试系统...\n');
            
            % 1. 初始化机器人
            obj.reloadRobot(obj.CurrentShovelLen);
            
            % 2. 启动 GUI
            obj.initGUI();
            
            % 3. 初始渲染
            obj.updateScene();
            obj.previewTrajectory(); 
        end
        
        function reloadRobot(obj, len)
            % 重新加载机器人模型 (用于修改铲子长度)
            fprintf('加载机器人模型 (铲长: %.2fm)...\n', len);
            bot = ScoopRobot7DOF(len); % 调用带参数的构造函数
            obj.RobotObj = bot;
            obj.RobotModel = bot.Robot;
            obj.CurrentConfig = bot.HomeConfig;
            
            % 重置 IK
            obj.IK = inverseKinematics('RigidBodyTree', obj.RobotModel);
            obj.IK.SolverParameters.MaxIterations = 100;
        end
        
        function initGUI(obj)
            % --- 主 3D 视图 ---
            obj.SimFig = figure('Name', '3D 仿真视图', 'Color', [0.2 0.2 0.2], ...
                'Units', 'normalized', 'Position', [0.35 0.1 0.6 0.8]);
            obj.Ax = axes('Parent', obj.SimFig, 'Color', [0.2 0.2 0.2], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
            hold(obj.Ax, 'on'); grid(obj.Ax, 'on'); axis(obj.Ax, 'equal');
            view(obj.Ax, 135, 20);
            xlabel('X'); ylabel('Y'); zlabel('Z');
            
            % 绘制环境基础
            patch(obj.Ax, [-3 3 3 -3], [-3 -3 3 3], [0 0 0 0], [0.3 0.3 0.3], 'FaceAlpha', 0.5);
            plot3(obj.Ax, [-1 1], [0 0], [1.4 1.4], 'c-', 'LineWidth', 3);
            
            % --- 控制面板 ---
            obj.CtrlFig = figure('Name', '控制面板', 'Units', 'normalized', ...
                'Position', [0.02 0.05 0.3 0.9], 'MenuBar', 'none', 'NumberTitle', 'off', 'Color', [0.95 0.95 0.95]);
            
            % 1. 模型参数设置 (悬臂长度)
            pnlModel = uipanel(obj.CtrlFig, 'Title', '1. 模型设置', 'Position', [0.02 0.88 0.96 0.10], 'FontSize', 10, 'FontWeight', 'bold');
            uicontrol(pnlModel, 'Style', 'text', 'String', '铲子/悬臂长度 (m):', 'Units', 'normalized', 'Position', [0.05 0.4 0.4 0.4], 'HorizontalAlignment', 'left');
            edtLen = uicontrol(pnlModel, 'Style', 'edit', 'String', num2str(obj.CurrentShovelLen), ...
                'Units', 'normalized', 'Position', [0.45 0.5 0.2 0.4]);
            uicontrol(pnlModel, 'Style', 'pushbutton', 'String', '应用更改', ...
                'Units', 'normalized', 'Position', [0.7 0.4 0.25 0.5], ...
                'Callback', @(s,e) obj.onApplyLength(edtLen));

            % 2. 机械臂姿态调控
            pnlJoints = uipanel(obj.CtrlFig, 'Title', '2. 机械臂关节调控', 'Position', [0.02 0.50 0.96 0.36], 'FontSize', 10, 'FontWeight', 'bold');
            jNames = {'Base (Yaw)', 'Shoulder', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3', 'Shovel Pitch'};
            ranges = [-pi pi; -pi 0; -pi pi; -2*pi 2*pi; -2*pi 2*pi; -2*pi 2*pi; deg2rad(-90) deg2rad(45)];
            
            for i = 1:7
                yPos = 0.88 - (i-1)*0.12;
                uicontrol(pnlJoints, 'Style', 'text', 'String', jNames{i}, 'Units', 'normalized', 'Position', [0.05 yPos 0.35 0.08], 'HorizontalAlignment', 'left');
                uicontrol(pnlJoints, 'Style', 'slider', 'Min', ranges(i,1), 'Max', ranges(i,2), 'Value', obj.CurrentConfig(i), ...
                    'Units', 'normalized', 'Position', [0.4 yPos 0.55 0.08], 'Callback', @(s,e) obj.onJointChange(s, i));
            end
            
            % 3. 曲线参数设置
            pnlCurve = uipanel(obj.CtrlFig, 'Title', '3. 铲泥曲线设置', 'Position', [0.02 0.22 0.96 0.26], 'FontSize', 10, 'FontWeight', 'bold');
            
            % 半径
            uicontrol(pnlCurve, 'Style', 'text', 'String', '作业半径 R:', 'Units', 'normalized', 'Position', [0.05 0.8 0.3 0.1]);
            uicontrol(pnlCurve, 'Style', 'slider', 'Min', 0.15, 'Max', 0.5, 'Value', obj.TrajParams.Radius, ...
                'Units', 'normalized', 'Position', [0.4 0.8 0.55 0.1], 'Callback', @(s,e) obj.onParamChange(s, 'Radius'));
            
            % 深度
            uicontrol(pnlCurve, 'Style', 'text', 'String', '挖掘深度 D:', 'Units', 'normalized', 'Position', [0.05 0.5 0.3 0.1]);
            uicontrol(pnlCurve, 'Style', 'slider', 'Min', 0.1, 'Max', 0.6, 'Value', obj.TrajParams.Depth, ...
                'Units', 'normalized', 'Position', [0.4 0.5 0.55 0.1], 'Callback', @(s,e) obj.onParamChange(s, 'Depth'));
            
            % 入水角
            uicontrol(pnlCurve, 'Style', 'text', 'String', '入水角度:', 'Units', 'normalized', 'Position', [0.05 0.2 0.3 0.1]);
            uicontrol(pnlCurve, 'Style', 'slider', 'Min', -90, 'Max', -45, 'Value', obj.TrajParams.EntryAngle, ...
                'Units', 'normalized', 'Position', [0.4 0.2 0.55 0.1], 'Callback', @(s,e) obj.onParamChange(s, 'EntryAngle'));

            % 4. 动画控制
            pnlAction = uipanel(obj.CtrlFig, 'Title', '4. 运行', 'Position', [0.02 0.02 0.96 0.18], 'FontSize', 10, 'FontWeight', 'bold');
            uicontrol(pnlAction, 'Style', 'pushbutton', 'String', '▶ 开始铲泥动画', ...
                'Units', 'normalized', 'Position', [0.05 0.55 0.9 0.4], ...
                'BackgroundColor', [0 0.6 0.3], 'ForegroundColor', 'w', 'FontSize', 12, 'FontWeight', 'bold', ...
                'Callback', @(s,e) obj.playAnimation());
            uicontrol(pnlAction, 'Style', 'pushbutton', 'String', '重置姿态', ...
                'Units', 'normalized', 'Position', [0.05 0.1 0.9 0.35], ...
                'Callback', @(s,e) obj.resetPose());
        end
        
        function updateScene(obj)
            cla(obj.Ax);
            % 环境
            patch(obj.Ax, [-3 3 3 -3], [-3 -3 3 3], [0 0 0 0], [0.3 0.3 0.3], 'FaceAlpha', 0.5);
            plot3(obj.Ax, [-1 1], [0 0], [1.4 1.4], 'c-', 'LineWidth', 3);
            
            % 机器人 (更新基座位置)
            baseBody = obj.RobotModel.getBody('base_link');
            setFixedTransform(baseBody.Joint, trvec2tform(obj.BasePos) * axang2tform([1 0 0 pi]));
            
            show(obj.RobotModel, obj.CurrentConfig, 'Parent', obj.Ax, 'PreservePlot', false, 'FastUpdate', true);
            
            % 绘制桶
            if isempty(obj.hBucket) || ~isvalid(obj.hBucket)
                [cx, cy, cz] = cylinder(obj.TrajParams.Radius);
                cz = cz * 0.5; cx = cx + obj.BucketPos(1); cy = cy + obj.BucketPos(2);
                obj.hBucket = surf(obj.Ax, cx, cy, cz, 'FaceColor', 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
            else
                 % 更新桶大小
                 [cx, cy, cz] = cylinder(obj.TrajParams.Radius);
                 cz = cz * 0.5; cx = cx + obj.BucketPos(1); cy = cy + obj.BucketPos(2);
                 set(obj.hBucket, 'XData', cx, 'YData', cy, 'ZData', cz);
            end
            
            % 轨迹线
            if isempty(obj.hTrajLine) || ~isvalid(obj.hTrajLine)
                obj.previewTrajectory();
            else
                % previewTrajectory 会更新它
            end
            
            axis(obj.Ax, [-1 2 -1.5 1.5 0 2]);
        end
        
        function previewTrajectory(obj)
            gen = ScoopPathGenerator();
            [tcpPath, ~, ~] = gen.generateScoopMotion( ...
                [obj.BucketPos, 0.4], obj.TrajParams.Radius, obj.TrajParams.Depth);
            
            if isempty(obj.hTrajLine) || ~isvalid(obj.hTrajLine)
                obj.hTrajLine = plot3(obj.Ax, tcpPath(:,1), tcpPath(:,2), tcpPath(:,3), 'r.-', 'LineWidth', 1.5);
            else
                set(obj.hTrajLine, 'XData', tcpPath(:,1), 'YData', tcpPath(:,2), 'ZData', tcpPath(:,3));
            end
        end
        
        function onApplyLength(obj, edt)
            val = str2double(get(edt, 'String'));
            if isnan(val) || val <= 0, warndlg('请输入有效的长度数值'); return; end
            obj.CurrentShovelLen = val;
            obj.reloadRobot(val);
            obj.updateScene();
            msgbox(sprintf('已更新铲子长度为 %.2fm', val));
        end
        
        function onJointChange(obj, src, idx)
            obj.CurrentConfig(idx) = get(src, 'Value');
            obj.updateScene();
        end
        
        function onParamChange(obj, src, type)
            obj.TrajParams.(type) = get(src, 'Value');
            obj.updateScene(); % 触发桶和轨迹的重绘
            obj.previewTrajectory();
        end
        
        function resetPose(obj)
            obj.CurrentConfig = obj.RobotObj.HomeConfig;
            obj.updateScene();
        end
        
        function playAnimation(obj)
            title(obj.Ax, '正在计算并执行...', 'Color', 'g');
            drawnow;
            
            gen = ScoopPathGenerator();
            bucketCenter = [obj.BucketPos, 0.4];
            [tcpPath, targetAngles, ~] = gen.generateScoopMotion(bucketCenter, obj.TrajParams.Radius, obj.TrajParams.Depth);
            
            % 寻找起点
            startPos = tcpPath(1,:);
            yawAngle = atan2(startPos(2), startPos(1));
            
            startTform = trvec2tform(startPos) * axang2tform([0 0 1 yawAngle]) * axang2tform([0 1 0 targetAngles(1)]);
            
            % 宽松权重
            weights = [0.1 1 0.1 100 100 100]; 
            [qStart, info] = obj.IK('tcp_frame', startTform, weights, obj.CurrentConfig);
            
            if ~strcmp(info.Status, 'success')
                title(obj.Ax, '起点不可达!', 'Color', 'r');
                warndlg('无法到达轨迹起点，请检查模型长度或关节限制。');
                return;
            end
            
            qTraj = qStart;
            for i = 1:size(tcpPath, 1)
                tPos = tcpPath(i, :);
                tPitch = targetAngles(i);
                
                % 目标位姿
                tgtTform = trvec2tform(tPos) * axang2tform([0 0 1 yawAngle]) * axang2tform([0 1 0 tPitch]);
                weights = [0 1 0 100 100 100]; % 强位置约束
                
                [qSol, ~] = obj.IK('tcp_frame', tgtTform, weights, qTraj);
                qTraj = qSol;
                
                obj.CurrentConfig = qSol;
                show(obj.RobotModel, qSol, 'Parent', obj.Ax, 'PreservePlot', false, 'FastUpdate', true);
                drawnow limitrate;
            end
            title(obj.Ax, '完成 ✅', 'Color', 'w');
        end
    end
end