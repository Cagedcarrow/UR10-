classdef run_scooping_demo < handle
    properties
        RobotObj, RobotModel, SimFig, Ax, CtrlFig
        CurrentConfig   % 当前 1x7 关节角
        
        % 环境布局参数
        BasePos = [0, 0, 1.5]      % 龙门架挂高 (建议 1.5m 以上防止跳变)
        BucketPos = [0.6, 0.4]     % 桶中心位置 (X, Y)
        
        % 动态参数
        CurrentShovelLen = 0.35; 
        TrajParams = struct('Radius', 0.25, 'Depth', 0.35, 'EntryAngle', -85);
        
        % 句柄与解算器
        StoredTraj = []         % 预解算轨迹
        hTrajLine, hBucket, hProgressSlider, IK
        lblBaseZ, lblBucketX, lblPitch
    end
    
    methods
        function obj = run_scooping_demo()
            fprintf('正在启动 7-DOF 铲泥工艺集成终端 (抛物线轨迹 + 姿态锁定版)...\n');
            obj.reloadRobot(obj.CurrentShovelLen);
            obj.initGUI();
            obj.updateScene();
        end
        
        function reloadRobot(obj, len)
            % 加载模型 (确保目录下有最新的 ScoopRobot7DOF.m)
            bot = ScoopRobot7DOF(len); 
            obj.RobotObj = bot;
            obj.RobotModel = bot.Robot;
            obj.CurrentConfig = bot.HomeConfig;
            
            % 初始化逆运动学解算器
            obj.IK = inverseKinematics('RigidBodyTree', obj.RobotModel);
            obj.IK.SolverParameters.MaxIterations = 150; 
        end
        
        function initGUI(obj)
            % --- 3D 视图窗口 ---
            obj.SimFig = figure('Name', 'UR10 铲泥动画仿真', 'Color', [0.1 0.1 0.1], ...
                'Units', 'normalized', 'Position', [0.35 0.1 0.6 0.8]);
            obj.Ax = axes('Parent', obj.SimFig, 'Color', [0.1 0.1 0.1], 'XColor','w', 'YColor','w', 'ZColor','w');
            hold(obj.Ax,'on'); grid(obj.Ax,'on'); axis(obj.Ax,'equal'); view(obj.Ax, 135, 20);
            
            % --- 控制面板 ---
            obj.CtrlFig = figure('Name', '参数控制台', 'Units', 'normalized', 'Position', [0.02 0.05 0.3 0.9], 'MenuBar', 'none', 'NumberTitle', 'off');
            
            % 1. 布局调整
            pnlEnv = uipanel(obj.CtrlFig, 'Title', '1. 环境布局 (解决跳变)', 'Position', [0.02 0.78 0.96 0.20], 'FontWeight', 'bold');
            uicontrol(pnlEnv, 'Style', 'text', 'String', '龙门架高度 (Z):', 'Units', 'normalized', 'Position', [0.05 0.7 0.4 0.15], 'HorizontalAlignment', 'left');
            obj.lblBaseZ = uicontrol(pnlEnv, 'Style', 'text', 'String', num2str(obj.BasePos(3)), 'Units', 'normalized', 'Position', [0.8 0.7 0.15 0.15]);
            uicontrol(pnlEnv, 'Style', 'slider', 'Min', 1.2, 'Max', 2.2, 'Value', obj.BasePos(3), 'Units', 'normalized', 'Position', [0.05 0.55 0.9 0.15], 'Callback', @(s,e) obj.onLayoutChange(s, 'BaseZ'));
            
            uicontrol(pnlEnv, 'Style', 'text', 'String', '泥桶距离 (X):', 'Units', 'normalized', 'Position', [0.05 0.25 0.4 0.15], 'HorizontalAlignment', 'left');
            obj.lblBucketX = uicontrol(pnlEnv, 'Style', 'text', 'String', num2str(obj.BucketPos(1)), 'Units', 'normalized', 'Position', [0.8 0.25 0.15 0.15]);
            uicontrol(pnlEnv, 'Style', 'slider', 'Min', 0.4, 'Max', 1.0, 'Value', obj.BucketPos(1), 'Units', 'normalized', 'Position', [0.05 0.1 0.9 0.15], 'Callback', @(s,e) obj.onLayoutChange(s, 'BucketX'));

            % 2. 轨迹预览与计算
            pnlAction = uipanel(obj.CtrlFig, 'Title', '2. 轨迹生成与预览', 'Position', [0.02 0.45 0.96 0.32], 'FontWeight', 'bold');
            uicontrol(pnlAction, 'Style', 'pushbutton', 'String', '⚡ 计算抛物线轨迹', 'Units', 'normalized', 'Position', [0.05 0.82 0.9 0.12], 'BackgroundColor', [0 0.5 0.8], 'ForegroundColor', 'w', 'Callback', @(s,e) obj.calculateFullTraj());
            
            uicontrol(pnlAction, 'Style', 'text', 'String', '预览进度:', 'Units', 'normalized', 'Position', [0.05 0.7 0.3 0.08], 'HorizontalAlignment', 'left');
            obj.hProgressSlider = uicontrol(pnlAction, 'Style', 'slider', 'Min', 1, 'Max', 100, 'Value', 1, 'Enable', 'off', 'Units', 'normalized', 'Position', [0.05 0.6 0.9 0.08], 'Callback', @(s,e) obj.onProgressScroll(s));
            
            % 关节微调
            jNames = {'Base', 'Shoulder', 'Elbow', 'W1', 'W2', 'W3', 'Shovel'};
            for i = 1:7
                yP = 0.5 - (i-1)*0.07;
                uicontrol(pnlAction, 'Style', 'text', 'String', jNames{i}, 'Units', 'normalized', 'Position', [0.05 yP 0.2 0.06]);
                uicontrol(pnlAction, 'Style', 'slider', 'Min', -pi, 'Max', pi, 'Value', obj.CurrentConfig(i), 'Units', 'normalized', 'Position', [0.3 yP 0.65 0.06], 'Callback', @(s,e) obj.onJointChange(s, i));
            end

            % 3. 运行控制
            pnlRun = uipanel(obj.CtrlFig, 'Title', '3. 播放控制', 'Position', [0.02 0.02 0.96 0.15], 'FontWeight', 'bold');
            uicontrol(pnlRun, 'Style', 'pushbutton', 'String', '▶ 播放铲泥动画', 'Units', 'normalized', 'Position', [0.05 0.55 0.9 0.35], 'BackgroundColor', [0 0.6 0.3], 'ForegroundColor', 'w', 'Callback', @(s,e) obj.playStoredAnimation());
            uicontrol(pnlRun, 'Style', 'pushbutton', 'String', '重置', 'Units', 'normalized', 'Position', [0.05 0.1 0.9 0.35], 'Callback', @(s,e) obj.resetPose());
        end

        function calculateFullTraj(obj)
    fprintf('开始执行“解耦控制”解算：机械臂定位末端，电机独立控制铲子...\n');
    gen = ScoopPathGenerator();
    [tcpPath, targetAngles, ~] = gen.generateScoopMotion([obj.BucketPos, 0.4], obj.TrajParams.Radius, obj.TrajParams.Depth);
    
    numSteps = size(tcpPath, 1);
    obj.StoredTraj = zeros(numSteps, 7);
    qLast = obj.CurrentConfig;
    yawAngle = atan2(obj.BucketPos(2), obj.BucketPos(1));

    for i = 1:numSteps
        % 1. 计算当前的铲子俯仰角 (Planner 给出)
        currentPitch = targetAngles(i);
        
        % 2. 反推机械臂末端(电机挂载点)的世界位置
        % 我们希望铲尖(tcpPath)到达指定位置，已知铲子长 ShovelLen
        % MountPos = TipPos - [ShovelLen * cos(pitch), 0, ShovelLen * sin(pitch)] (在局部系下)
        dx = obj.CurrentShovelLen * cos(currentPitch);
        dz = obj.CurrentShovelLen * sin(currentPitch);
        
        % 转换到世界系下的 Mount 位置
        mountPos = tcpPath(i,:) - [dx*cos(yawAngle), dx*sin(yawAngle), dz];
        
        % 3. 构建电机挂载点的目标位姿 (末端法兰始终朝向桶中心)
        tgtTform = trvec2tform(mountPos) * axang2tform([0 0 1 yawAngle]);
        
        % 4. 只对前 6 轴求解 IK，目标设为 mount_link 【关键：保证末端不翻转】
        weights = [10 10 10 100 100 100]; 
        [qSol, info] = obj.IK('mount_link', tgtTform, weights, qLast);
        
        % 5. 直接将第 7 轴角度赋予解向量，不让 IK 干扰它
        qSol(7) = currentPitch;
        
        obj.StoredTraj(i,:) = qSol;
        qLast = qSol;
    end
    set(obj.hProgressSlider, 'Enable', 'on', 'Max', numSteps, 'Value', 1);
    msgbox('计算完成！采用末端解耦控制，跳变与翻转已消除。');
end

        function updateScene(obj)
            if isempty(obj.Ax) || ~isgraphics(obj.Ax), return; end
            cla(obj.Ax);
            patch(obj.Ax, [-3 3 3 -3], [-3 -3 3 3], [0 0 0 0], [0.3 0.3 0.3], 'FaceAlpha', 0.5);
            plot3(obj.Ax, [-1 1], [0 0], [obj.BasePos(3) obj.BasePos(3)], 'c-', 'LineWidth', 3); % 龙门架
            
            baseBody = obj.RobotModel.getBody('base_link');
            setFixedTransform(baseBody.Joint, trvec2tform(obj.BasePos) * axang2tform([1 0 0 pi]));
            show(obj.RobotModel, obj.CurrentConfig, 'Parent', obj.Ax, 'PreservePlot', false, 'FastUpdate', true);
            
            % 绘制桶
            [cx, cy, cz] = cylinder(obj.TrajParams.Radius);
            cz = cz * 0.4; cx = cx + obj.BucketPos(1); cy = cy + obj.BucketPos(2);
            surf(obj.Ax, cx, cy, cz, 'FaceColor', 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'b');
            
            % 预览轨迹
            obj.previewTrajectory();
            axis(obj.Ax, [-1 2 -1.5 1.5 0 2.5]);
        end

        function onLayoutChange(obj, src, type)
            val = get(src, 'Value');
            if strcmp(type, 'BaseZ'), obj.BasePos(3) = val; set(obj.lblBaseZ, 'String', sprintf('%.2f', val)); end
            if strcmp(type, 'BucketX'), obj.BucketPos(1) = val; set(obj.lblBucketX, 'String', sprintf('%.2f', val)); end
            obj.updateScene();
            obj.StoredTraj = []; set(obj.hProgressSlider, 'Enable', 'off');
        end

        function onProgressScroll(obj, src)
            idx = round(get(src, 'Value'));
            if ~isempty(obj.StoredTraj)
                obj.CurrentConfig = obj.StoredTraj(idx, :);
                obj.updateScene();
                % 显示实时铲面角度
                tform = getTransform(obj.RobotModel, obj.CurrentConfig, 'tcp_frame');
                eul = rotm2eul(tform(1:3,1:3), 'ZYX');
                title(obj.Ax, sprintf('进度: %d/%d | 实时俯仰角: %.1f°', idx, size(obj.StoredTraj,1), rad2deg(eul(2))), 'Color', 'w');
            end
        end

        function previewTrajectory(obj)
            gen = ScoopPathGenerator();
            [tcpPath, ~, ~] = gen.generateScoopMotion([obj.BucketPos, 0.4], obj.TrajParams.Radius, obj.TrajParams.Depth);
            plot3(obj.Ax, tcpPath(:,1), tcpPath(:,2), tcpPath(:,3), 'r-', 'LineWidth', 1.5);
        end

        function playStoredAnimation(obj)
            if isempty(obj.StoredTraj), return; end
            for i = 1:size(obj.StoredTraj, 1)
                set(obj.hProgressSlider, 'Value', i);
                obj.onProgressScroll(obj.hProgressSlider);
                pause(0.04); 
            end
        end

        function onApplyLength(obj, edt)
            val = str2double(get(edt, 'String'));
            if ~isnan(val), obj.CurrentShovelLen = val; obj.reloadRobot(val); obj.updateScene(); end
        end

        function onJointChange(obj, src, idx)
            obj.CurrentConfig(idx) = get(src, 'Value'); obj.updateScene();
        end

        function onParamChange(obj, src, type)
            obj.TrajParams.(type) = get(src, 'Value'); obj.updateScene();
        end

        function resetPose(obj)
            obj.CurrentConfig = obj.RobotObj.HomeConfig; obj.updateScene();
        end
    end
end