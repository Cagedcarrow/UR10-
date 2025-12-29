classdef Ur10Controller < handle
    properties
        Robot, CurrentConfig, SimFig, CtrlFig, Ax
        JLables, SLables, LLables 
        BasePos = [0, 0, 1.4] 
        WallParams = [1.6, 0, 1] 
        BucketPos = [0.6, 0.5]
        ShovelAttitude = [0, 0, 0]
        WallObj, BucketObj, ShovelLength
        
        % --- 可视化相关句柄 ---
        WorkspacePatch = []
        ShowWS = false; 
        
        % 新增：红色接触点云句柄 & 坐标标签句柄
        ContactDots = [] 
        CenterMarker = []
        CenterText = []
        
        IsLayoutValid = false; 
    end
    
    methods
        function obj = Ur10Controller(gantryHeight, shovelLength)
            obj.BasePos(3) = gantryHeight;
            obj.ShovelLength = shovelLength;
            obj.Robot = loadrobot('universalUR10', 'DataFormat', 'row');
            
            shovelBody = rigidBody('shovel_link');
            shovelJoint = rigidBodyJoint('shovel_joint', 'fixed');
            setFixedTransform(shovelJoint, trvec2tform([0, 0, shovelLength]));
            shovelBody.Joint = shovelJoint;
            addVisual(shovelBody, "Box", [0.1 0.01 shovelLength], trvec2tform([0, 0, shovelLength/2]));
            addBody(obj.Robot, shovelBody, 'tool0');
            
            obj.WallObj = collisionBox(0.1, 2.0, obj.WallParams(3));
            obj.BucketObj = collisionCylinder(0.2, 0.4);
            obj.CurrentConfig = [0, -pi/2, pi/2, -pi/2, -pi/2, 0];
        end
        
        function launchGUI(obj)
            obj.SimFig = figure('Name', 'UR10 仿真 - 3D 视图', 'Color', 'w', ...
                'Units', 'normalized', 'Position', [0.35 0.1 0.6 0.8]);
            obj.Ax = axes('Parent', obj.SimFig);
            hold(obj.Ax, 'on'); grid(obj.Ax, 'on'); view(obj.Ax, 135, 30);
            axis(obj.Ax, [-3 3 -3 3 0 4]); axis(obj.Ax, 'equal');
            rotate3d(obj.SimFig, 'on');

            obj.CtrlFig = figure('Name', '操作控制台', 'Color', [0.95 0.95 0.95], ...
                'Units', 'normalized', 'Position', [0.05 0.1 0.28 0.85], 'MenuBar', 'none');
            
            obj.createControlPanels(); 
            obj.render();
        end

        function render(obj)
            if isempty(obj.Ax) || ~isgraphics(obj.Ax), return; end
            
            [az, el] = view(obj.Ax);
            
            % 清理上一帧的动态元素 (保留静态背景以提高性能)
            if ~isempty(obj.ContactDots) && isvalid(obj.ContactDots), delete(obj.ContactDots); end
            if ~isempty(obj.CenterMarker) && isvalid(obj.CenterMarker), delete(obj.CenterMarker); end
            if ~isempty(obj.CenterText) && isvalid(obj.CenterText), delete(obj.CenterText); end
            if ~isempty(obj.WorkspacePatch) && isvalid(obj.WorkspacePatch), delete(obj.WorkspacePatch); end
            
            cla(obj.Ax); hold(obj.Ax, 'on');
            
            % 1. 机器人与环境基本更新
            T_base = trvec2tform(obj.BasePos) * axang2tform([1 0 0 pi]);
            setFixedTransform(obj.Robot.getBody('base_link').Joint, T_base);
            T_shovel = trvec2tform([0, 0, obj.ShovelLength]) * eul2tform(fliplr(obj.ShovelAttitude), 'ZYX');
            setFixedTransform(obj.Robot.getBody('shovel_link').Joint, T_shovel);
            
            obj.WallObj = collisionBox(0.1, 2.0, obj.WallParams(3));
            obj.WallObj.Pose = trvec2tform([obj.WallParams(1), obj.WallParams(2), obj.WallParams(3)/2]);
            obj.BucketObj.Pose = trvec2tform([obj.BucketPos(1), obj.BucketPos(2), 0.2]);
            
            % 2. 绘制基础辅助线
            [GX, GY] = meshgrid(-3:1:3, -3:1:3);
            mesh(obj.Ax, GX, GY, ones(size(GX))*obj.BasePos(3), 'FaceColor', 'none', 'EdgeColor', [0.8 0.2 0.2], 'LineStyle', ':');
            patch(obj.Ax, 'XData', [-3 3 3 -3], 'YData', [-3 -3 3 3], 'ZData', [0 0 0 0], 'FaceColor', [0.9 0.9 0.9], 'FaceAlpha', 0.2);
            
            % 3. 计算工作空间参数 (半径 = 臂展 + 铲子)
            maxReach = 1.3 + obj.ShovelLength;
            
            % --- 功能A: 工作空间球体 (淡青色) ---
            if obj.ShowWS
                [sx, sy, sz] = sphere(40);
                sx = sx * maxReach + obj.BasePos(1);
                sy = sy * maxReach + obj.BasePos(2);
                sz = sz * maxReach + obj.BasePos(3);
                obj.WorkspacePatch = surf(obj.Ax, sx, sy, sz, 'FaceColor', 'c', 'EdgeColor', 'none', 'FaceAlpha', 0.05);
            end
            
            % --- 功能B: 绘制接触区域 (红色热力点) ---
            if obj.ShowWS
                % B1. 生成墙面检测点云 (面向机器人的那一面)
                [wy, wz] = meshgrid(obj.WallParams(2)-1:0.1:obj.WallParams(2)+1, 0:0.1:obj.WallParams(3));
                wx = ones(size(wy)) * (obj.WallParams(1) - 0.05); % 墙表面
                
                % B2. 生成桶面检测点云 (桶口和侧面)
                theta = 0:0.2:2*pi;
                bx_rim = obj.BucketPos(1) + 0.2*cos(theta);
                by_rim = obj.BucketPos(2) + 0.2*sin(theta);
                bz_rim = ones(size(theta)) * 0.4; % 桶口高度
                
                % B3. 合并所有检测点
                checkPts = [wx(:), wy(:), wz(:); bx_rim(:), by_rim(:), bz_rim(:)];
                
                % B4. 计算距离并筛选
                dists = sqrt(sum((checkPts - obj.BasePos).^2, 2));
                inRangeIdx = dists <= maxReach & dists >= 0.3; % 在最大射程内且不在死区
                
                % B5. 绘制红色接触点
                if any(inRangeIdx)
                    obj.ContactDots = plot3(obj.Ax, checkPts(inRangeIdx,1), checkPts(inRangeIdx,2), checkPts(inRangeIdx,3), ...
                        '.', 'Color', 'r', 'MarkerSize', 8);
                end
            end
            
            % --- 功能C: 桶口几何中心标注 ---
            center = [obj.BucketPos(1), obj.BucketPos(2), 0.4];
            % 画十字星
            obj.CenterMarker = plot3(obj.Ax, center(1), center(2), center(3), ...
                'p', 'MarkerSize', 12, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'y');
            % 画坐标文字
            coordStr = sprintf(' Top: (%.2f, %.2f, %.2f)', center(1), center(2), center(3));
            obj.CenterText = text(obj.Ax, center(1), center(2), center(3)+0.15, coordStr, ...
                'Color', 'k', 'FontSize', 9, 'FontWeight', 'bold', 'BackgroundColor', 'w', 'EdgeColor', 'k');
            
            % 4. 渲染实体
            show(obj.WallObj, 'Parent', obj.Ax);
            show(obj.BucketObj, 'Parent', obj.Ax);
            show(obj.Robot, obj.CurrentConfig, 'Parent', obj.Ax, 'Visuals', 'on', 'PreservePlot', false, 'FastUpdate', true);
            
            view(obj.Ax, az, el);
            camlight(obj.Ax, 'headline'); lighting(obj.Ax, 'gouraud');
            axis(obj.Ax, [-3 3 -3 3 0 4]);
            
            if obj.IsLayoutValid
                title(obj.Ax, '✅ 验证通过', 'Color', [0 0.6 0]);
            else
                title(obj.Ax, '⚠️ 调整布局中...', 'Color', 'k');
            end
            
            drawnow limitrate;
        end
        
        function toggleWorkspace(obj, src)
            obj.ShowWS = get(src, 'Value');
            obj.render();
        end
        
        function analyzeReachability(obj)
             fprintf('正在检测边界接触...\n');
             obj.IsLayoutValid = false; 
             bucketTop = [obj.BucketPos(1), obj.BucketPos(2), 0.4]; 
             wallTop = [obj.WallParams(1)-0.12, obj.WallParams(2), obj.WallParams(3)]; 
             try
                 [bOk, ~] = checkReachability(obj.Robot, bucketTop, [0,0,0], obj.CurrentConfig);
                 [wOk, ~] = checkReachability(obj.Robot, wallTop, [0, obj.ShovelAttitude(2), 0], obj.CurrentConfig);
                 if bOk && wOk
                     obj.IsLayoutValid = true;
                     msgbox('✅ 验证通过！', '成功');
                 else
                     errordlg('❌ 部分点不可达', '不可达');
                 end
             catch ME
                 errordlg(ME.message);
             end
             obj.render(); 
        end

        function runTrajectoryPlanning(obj)
            if ~obj.IsLayoutValid, warndlg('请先验证！'); return; end
            try
                planner = TrajectoryPlanner(obj);
                [traj, ~] = planner.planFullTask();
                planner.visualizeTraj(traj);
            catch ME
                errordlg(ME.message);
            end
        end

        function createControlPanels(obj)
            % 1. 关节面板
            bg1 = uipanel(obj.CtrlFig, 'Position', [0.05 0.75 0.9 0.23], 'Title', '1. 关节微调');
            for i = 1:6
                y = 0.85 - (i-1)*0.16;
                obj.JLables(i) = uicontrol('Parent', bg1, 'Style', 'text', 'Units', 'normalized', 'Position', [0.5 y+0.06 0.4 0.08], 'String', num2str(obj.CurrentConfig(i), '%.2f'));
                uicontrol('Parent', bg1, 'Style', 'slider', 'Units', 'normalized', 'Min', -2*pi, 'Max', 2*pi, 'Value', obj.CurrentConfig(i), 'Position', [0.05 y 0.9 0.08], 'Callback', @(src, ev) obj.syncData(src, 'joint', i));
            end
            
            % 2. 布局面板
            bg3 = uipanel(obj.CtrlFig, 'Position', [0.05 0.22 0.9 0.51], 'Title', '2. 场景布局');
            
            uicontrol('Parent', bg3, 'Style', 'togglebutton', 'Units', 'normalized', ...
                'Position', [0.05 0.92 0.9 0.07], 'String', '🔵 显示范围 & 接触热区', ...
                'FontSize', 10, 'Callback', @(s,e) obj.toggleWorkspace(s));
            
            cfg = {'Base X',-2,2,obj.BasePos(1),'bx'; 'Base Y',-2,2,obj.BasePos(2),'by'; 'Base Z',1,3,obj.BasePos(3),'bz'; ...
                   'Wall X',0.2,2.5,obj.WallParams(1),'wx'; 'Wall Y',-2,2,obj.WallParams(2),'wy'; 'Wall H',0.1,3.5,obj.WallParams(3),'wh'; ...
                   'Bucket X',0,2,obj.BucketPos(1),'kx'; 'Bucket Y',-2,2,obj.BucketPos(2),'ky'};
            for i = 1:8
                y = 0.82 - (i-1)*0.10;
                uicontrol('Parent', bg3, 'Style', 'text', 'Units', 'normalized', 'Position', [0.05 y+0.04 0.5 0.05], 'String', cfg{i,1}, 'HorizontalAlignment','left');
                obj.LLables(i) = uicontrol('Parent', bg3, 'Style', 'text', 'Units', 'normalized', 'Position', [0.5 y+0.04 0.4 0.05], 'String', num2str(cfg{i,4}, '%.2f'));
                uicontrol('Parent', bg3, 'Style', 'slider', 'Units', 'normalized', 'Min', cfg{i,2}, 'Max', cfg{i,3}, 'Value', cfg{i,4}, 'Position', [0.05 y 0.9 0.05], 'Callback', @(src, ev) obj.syncData(src, cfg{i,5}, i));
            end
            
            % 3. 任务按钮
            bg4 = uipanel(obj.CtrlFig, 'Position', [0.05 0.01 0.9 0.20], 'Title', '3. 任务操作');
            uicontrol('Parent', bg4, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.1 0.55 0.8 0.35], 'String', '第一步：分析可行性', 'Callback', @(s,e) obj.analyzeReachability());
            uicontrol('Parent', bg4, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.35], 'String', '第二步：开始规划', 'Callback', @(s,e) obj.runTrajectoryPlanning());
        end

        function syncData(obj, src, type, idx)
            val = get(src, 'Value');
            obj.IsLayoutValid = false; 
            if strcmp(type, 'joint')
                obj.CurrentConfig(idx) = val; set(obj.JLables(idx), 'String', num2str(val, '%.2f'));
            elseif strcmp(type, 'shovel')
                obj.ShovelAttitude(idx) = val; set(obj.SLables(idx), 'String', num2str(val, '%.2f'));
            else
                set(obj.LLables(idx), 'String', num2str(val, '%.2f'));
                switch type
                    case 'bx', obj.BasePos(1) = val; case 'by', obj.BasePos(2) = val; case 'bz', obj.BasePos(3) = val;
                    case 'wx', obj.WallParams(1) = val; case 'wy', obj.WallParams(2) = val; case 'wh', obj.WallParams(3) = val;
                    case 'kx', obj.BucketPos(1) = val; case 'ky', obj.BucketPos(2) = val;
                end
            end
            obj.render();
        end
    end
end