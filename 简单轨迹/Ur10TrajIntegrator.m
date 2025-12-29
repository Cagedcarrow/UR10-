classdef Ur10TrajIntegrator < handle
    properties
        Robot, IK, Fig, Ax
        Inputs = struct() % 存储 UI 输入框句柄
        CurrentConfig = [0, -pi/2, pi/2, -pi/2, -pi/2, 0]
        BasePos = [0, 0, 1.4]
        ShovelLength = 0.25
        ShowWS = false
    end
    
    methods
        function obj = Ur10TrajIntegrator()
            % 1. 加载模型与求解器
            obj.Robot = loadrobot('universalUR10', 'DataFormat', 'row');
            
            % 挂载铲子
            shovelBody = rigidBody('shovel_link');
            shovelJoint = rigidBodyJoint('shovel_joint', 'fixed');
            setFixedTransform(shovelJoint, trvec2tform([0, 0, obj.ShovelLength]));
            shovelBody.Joint = shovelJoint;
            addVisual(shovelBody, "Box", [0.1 0.01 obj.ShovelLength], trvec2tform([0, 0, obj.ShovelLength/2]));
            addBody(obj.Robot, shovelBody, 'tool0');
            
            obj.IK = inverseKinematics('RigidBodyTree', obj.Robot);
            
            % 2. 启动界面
            obj.createGUI();
            obj.render();
        end
        
        function createGUI(obj)
            obj.Fig = figure('Name', 'UR10 轨迹集成研究工具', 'Color', [0.94 0.94 0.94], ...
                'Units', 'normalized', 'Position', [0.2 0.2 0.6 0.6]);
            
            % 3D 视图区
            obj.Ax = axes('Parent', obj.Fig, 'Units', 'normalized', 'Position', [0.35 0.1 0.6 0.85]);
            hold(obj.Ax, 'on'); grid(obj.Ax, 'on'); view(obj.Ax, 135, 30);
            axis(obj.Ax, [-2.5 2.5 -2.5 2.5 0 3]); axis(obj.Ax, 'equal');
            
            % 控制面板区
            panel = uipanel(obj.Fig, 'Title', '轨迹参数输入', 'Units', 'normalized', 'Position', [0.02 0.1 0.3 0.85]);
            
            % 起始坐标输入
            uicontrol(panel, 'Style', 'text', 'String', '--- 起始点坐标 (m) ---', 'Units', 'normalized', 'Position', [0.1 0.9 0.8 0.05]);
            obj.Inputs.sx = obj.createEdit(panel, 0.75, '0.8'); 
            obj.Inputs.sy = obj.createEdit(panel, 0.70, '0.0'); 
            obj.Inputs.sz = obj.createEdit(panel, 0.65, '0.5');
            
            % 终止坐标输入
            uicontrol(panel, 'Style', 'text', 'String', '--- 终止点坐标 (m) ---', 'Units', 'normalized', 'Position', [0.1 0.55 0.8 0.05]);
            obj.Inputs.ex = obj.createEdit(panel, 0.40, '1.4'); 
            obj.Inputs.ey = obj.createEdit(panel, 0.35, '0.0'); 
            obj.Inputs.ez = obj.createEdit(panel, 0.30, '1.8');
            
            % 功能按钮
            uicontrol(panel, 'Style', 'togglebutton', 'String', '显示工作空间包络', 'Units', 'normalized', ...
                'Position', [0.1 0.18 0.8 0.06], 'Callback', @(s,e) obj.toggleWS(s));
            
            uicontrol(panel, 'Style', 'pushbutton', 'String', '开始执行规划', 'Units', 'normalized', ...
                'Position', [0.1 0.05 0.8 0.1], 'BackgroundColor', [0 0.4 0.7], 'ForegroundColor', 'w', ...
                'FontSize', 12, 'FontWeight', 'bold', 'Callback', @(s,e) obj.startPlanning());
        end
        
        function editHandle = createEdit(obj, parent, y, defaultStr)
            uicontrol(parent, 'Style', 'text', 'String', 'X/Y/Z:', 'Units', 'normalized', 'Position', [0.05 y 0.2 0.04], 'HorizontalAlignment', 'right');
            editHandle = uicontrol(parent, 'Style', 'edit', 'String', defaultStr, 'Units', 'normalized', 'Position', [0.3 y 0.6 0.05], 'BackgroundColor', 'w');
        end
        
        function render(obj)
            if ~isgraphics(obj.Ax), return; end
            cla(obj.Ax); hold(obj.Ax, 'on');
            
            % 更新基座
            T_base = trvec2tform(obj.BasePos) * axang2tform([1 0 0 pi]);
            setFixedTransform(obj.Robot.getBody('base_link').Joint, T_base);
            
            % 绘制工作空间
            if obj.ShowWS
                maxReach = 1.3 + obj.ShovelLength;
                [sx, sy, sz] = sphere(30);
                surf(obj.Ax, sx*maxReach+obj.BasePos(1), sy*maxReach+obj.BasePos(2), sz*maxReach+obj.BasePos(3), ...
                    'FaceColor', 'c', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
            end
            
            % 绘制机器人
            show(obj.Robot, obj.CurrentConfig, 'Parent', obj.Ax, 'PreservePlot', false, 'FastUpdate', true);
            axis(obj.Ax, [-2.5 2.5 -2.5 2.5 0 3]);
            drawnow;
        end
        
        function toggleWS(obj, src)
            obj.ShowWS = get(src, 'Value');
            obj.render();
        end
        
        function startPlanning(obj)
            % 1. 获取输入数据
            sPos = [str2double(obj.Inputs.sx.String), str2double(obj.Inputs.sy.String), str2double(obj.Inputs.sz.String)];
            ePos = [str2double(obj.Inputs.ex.String), str2double(obj.Inputs.ey.String), str2double(obj.Inputs.ez.String)];
            
            if any(isnan([sPos, ePos])), errordlg('请输入有效的数字坐标！'); return; end
            
            % 2. 路径预检查
            fprintf('预检查起终点可达性...\n');
            weights = [0.2 0.2 0.2 1 1 1];
            [~, sInfo] = obj.IK('shovel_link', trvec2tform(sPos), weights, obj.CurrentConfig);
            [~, eInfo] = obj.IK('shovel_link', trvec2tform(ePos), weights, obj.CurrentConfig);
            
            if ~strcmp(sInfo.Status, 'success') || ~strcmp(eInfo.Status, 'success')
                errordlg('起始点或终点无法到达！请检查坐标是否在工作范围内。');
                return;
            end
            
            % 3. 执行插补规划
            steps = 50;
            traj = [linspace(sPos(1), ePos(1), steps)', linspace(sPos(2), ePos(2), steps)', linspace(sPos(3), ePos(3), steps)'];
            
            % 动画显示
            hold(obj.Ax, 'on');
            plot3(obj.Ax, traj(:,1), traj(:,2), traj(:,3), 'r--', 'LineWidth', 1.5);
            
            for i = 1:steps
                tform = trvec2tform(traj(i,:));
                [config, info] = obj.IK('shovel_link', tform, weights, obj.CurrentConfig);
                if strcmp(info.Status, 'success')
                    obj.CurrentConfig = config;
                    obj.render();
                    pause(0.02);
                else
                    fprintf('路径在第 %d 步中断。\n', i); break;
                end
            end
            msgbox('轨迹执行完成！', '规划成功');
        end
    end
end