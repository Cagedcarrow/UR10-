classdef Ur10SuccessDemo < handle
    properties
        Robot, IK, Fig, Ax
        Inputs = struct() 
        CurrentConfig = [0, -pi/2, pi/2, -pi/2, -pi/2, 0]
        BasePos = [0, 0, 1.4]  % 固定的合理挂高
        ShovelLength = 0.25
    end
    
    methods
        function obj = Ur10SuccessDemo()
            % 加载机器人并挂载铲子
            obj.Robot = loadrobot('universalUR10', 'DataFormat', 'row');
            shovel = rigidBody('shovel_link');
            jnt = rigidBodyJoint('shovel_joint', 'fixed');
            setFixedTransform(jnt, trvec2tform([0, 0, obj.ShovelLength]));
            shovel.Joint = jnt;
            addVisual(shovel, "Box", [0.1 0.01 obj.ShovelLength], trvec2tform([0, 0, obj.ShovelLength/2]));
            addBody(obj.Robot, shovel, 'tool0');
            
            % 初始化求解器
            obj.IK = inverseKinematics('RigidBodyTree', obj.Robot);
            
            % 创建界面
            obj.createGUI();
            obj.render();
        end
        
        function createGUI(obj)
            obj.Fig = figure('Name', 'UR10 可行性验证工具', 'Units', 'normalized', 'Position', [0.2 0.2 0.6 0.6], 'Color', 'w');
            obj.Ax = axes('Parent', obj.Fig, 'Position', [0.35 0.1 0.6 0.8]);
            hold(obj.Ax, 'on'); grid(obj.Ax, 'on'); view(obj.Ax, 135, 30);
            
            panel = uipanel(obj.Fig, 'Title', '轨迹参数 (单位: m)', 'Position', [0.02 0.1 0.3 0.8]);
            
            % 预设了一组绝对可行的坐标
            uicontrol(panel, 'Style', 'text', 'String', '起始点 (Start):', 'Units', 'normalized', 'Position', [0.1 0.85 0.8 0.05]);
            obj.Inputs.sx = obj.createIn(panel, 0.80, '0.7'); 
            obj.Inputs.sy = obj.createIn(panel, 0.75, '0.2'); 
            obj.Inputs.sz = obj.createIn(panel, 0.70, '0.4');
            
            uicontrol(panel, 'Style', 'text', 'String', '终止点 (End):', 'Units', 'normalized', 'Position', [0.1 0.55 0.8 0.05]);
            obj.Inputs.ex = obj.createIn(panel, 0.50, '0.7'); 
            obj.Inputs.ey = obj.createIn(panel, 0.45, '-0.4'); 
            obj.Inputs.ez = obj.createIn(panel, 0.40, '1.2');
            
            uicontrol(panel, 'Style', 'pushbutton', 'String', '▶ 执行平滑轨迹', 'Units', 'normalized', ...
                'Position', [0.1 0.1 0.8 0.15], 'BackgroundColor', [0.2 0.6 0.2], 'ForegroundColor', 'w', ...
                'FontSize', 12, 'FontWeight', 'bold', 'Callback', @(s,e) obj.execute());
        end
        
        function h = createIn(obj, p, y, def), h = uicontrol(p, 'Style', 'edit', 'String', def, 'Units', 'normalized', 'Position', [0.3 y 0.6 0.04]); end
        
        function render(obj)
            cla(obj.Ax);
            % 设置基座
            T_base = trvec2tform(obj.BasePos) * axang2tform([1 0 0 pi]);
            setFixedTransform(obj.Robot.getBody('base_link').Joint, T_base);
            
            % 绘制工作范围球体
            [sx, sy, sz] = sphere(20); radius = 1.3 + obj.ShovelLength;
            surf(obj.Ax, sx*radius+obj.BasePos(1), sy*radius+obj.BasePos(2), sz*radius+obj.BasePos(3), ...
                'FaceColor', 'c', 'EdgeColor', 'none', 'FaceAlpha', 0.05);
            
            show(obj.Robot, obj.CurrentConfig, 'Parent', obj.Ax, 'PreservePlot', false);
            axis(obj.Ax, [-2 2 -2 2 0 3]); axis equal;
        end
        
        function execute(obj)
            s = [str2double(obj.Inputs.sx.String), str2double(obj.Inputs.sy.String), str2double(obj.Inputs.sz.String)];
            e = [str2double(obj.Inputs.ex.String), str2double(obj.Inputs.ey.String), str2double(obj.Inputs.ez.String)];
            
            % 关键：使用“位置优先”权重。忽略微小的姿态偏差。
            % 权重格式：[Roll Pitch Yaw X Y Z]
            weights = [0.1 0.1 0.1 1 1 1]; 
            
            steps = 40;
            pathX = linspace(s(1), e(1), steps);
            pathY = linspace(s(2), e(2), steps);
            pathZ = linspace(s(3), e(3), steps);
            
            plot3(obj.Ax, pathX, pathY, pathZ, 'r:', 'LineWidth', 2);
            
            for i = 1:steps
                target = trvec2tform([pathX(i), pathY(i), pathZ(i)]) * axang2tform([0 1 0 0]);
                [q, info] = obj.IK('shovel_link', target, weights, obj.CurrentConfig);
                
                if strcmp(info.Status, 'success')
                    obj.CurrentConfig = q;
                    obj.render();
                    drawnow limitrate;
                else
                    title(obj.Ax, sprintf('中断！第 %d 步不可达', i), 'Color', 'r');
                    return;
                end
            end
            title(obj.Ax, '✅ 轨迹规划并执行成功！', 'Color', [0 0.5 0]);
        end
    end
end