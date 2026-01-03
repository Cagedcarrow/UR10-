classdef ScoopRobot7DOF < handle
    properties
        Robot       % rigidBodyTree 对象
        HomeConfig  % 初始关节角 (1x7)
        ShovelLen   % 铲子摆臂长度 (电机轴中心 -> 铲尖)
        MountLen    % 垂直延长杆长度 (法兰盘 -> 电机轴中心)
        BaseHeight = 1.4; % 龙门架挂高
    end
    
    methods
        function obj = ScoopRobot7DOF(customShovelLen)
            % 构造函数
            if nargin < 1
                obj.ShovelLen = 0.35; % 默认铲长 (橙色杆+铲斗)
            else
                obj.ShovelLen = customShovelLen;
            end
            
            % 垂直杆长度
            obj.MountLen = 0.1; 
            
            % 1. 加载基础 UR10 模型
            obj.Robot = loadrobot('universalUR10', 'DataFormat', 'row');
            
            % 2. 修正基座位置 (倒挂逻辑)
            baseBody = obj.Robot.getBody('base_link');
            T_base = trvec2tform([0, 0, obj.BaseHeight]) * axang2tform([1 0 0 pi]);
            setFixedTransform(baseBody.Joint, T_base);
            
            % --- 3. 构建垂直安装支架 (Mount Link) ---
            % 这是一个刚性固定的延长杆
            mountBody = rigidBody('mount_link');
            jntMount = rigidBodyJoint('mount_joint', 'fixed');
            setFixedTransform(jntMount, trvec2tform([0,0,0])); % 紧贴 tool0
            mountBody.Joint = jntMount;
            
            % [视觉 A]: 法兰连接盘 (模拟图中的圆形法兰)
            addVisual(mountBody, 'Cylinder', [0.04 0.01], trvec2tform([0 0 0.005]));
            
            % [视觉 B]: 垂直刚性杆 (白色铝型材)
            % 几何中心设在 MountLen/2 处
            addVisual(mountBody, 'Box', [0.04 0.04 obj.MountLen], ...
                trvec2tform([0 0 obj.MountLen/2]));
            
            % [视觉 C]: 电机座 (U型架，位于杆子最底端)
            addVisual(mountBody, 'Box', [0.08 0.05 0.05], ...
                trvec2tform([0 0 obj.MountLen]));
            
            addBody(obj.Robot, mountBody, 'tool0');
            
            % --- 4. 构建铲子 (Shovel Link) ---
            % 关键点：旋转关节位于垂直杆的末端
            shovelBody = rigidBody('shovel_link');
            jntShovel = rigidBodyJoint('shovel_joint', 'revolute');
            jntShovel.JointAxis = [1 0 0]; % 绕?轴旋转
            
            % 铲子摆动范围 (根据图纸结构，应该不能向后折太多)
            jntShovel.PositionLimits = deg2rad([-45, 135]); 
            
            % 关节原点：垂直杆的末端 (Z = MountLen)
            setFixedTransform(jntShovel, trvec2tform([0, 0, obj.MountLen]));
            shovelBody.Joint = jntShovel;
            
            % [视觉 D]: 铲柄 (橙色锥体+杆)
            % 假设铲子自然状态下是沿着 Z 轴继续延伸的，或者是斜向伸出的
            % 这里我们模拟它有一个初始的安装角度，比如 30度
            % 但为了运动学求解简单，我们通常建模为直的，通过设置 HomeConfig 来表现倾斜
            
            addVisual(shovelBody, 'Cylinder', [0.02, obj.ShovelLen], ...
                trvec2tform([0, 0, obj.ShovelLen/2]) * axang2tform([1 0 0 -pi/2]));
            
            % [视觉 E]: 铲斗 (末端平板)
            tipTrans = trvec2tform([0, 0, obj.ShovelLen]) * axang2tform([0 1 0 pi/2]);
            addVisual(shovelBody, 'Box', [0.15, 0.005, 0.1], tipTrans);
            
            addBody(obj.Robot, shovelBody, 'mount_link');
            
            % --- 5. 定义 TCP (工具中心点) ---
            tcpBody = rigidBody('tcp_frame');
            tcpJoint = rigidBodyJoint('tcp_joint', 'fixed');
            setFixedTransform(tcpJoint, trvec2tform([0, 0, obj.ShovelLen])); 
            tcpBody.Joint = tcpJoint;
            addBody(obj.Robot, tcpBody, 'shovel_link');
            
            % --- 6. 智能初始姿态 ---
            % 让铲子稍微翘起一点，符合图纸中的自然下垂状态
            % [Base, Shoulder, Elbow, W1, W2, W3, Shovel]
            targetYaw = atan2(0.4, 0.6); 
            obj.HomeConfig = [targetYaw, -pi/3, -2*pi/3, -pi/2, pi/2, 0, deg2rad(30)];
        end
    end
end