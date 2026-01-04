classdef ScoopRobot7DOF < handle
    properties
        Robot, HomeConfig, ShovelLen, MountLen, BaseHeight = 1.4; 
    end
    methods
        function obj = ScoopRobot7DOF(customShovelLen)
            if nargin < 1, obj.ShovelLen = 0.35; else, obj.ShovelLen = customShovelLen; end
            obj.MountLen = 0.15;
            obj.Robot = loadrobot('universalUR10', 'DataFormat', 'row');
            
            % 倒挂基座
            baseBody = obj.Robot.getBody('base_link');
            setFixedTransform(baseBody.Joint, trvec2tform([0, 0, obj.BaseHeight]) * axang2tform([1 0 0 pi]));
            
            % 垂直连接杆 (Mount Link)
            mountBody = rigidBody('mount_link');
            addVisual(mountBody, 'Box', [0.04 0.04 obj.MountLen], trvec2tform([0 0 obj.MountLen/2]));
            addVisual(mountBody, 'Box', [0.08 0.06 0.06], trvec2tform([0 0 obj.MountLen])); % 电机座
            addBody(obj.Robot, mountBody, 'tool0');
            
            % 铲子关节 - 【修正为 Y 轴旋转，实现真正的俯仰】
            shovelBody = rigidBody('shovel_link');
            jntShovel = rigidBodyJoint('shovel_joint', 'revolute');
            jntShovel.JointAxis = [0 1 0]; 
            setFixedTransform(jntShovel, trvec2tform([0, 0, obj.MountLen]));
            shovelBody.Joint = jntShovel;
            
            % 铲斗视觉：0度时水平指向 X 正方向
            visTform = axang2tform([0 1 0 pi/2]) * trvec2tform([0, 0, obj.ShovelLen/2]); 
            addVisual(shovelBody, 'Cylinder', [0.02, obj.ShovelLen], visTform);
            addVisual(shovelBody, 'Box', [0.15, 0.005, 0.2], trvec2tform([obj.ShovelLen, 0, 0])); 
            addBody(obj.Robot, shovelBody, 'mount_link');
            
            % TCP 设在铲尖
            tcpBody = rigidBody('tcp_frame');
            setFixedTransform(tcpBody.Joint, trvec2tform([obj.ShovelLen, 0, 0])); 
            addBody(obj.Robot, tcpBody, 'shovel_link');
            
            % 预瞄初始姿态
            obj.HomeConfig = [atan2(0.4, 0.6), -pi/3, -2*pi/3, -pi/2, pi/2, 0, -pi/2];
        end
    end
end