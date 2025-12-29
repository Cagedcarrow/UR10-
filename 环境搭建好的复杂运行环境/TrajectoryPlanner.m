classdef TrajectoryPlanner < handle
    properties
        Controller      % 引用 Ur10Controller 实例
        RobotModel      % 机器人模型
        IK              % 逆运动学求解器
    end
    
    methods
        function obj = TrajectoryPlanner(controllerInstance)
            obj.Controller = controllerInstance;
            obj.RobotModel = controllerInstance.Robot;
            obj.IK = inverseKinematics('RigidBodyTree', obj.RobotModel);
        end
        
        function [jointTraj, tVec] = planFullTask(obj)
            fprintf('开始规划：取泥 -> 抹泥 完整路径...\n');
            
            % 1. 环境参数提取
            bX = obj.Controller.BucketPos(1); bY = obj.Controller.BucketPos(2);
            wX = obj.Controller.WallParams(1); wY = obj.Controller.WallParams(2);
            
            % 2. 定义笛卡尔关键点序列 (X, Y, Z, Roll, Pitch, Yaw)
            % 路径：当前位 -> 桶上方 -> 桶内 -> 桶上方 -> 墙起点 -> 墙终点
            wp_cart = [
                obj.getCurrentTCP(), 0, 0, 0;                         % 0: 初始位
                bX, bY, 0.8,         0, 0, 0;                         % 1: 桶上方
                bX, bY, 0.35,        0, 0, 0;                         % 2: 桶内 (取泥点)
                bX, bY, 0.8,         0, 0, 0;                         % 3: 提起来
                wX-0.12, wY, 0.8,    0, obj.Controller.ShovelAttitude(2), 0; % 4: 墙起始点
                wX-0.12, wY, 1.8,    0, obj.Controller.ShovelAttitude(2), 0  % 5: 墙终点
            ];
            
            % 3. 逐点求解逆运动学 (IK)
            numWp = size(wp_cart, 1);
            jointWaypoints = zeros(numWp, 6);
            initialGuess = obj.Controller.CurrentConfig;
            
            % 设置权重：[Roll Pitch Yaw X Y Z]
            % 默认权重尽量保持姿态精度
            standardWeights = [1 1 1 1 1 1];
            % 宽松权重：当标准求解失败时，优先保证位置(X Y Z)准确
            relaxedWeights = [0.1 0.1 0.1 1 1 1];
            
            for i = 1:numWp
                tform = trvec2tform(wp_cart(i, 1:3)) * eul2tform(fliplr(wp_cart(i, 4:6)), 'ZYX');
                
                % 第一次尝试：标准求解
                [config, solInfo] = obj.IK('shovel_link', tform, standardWeights, initialGuess);
                
                % 第二次尝试：如果失败，放宽姿态约束
                if ~strcmp(solInfo.Status, 'success')
                    [config, solInfo] = obj.IK('shovel_link', tform, relaxedWeights, initialGuess);
                end
                
                if strcmp(solInfo.Status, 'success')
                    jointWaypoints(i, :) = config;
                    initialGuess = config; % 更新猜测值，保证轨迹连续性
                else
                    error('关键点 %d (目标坐标: [%.2f, %.2f, %.2f]) 求解失败！\n提示：该位置超出机械臂航程，请降低基座高度或移近物体。', ...
                        i, wp_cart(i,1), wp_cart(i,2), wp_cart(i,3));
                end
            end
            
            % 4. 多项式插值生成平滑轨迹
            timePoints = [0, 2, 3, 4, 7, 10]; 
            tVec = 0:0.05:10; 
            jointTraj = cubicpolytraj(jointWaypoints', timePoints, tVec)';
        end
        
        function pos = getCurrentTCP(obj)
            tform = getTransform(obj.RobotModel, obj.Controller.CurrentConfig, 'shovel_link');
            pos = tform(1:3, 4)';
        end
        
        function visualizeTraj(obj, jointTraj)
            fprintf('正在播放作业动画...\n');
            for i = 1:size(jointTraj, 1)
                obj.Controller.CurrentConfig = jointTraj(i, :);
                % 实时更新 GUI 界面
                obj.Controller.render();
                pause(0.01);
            end
            fprintf('作业流程演示完毕。\n');
        end
    end
end