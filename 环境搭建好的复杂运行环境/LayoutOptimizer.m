classdef LayoutOptimizer < handle
    properties
        RobotModel
    end
    
    methods
        function obj = LayoutOptimizer(robot)
            obj.RobotModel = robot;
        end
        
        function [bestBase, bestBucket] = findOptimalLayout(obj, wallX, wallY, wallH)
            fprintf('--- 以墙为基准进行布局优化 (带可达性强制约束) ---\n');
            
            % 1. 搜索参数：增加距离范围，并尝试不同的挂高 Z
            dist_range = 0.6 : 0.05 : 1.1; 
            offsetY_range = -0.4 : 0.1 : 0.4;
            height_range = 1.6 : 0.2 : 2.0; % 尝试不同的龙门架高度
            
            maxW = -1; % 初始化为负，确保能识别出成功的布局
            bestBase = [wallX - 0.8, wallY, 1.8]; 
            
            % 需要强制到达的关键点 [X, Y, Z]
            plasterPoints = [
                wallX, wallY, 0.8;
                wallX, wallY, 1.8;
                wallX, wallY, 0.4   % 模拟桶的深度，确保基座能向下伸够
            ];
            
            % 2. 迭代搜索
            for hz = height_range
                for d = dist_range
                    for oy = offsetY_range
                        candidateBase = [wallX - d, wallY + oy, hz]; 
                        
                        % calculateAverageW 内部现在会判断是否所有点都可达
                        currentW = obj.calculateAverageW(candidateBase, plasterPoints);
                        
                        if currentW > maxW
                            maxW = currentW;
                            bestBase = candidateBase;
                        end
                    end
                end
            end
            
            if maxW <= 0
                warning('未找到 100%% 可达的布局，请手动调低 Base Z 或移近墙壁！');
            end
            
            % 3. 桶位置：放在基座正下方偏向墙的一侧
            bestBucket = [bestBase(1) + 0.3, bestBase(2)];
            fprintf('建议布局 -> 离墙: %.2fm, 挂高: %.2fm, 可操作度: %.4f\n', ...
                abs(wallX - bestBase(1)), bestBase(3), maxW);
        end
    end
    
    methods (Access = private)
        function avgW = calculateAverageW(obj, basePos, points)
            % 更新基座
            baseBody = getBody(obj.RobotModel, 'base_link');
            setFixedTransform(baseBody.Joint, trvec2tform(basePos) * axang2tform([1 0 0 pi]));
            
            ik = inverseKinematics('RigidBodyTree', obj.RobotModel);
            totalW = 0; 
            
            for i = 1:size(points, 1)
                tform = trvec2tform(points(i,:)) * eul2tform([0, pi/6, 0], 'ZYX');
                [config, solInfo] = ik('shovel_link', tform, [1 1 1 1 1 1], obj.RobotModel.homeConfiguration);
                
                % 只要有一个点不可达，该布局直接废弃
                if ~strcmp(solInfo.Status, 'success')
                    avgW = -1; return; 
                end
                
                jac = geometricJacobian(obj.RobotModel, config, 'shovel_link');
                totalW = totalW + sqrt(det(jac * jac'));
            end
            avgW = totalW / size(points, 1);
        end
    end
end