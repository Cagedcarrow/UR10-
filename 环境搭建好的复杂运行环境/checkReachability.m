function [isPossible, info] = checkReachability(robot, targetPos, targetRPY, currentConfig)
    % 功能：判断给定位置和姿态，UR10 是否能够到达
    % 输入：robot (模型), targetPos [x y z], targetRPY [r p y], currentConfig (初始参考位姿)
    
    persistent ik weights
    if isempty(ik)
        ik = inverseKinematics('RigidBodyTree', robot);
        weights = [1 1 1 1 1 1]; % 姿态和位置权重均等
    end
    
    % 构建目标变换矩阵
    % 注意：建筑作业中攻角(Pitch)和偏航(Yaw)很重要
    tform = trvec2tform(targetPos) * eul2tform(fliplr(targetRPY), 'ZYX');
    
    % 尝试求解 IK
    [configSol, solInfo] = ik('shovel_link', tform, weights, currentConfig);
    
    % 返回结果
    if strcmp(solInfo.Status, 'success')
        isPossible = true;
        info = '可到达';
    else
        isPossible = false;
        info = solInfo.Status; % 返回具体失败原因，如 'IterationLimitExceeded'
    end
end