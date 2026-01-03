classdef ScoopPathGenerator < handle
    methods
        function [waypoints, shovelAngles, tVec] = generateScoopMotion(obj, bucketPos, radius, depth)
            % 强制增大虚拟半径，使轨迹更平滑
            effRadius = radius * 1.5; 
            
            fprintf('生成平滑铲泥轨迹 (虚拟半径: %.2fm)...\n', effRadius);
            
            % 关键高度
            hoverZ = bucketPos(3) + 0.3;      % 准备高度 (抬高点)
            bottomZ = bucketPos(3) - depth + 0.1; % 铲泥最低点
            
            % 关键点定义
            % P1: 桶上方准备
            p1 = bucketPos + [0, 0, 0.3];
            
            % P2: 入水点 (向后退一点，留出冲刺距离)
            direction = [-1, 0, 0]; % 简化的局部方向
            p2 = bucketPos + [-effRadius*0.8, 0, 0.15]; 
            
            % P3: 桶底最深处 (圆弧底)
            p3 = bucketPos + [0, 0, -depth + 0.15];
            
            % P4: 出水点 (向前上方)
            p4 = bucketPos + [effRadius*0.6, 0, 0.1];
            
            % P5: 提拉结束
            p5 = p4 + [0, 0, 0.4];
            
            % --- 插值 (增加点数使轨迹更密更平滑) ---
            N1 = 30; N2 = 50; N3 = 30;
            
            % 1. 下潜 (直线)
            traj1 = [linspace(p2(1), p3(1), N1)', linspace(p2(2), p3(2), N1)', linspace(p2(3), p3(3), N1)'];
            ang1  = linspace(-deg2rad(80), -deg2rad(60), N1)'; 
            
            % 2. 兜底 (贝塞尔平滑曲线)
            t = linspace(0, 1, N2)';
            % 控制点：让曲线底部更圆润
            ctrlPt = p3 + [0, 0, -0.1]; 
            traj2 = (1-t).^2.*p3 + 2*(1-t).*t.*ctrlPt + t.^2.*p4;
            % 铲子角度：翻腕 (从 -60 到 0度水平)
            ang2 = linspace(-deg2rad(60), 0, N2)';
            
            % 3. 提拉 (直线)
            traj3 = [linspace(p4(1), p5(1), N3)', linspace(p4(2), p5(2), N3)', linspace(p4(3), p5(3), N3)'];
            ang3  = zeros(N3, 1);
            
            % 合并
            waypoints = [traj1; traj2; traj3];
            shovelAngles = [ang1; ang2; ang3];
            tVec = 1:size(waypoints,1);
        end
    end
end