% 输入参数：
%   robot -> 7自由度机器人对象，使用SerialLink类表示
%   q_start, q_end -> 1x7向量，表示配置空间中需要检测的直线路径段的起点和终点配置
%   link_radius -> 标量，机器人各连杆圆柱体的半径
%   sphere_centers -> Nx3矩阵，包含N个球形障碍物的中心坐标
%   sphere_radii -> Nx1向量，包含N个球形障碍物的半径 
%   cuboid_origin -> 立方体障碍物原点坐标 [x,y,z]
%   cuboid_ckg -> 立方体障碍物尺寸 [长,宽,高]
%   resolution -> 标量，路径离散化分辨率（可选参数，默认11）
% 输出参数：
%   in_collision -> 布尔值，true表示路径段存在碰撞

function in_collision = check_edge(robot, q_start, q_end, link_radius, sphere_centers, sphere_radii, cuboid_origin, cuboid_ckg, resolution)
    % 参数预处理：如果没有提供分辨率参数，设置默认值为11（在0-1区间生成11个采样点）
    if nargin < 9
        resolution = 11;  % 默认沿路径均匀采样11个点进行碰撞检测
    end
    
    % 生成路径离散化参数：在[0,1]区间生成resolution个等间距参数值
    ticks = linspace(0, 1, resolution)';  % 生成列向量，用于插值计算
    
    % 计算中间配置：通过线性插值生成从q_start到q_end的路径点
    % 原理：每个关节角度在起始配置和目标配置之间线性变化
    n = length(ticks);  % 获取采样点数量
    configs = repmat(q_start, n, 1) + ...              % 初始配置矩阵
             repmat(q_end - q_start, n, 1) .* ...      % 关节角度变化量
             repmat(ticks, 1, length(q_start));        % 时间参数矩阵
    
    % 初始化碰撞标志为false（默认无碰撞）
    in_collision = false;
    
    % 遍历所有中间配置进行碰撞检测
    for i = 1:n
        % 对每个中间配置调用check_collision进行详细碰撞检测
        if check_collision(robot, configs(i,:), link_radius, ...
                          sphere_centers, sphere_radii, ...
                          cuboid_origin, cuboid_ckg)
            % 如果检测到任意一点存在碰撞，立即终止检测
            in_collision = true;
            break  % 提前退出循环以节省计算时间
        end
    end
end

% 算法说明：
% 1. 路径离散化：将连续路径转换为离散配置点序列
%    - 通过线性插值生成resolution个中间配置
%    - 默认采样11个点（包含起点和终点）
% 
% 2. 碰撞检测策略：
%    - 顺序检查每个离散配置点的碰撞状态
%    - 采用"快速失败"机制：发现任一碰撞点立即返回结果
%
% 3. 参数选择建议：
%    - resolution值越大检测越精确，但计算时间增加
%    - 对于快速移动的障碍物，建议增加分辨率
%    - 典型取值范围：5（粗糙）~21（精细）
%
% 4. 碰撞检测范围：
%    - 同时考虑球形和立方体障碍物
%    - 检测机器人所有连杆与障碍物的干涉
%    - 基于圆柱体模型进行快速碰撞检测