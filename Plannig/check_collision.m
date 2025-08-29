% 输入参数：
%   robot -> 4自由度串行连杆机器人对象(SerialLink类实例)
%   q -> 1x4向量，需要检测的关节空间配置
%   link_radius -> 标量，机器人连杆圆柱体的半径
%   sphere_centers -> Nx3矩阵，N个球形障碍物的中心坐标[x,y,z]
%   sphere_radii -> Nx1向量，N个球形障碍物的半径 
%   cuboid_origin -> Mx3矩阵，M个立方体障碍物的原点坐标[x,y,z]
%   cuboid_ckg -> Mx3矩阵，M个立方体的尺寸[长,宽,高]
%   resolution -> 离散化采样点数（可选参数，默认11）
% 输出参数：
%   in_collision -> 布尔值，true表示当前配置存在碰撞

function in_collision = check_collision(robot, q, link_radius, sphere_centers, sphere_radii, cuboid_origin, cuboid_ckg, resolution)
    % 获取各关节坐标系原点（机械臂连杆端点）
    % 注意：虽然机器人是4自由度，但这里计算到第7个坐标系是为了包含末端执行器
    x1 = [0 0 0]';  % 基座坐标系原点（第1关节）
    
    % 计算第2关节位置：应用第1关节变换矩阵
    T2 = robot.A(1,q);  % 第1关节到第2关节的齐次变换矩阵
    x2 = T2.t;          % 提取平移向量作为第2关节坐标
    
    % 计算第3关节位置：应用第1-2关节变换矩阵
    T3 = robot.A(1:2,q);  % 累积变换矩阵
    x3 = T3.t;            % 第3关节坐标
    
    % 计算第4关节位置：应用第1-3关节变换矩阵
    T4 = robot.A(1:3,q);
    x4 = T4.t;            % 第4关节坐标
    
    % 后续关节计算（虽然机器人只有4自由度，但代码保留结构扩展性）
    T5 = robot.A(1:4,q);
    x5 = T5.t;            % 第5关节坐标（实际与T4相同）
    
    T6 = robot.A(1:5,q);
    x6 = T6.t;            % 第6关节坐标
    
    T7 = robot.A(1:6,q);
    x7 = T7.t;            % 第7关节坐标

    T8 = robot.A(1:7,q);
    x8 = T8.t;            % 第7关节坐标

    % 参数预处理：设置默认分辨率
    if nargin < 8
        resolution = 11;  % 每段连杆采样11个点
    end
    
    % 生成离散化参数：在[0,1]区间生成线性插值系数
    ticks = linspace(0, 1, resolution);  % 生成1x11插值系数向量
    
    % 离散化各连杆线段（将每段连杆分解为多个采样点）
    n = length(ticks);
    % 关节1到2之间的采样点：x1 + t*(x2-x1), t∈[0,1]
    x12 = repmat(x1, 1, n) + repmat(x2 - x1, 1, n) .* repmat(ticks, 3, 1);
    % 关节2到3之间的采样点
    x23 = repmat(x2, 1, n) + repmat(x3 - x2, 1, n) .* repmat(ticks, 3, 1);
    % 关节3到4之间的采样点
    x34 = repmat(x3, 1, n) + repmat(x4 - x3, 1, n) .* repmat(ticks, 3, 1);
    % 后续关节采样点（保持结构完整性）
    x45 = repmat(x4, 1, n) + repmat(x5 - x4, 1, n) .* repmat(ticks, 3, 1);
    x56 = repmat(x5, 1, n) + repmat(x6 - x5, 1, n) .* repmat(ticks, 3, 1);
    x67 = repmat(x6, 1, n) + repmat(x7 - x6, 1, n) .* repmat(ticks, 3, 1);
    x78 = repmat(x7, 1, n) + repmat(x8 - x7, 1, n) .* repmat(ticks, 3, 1);

    % 合并所有采样点（每列是一个三维点）
    points = [x12 x23 x34 x45 x56 x67 x78];  % 3x(6*resolution)矩阵

    % 初始化碰撞标志
    in_collision = false;
    
    % ================== 球形障碍物碰撞检测 ==================
    for i = 1:size(sphere_centers, 1)
        % 计算所有采样点到当前球心的平方距离
        dist_sq = sum((points - repmat(sphere_centers(i,:)', 1, size(points, 2))).^2, 1);
        
        % 检测碰撞条件：存在至少一个点满足 距离平方 < (半径和)^2
        if any(dist_sq < (link_radius + sphere_radii(i))^2)
            in_collision = true;
            break;  % 检测到碰撞立即退出循环
        end
    end
    
    % ================== 立方体障碍物碰撞检测（仅当未检测到球体碰撞时执行） ==================
    if(~in_collision)
        for i = 1:size(cuboid_origin, 1)
            % 提取当前立方体参数
            ox = cuboid_origin(i,1);  % 原点x坐标
            oy = cuboid_origin(i,2);  % 原点y坐标
            oz = cuboid_origin(i,3);  % 原点z坐标
            l = cuboid_ckg(i,1);      % 长度（x轴方向）
            w = cuboid_ckg(i,2);      % 宽度（y轴方向）
            h = cuboid_ckg(i,3);      % 高度（z轴方向）
            
            % 构建立方体区域判断条件（轴对齐立方体AABB）
            % 判断采样点是否在立方体内（考虑连杆半径膨胀）
            in_x = points(1,:) > (ox - link_radius) & points(1,:) < (ox + l + link_radius);
            in_y = points(2,:) > (oy - link_radius) & points(2,:) < (oy + w + link_radius);
            in_z = points(3,:) > (oz - link_radius) & points(3,:) < (oz + h + link_radius);
            
            % 综合三个轴向判断结果
            if any(in_x & in_y & in_z)
                in_collision = true;
                break;
            end
        end    
    end

    % ================== 不规则障碍物碰撞检测 ==================
    if(~in_collision)
        %正12边棱柱代替圆柱
        [v1, f1] = generate_prism(x1, x2, link_radius, 12);
        [v2, f2] = generate_prism(x2, x3, link_radius, 12);
        [v3, f3] = generate_prism(x3, x4, link_radius, 12);
        [v4, f4] = generate_prism(x4, x5, link_radius, 12);
        [v5, f5] = generate_prism(x5, x6, link_radius, 12);
        [v6, f6] = generate_prism(x6, x7, link_radius, 12);
        [v7, f7] = generate_prism(x7, x8, link_radius, 12);

        % num_vertices = size(v1, 1);
        % colors = jet(num_vertices); % 使用 jet 颜色映射
        % 创建隐藏的图形窗口
        fig = figure('Visible', 'off'); 
        % 使用 patch 绘制
        S1 = patch('Vertices', v1, 'Faces', f1, 'Visible', 'off');
        S2 = patch('Vertices', v2, 'Faces', f2, 'Visible', 'off');
        S3 = patch('Vertices', v3, 'Faces', f3, 'Visible', 'off');
        S4 = patch('Vertices', v4, 'Faces', f4, 'Visible', 'off');
        S5 = patch('Vertices', v5, 'Faces', f5, 'Visible', 'off');
        S6 = patch('Vertices', v6, 'Faces', f6, 'Visible', 'off');
        S7 = patch('Vertices', v7, 'Faces', f7, 'Visible', 'off');
        %How many iterations to allow for collision detection.
        iterationsAllowed = 6;
        
        % Load sample vertex and face data for two convex polyhedra
        LoveShape;
        
        % Make shape 1
        SO1.Vertices = VO1;
        SO1.Faces = FO1;
        SO1.FaceVertexCData = jet(size(VO1,1));
        SO1.FaceColor = 'interp';
        S1Obj = patch(SO1,'Visible', 'off');
        
        % Make shape 2
        SO2.Vertices = VO2;
        SO2.Faces = FO2;
        SO2.FaceVertexCData = jet(size(VO2,1));
        SO2.FaceColor = 'interp';
        S2Obj = patch(SO2,'Visible', 'off');
        
        collision_results = [GJK(S1,S1Obj,iterationsAllowed),... 
                             GJK(S2,S1Obj,iterationsAllowed),...
                             GJK(S3,S1Obj,iterationsAllowed),...
                             GJK(S4,S1Obj,iterationsAllowed),...
                             GJK(S5,S1Obj,iterationsAllowed),...
                             GJK(S6,S1Obj,iterationsAllowed),...
                             GJK(S7,S1Obj,iterationsAllowed),...
                             GJK(S1,S2Obj,iterationsAllowed),...
                             GJK(S2,S2Obj,iterationsAllowed),...
                             GJK(S3,S2Obj,iterationsAllowed),...
                             GJK(S4,S2Obj,iterationsAllowed),...
                             GJK(S5,S2Obj,iterationsAllowed),...
                             GJK(S6,S2Obj,iterationsAllowed),...
                             GJK(S7,S2Obj,iterationsAllowed)];
        in_collision = any(collision_results);
        % 关闭隐藏窗口（可选）
        close(fig);
    end
end

% 算法说明：
% 1. 连杆离散化建模：
%    - 将每段连杆分解为多个采样点（沿直线均匀分布）
%    - 通过检查这些采样点与障碍物的关系近似检测整个连杆的碰撞
%
% 2. 碰撞检测原理：
%    - 对球形障碍物：计算采样点到球心的距离，考虑连杆半径膨胀
%    - 对立方体障碍物：检测采样点是否在膨胀后的立方体AABB内
%
% 3. 参数选择建议：
%    - resolution值越大检测精度越高，但计算量增加
%    - 典型值：简单场景用5-10，复杂场景用15-20
%    - 连杆半径应考虑安全余量（实际半径的1.2-1.5倍）
%
% 4. 优化提示：
%    - 可预先计算所有障碍物的膨胀尺寸，减少实时计算量
%    - 对长连杆可采用自适应采样策略（端点附近密集采样）
%    - 使用并行运算加速矩阵操作（如用GPU加速）