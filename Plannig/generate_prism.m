function [vertices, faces] = generate_prism(x1, x2, link_radius, num_sides)
    % 输入：
    %   x1: 圆柱底面圆心坐标 [x, y, z]
    %   x2: 圆柱顶面圆心坐标 [x, y, z]
    %   link_radius: 圆柱半径
    %   num_sides: 棱柱的边数（默认12）
    %
    % 输出：
    %   vertices: 棱柱的所有顶点坐标（N×3矩阵）
    %   faces: 棱柱的面定义（用于patch绘图）

    if nargin < 4
        num_sides = 12; % 默认12条边
    end

    % 1. 计算圆柱的轴向向量
    axis_vec = x2 - x1;
    height = norm(axis_vec);
    axis_unit = axis_vec / height;

    % 2. 计算垂直于轴向的局部坐标系（用于生成圆环点）
    % 取一个非平行于轴向的向量（如[1,0,0]或[0,1,0]）
    if abs(axis_unit(1)) < 0.9
        perp_vec = cross(axis_unit, [1; 0; 0]);
    else
        perp_vec = cross(axis_unit, [0; 1; 0]);
    end
    perp_vec = perp_vec / norm(perp_vec);

    % 3. 生成12个等角度分布的圆环点（底面和顶面）
    theta = linspace(0, 2*pi, num_sides + 1);
    theta(end) = []; % 去掉重复的2pi点

    % 底面顶点（基于x1）
    bottom_vertices = zeros(num_sides, 3);
    for i = 1:num_sides
        % 在垂直于轴向的平面上计算点的偏移
        offset = link_radius * (cos(theta(i)) * perp_vec + sin(theta(i)) * cross(axis_unit, perp_vec));
        bottom_vertices(i, :) = x1 + offset;
    end

    % 顶面顶点（基于x2）
    top_vertices = zeros(num_sides, 3);
    for i = 1:num_sides
        offset = link_radius * (cos(theta(i)) * perp_vec + sin(theta(i)) * cross(axis_unit, perp_vec));
        top_vertices(i, :) = x2 + offset;
    end

    % 4. 组合所有顶点
    vertices = [bottom_vertices; top_vertices];

    % 5. 定义棱柱的面（Faces）
    % (a) 底面（1~num_sides）
    bottom_face = 1:num_sides;

    % (b) 顶面（num_sides+1 ~ 2*num_sides）
    top_face = (num_sides+1):(2*num_sides);

    % (c) 侧面（四边形）
    side_faces = nan(num_sides, num_sides);
    for i = 1:num_sides
        next_i = mod(i, num_sides) + 1;
        side_faces(i, 1:4) = [i, next_i, next_i + num_sides, i + num_sides];
    end
    
    % 组合所有面
    faces = [
        bottom_face;       % 底面（多边形）
        top_face;          % 顶面（多边形）
        side_faces         % 侧面（四边形）
    ];
end

% %调用
% x1 = [0, 0, 0];      % 底面圆心
% x2 = [1, 1, 1];      % 顶面圆心
% link_radius = 0.5;   % 圆柱半径
% 
% [vertices, faces] = generate_prism(x1, x2, link_radius, 12);
% num_vertices = size(vertices, 1);
% colors = jet(num_vertices); % 使用 jet 颜色映射
% % 使用 patch 绘制
% figure;
% S = patch('Vertices', vertices, 'Faces', faces, ...
%           'FaceVertexCData', colors, 'FaceColor', 'interp', 'EdgeColor', 'k');
% axis equal;
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title('12边棱柱（圆柱近似）');