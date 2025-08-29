%%% KUKA正逆运动学推导 + RRT* 关节空间轨迹规划与B样条平滑可视化
% 说明：
%  1) 本脚本基于 Peter Corke Robotics Toolbox（SerialLink/Link/fkine/plot/teach），运行前请确保已安装并在路径中。
%  2) 本脚本统一以“毫米(mm)”为长度单位、弧度为角度单位；若你的工具箱/模型默认使用米(m)，需自查单位一致性（此脚本不修改任何代码）。
%  3) 规划在关节空间进行：RRT* 搜索 q=[q1..q6]，碰撞检测在几何模型上完成（调用外部的 check_edge 内部逻辑由你已有版本决定）。
%  4) B样条仅对规划出的关节轨迹做平滑；请确保你有 b_spline(theta,t,k) 的实现（返回 q,qd,qdd）。
clc; clear; close all   % 清空命令窗口、清变量、关图窗

% -----------------------------
% 基础常量与UR5 DH参数（Modified DH） 
% -----------------------------
degtorad = pi/180;    % 角度→弧度换算因子

%% 连杆偏距 d (单位:mm)
% KUKA LBR iiwa 的关节偏距参数
d1 = 340;      % 基座到关节1的垂直距离
d2 = 0;        % 关节2偏距（无）
d3 = 0;        % 关节3偏距（无）
d4 = 400;      % 关节4偏距（腕部高度）
d5 = 0;        % 关节5偏距（无）
d6 = 126;      % 关节6偏距
d7 = 90;       % 关节7偏距（末端法兰）
d = [d1, d2, d3, d4, d5, d6, d7];

%% 连杆长度 a (单位:mm)
% KUKA LBR iiwa 的连杆长度参数
a1 = 0;        % 关节1到关节2的长度（无）
a2 = 0;        % 关节2到关节3的长度（无）
a3 = 400;      % 关节3到关节4的长度（前臂）
a4 = 0;        % 关节4到关节5的长度（无）
a5 = 0;        % 关节5到关节6的长度（无）
a6 = 0;        % 关节6到关节7的长度（无）
a7 = 0;        % 关节7到末端的长度（已包含在d7）
a = [a1, a2, a3, a4, a5, a6, a7];

%% 连杆扭转角 alpha (弧度)
% Modified DH 约定（绕X轴旋转）
alpha1 = 0 * degtorad;      % 关节1无扭转
alpha2 = -90 * degtorad;    % 关节2绕X轴旋转-90°
alpha3 = 90 * degtorad;     % 关节3绕X轴旋转+90°
alpha4 = 90 * degtorad;     % 关节4绕X轴旋转+90°
alpha5 = -90 * degtorad;    % 关节5绕X轴旋转-90°
alpha6 = -90 * degtorad;    % 关节6绕X轴旋转-90°
alpha7 = 90 * degtorad;     % 关节7绕X轴旋转+90°
alpha = [alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7];

% -----------------------------
% 建模：Modified DH + KUKA关节偏置 offset
% -----------------------------
L(1) = Link([0, d1, a1, alpha1], 'modified');
L(2) = Link([0, d2, a2, alpha2], 'modified');
L(2).offset = -pi/2;    % KUKA零位偏置：关节2初始-90°
L(3) = Link([0, d3, a3, alpha3], 'modified');
L(4) = Link([0, d4, a4, alpha4], 'modified');
L(4).offset = pi;       % KUKA零位偏置：关节4初始+180°
L(5) = Link([0, d5, a5, alpha5], 'modified');
L(6) = Link([0, d6, a6, alpha6], 'modified');
L(7) = Link([0, d7, a7, alpha7], 'modified');
L(7).offset = pi/2;     % KUKA零位偏置：关节7初始+90°

% 串联机器人
robot = SerialLink(L, 'name', 'KUKA LBR iiwa');

% -----------------------------
% 可视化：初始位姿与示教面板
% -----------------------------
figure(1)

view(3);                              % 三维视角
robot.plot([0, 0, 0, 0, 0, 0, 0], 'tilesize', 500)   % 初始关节全0（已考虑 offset）
robot.teach()                         % 打开交互式示教（可拖动关节）
hold on

%% 验证正逆运动学（保留占位，便于你自行添加测试用例）
% 例如：T = robot.fkine(q); q_ik = robot.ikine(T, ...)

%% 轨迹规划（关节空间）
% 起止关节配置（单位：弧度）
% q_start = [0, -pi/3, pi/6, pi/6, -pi/6, pi/6, 0];
% q_goal  = [pi, -5*pi/6, pi/3, 0, pi/6, pi/5, 0];
q_start = [0,0,0,0,0,0,0];
q_goal = [pi/6,pi/2,pi/2,pi/2,pi/2,pi/2,pi/2];
% -----------------------------
% 计算并绘制起止末端位姿（蓝点）
% -----------------------------
T_start = zeros(4, 4);
T_goal  = zeros(4, 4);
T_start(:, :) = robot.fkine(q_start);
T_goal(:, :)  = robot.fkine(q_goal);

figure(1)
plot3(T_goal(1, 4), T_goal(2, 4), T_goal(3, 4), '-o', 'Color', 'r', 'MarkerSize', 10, 'MarkerFaceColor', 'r')
hold on
plot3(T_start(1, 4), T_start(2, 4), T_start(3, 4), '-o', 'Color', 'b', 'MarkerSize', 10, 'MarkerFaceColor', 'b')
hold on
light('Position', [1 1 1], 'Style','infinite');   % 环境光源
lighting gouraud;
material([0.6 0.8 0.4]);                          % 表面材质参数

% Load sample vertex and face data for two convex polyhedra
LoveShape;

% Make shape 1
SO1.Vertices = VO1;
SO1.Faces = FO1;
SO1.FaceVertexCData = jet(size(VO1,1));
SO1.FaceColor = 'interp';
patch(SO1);

% Make shape 2
SO2.Vertices = VO2;
SO2.Faces = FO2;
SO2.FaceVertexCData = jet(size(VO2,1));
SO2.FaceColor = 'interp';
patch(SO2);
% 注：避障考虑了基座碰撞，不要将心形障碍直接置于原点处，否则基座的GJK判断始终为true
% -----------------------------
% 关节限制（规划边界，弧度）
% -----------------------------
% 注：这些限制用于采样与碰撞检测的“可行域”，不改变 SerialLink 内部限制。
q_min = deg2rad([-170, -120, -170, -120, -170, -120, -175]);  % 最小值
q_max = deg2rad([ 170,  120,  170,  120,  170,  120,  175]);  % 最大值

% -----------------------------
% 碰撞外形与环境障碍（单位：mm）
% -----------------------------
link_radius = 50; % 各连杆“包络圆柱”的半径，供碰撞检测使用（越大越保守）

% 球形障碍（每行一个球：中心[x y z]，下面半径列表对应）
sphere_center = [ -400,    0, -200; 
                     0, -800,   80;
                     0,  600,  800];
sphere_radius = [200, 200, 320];   % 半径（mm）

% 长方体障碍：origin为一角点，ckg为尺寸向量（宽/长/高）
% 说明：原注释写“乘以4缩放”，但代码实际乘以 2，这里已修正注释以与代码一致。
cuboid_origin = 2 * [
    -200,   100,   -250;
    130, -70, 250];
cuboid_ckg = 2 * [
    150, 100, 100;
    100,150,100];

% 绘制障碍
draw_sphere(sphere_center, sphere_radius, 3);
draw_cuboid(cuboid_origin, cuboid_ckg, 2);

% 视域范围（单位：mm）
xlim([-1200, 1200]);
ylim([-1200, 1200]);
zlim([-1200, 1200]);

%切换算法%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 使用 RRT* 进行关节空间路径规划
% 如果你希望重复实验时不每次都重新规划，可使用下方的 save/load。
% 这里默认执行一次规划并保存结果到 path.mat。
 % [path, path_found] = RRTStar(robot, q_min, q_max, q_start, q_goal, link_radius, ...
 %    sphere_center, sphere_radius, cuboid_origin, cuboid_ckg);
 [path, path_found] = BiRRTStar(robot, q_min, q_max, q_start, q_goal, link_radius, ...
    sphere_center, sphere_radius, cuboid_origin, cuboid_ckg);
save('path.mat', 'path');

% 复用缓存（按需启用）：
% load('path.mat');   % 加载保存的路径数据
% path_found = true;

% -----------------------------
% 路径可视化（末端轨迹 + B样条平滑）
% -----------------------------
if path_found
    % 注意：path.pos 是结构体数组，每个元素含字段 .q（1×6 关节向量）
    % 翻转第二维（保持与后续处理一致的存储顺序；沿用你原逻辑，不改代码）
    path.pos = flip(path.pos, 2);

    % 正运动学得到沿途末端位姿，用红线绘制原始RRT*轨迹
    T = zeros(4, 4, length(path.pos));
    for i = 1:length(path.pos)
        T(:, :, i) = robot.fkine(path.pos(i).q);
    end
    plot3(squeeze(T(1, 4, :)), squeeze(T(2, 4, :)), squeeze(T(3, 4, :)), 'r-', 'LineWidth', 2);
    hold on

    %%
    % % ---- B样条平滑（仅平滑关节轨迹，不改变起终点约束） ----
%     k = 5;      % 三次B样条
%     Pr = 50;    % 曲线采样精度（每段采样点数）
%     t = linspace(0, 1, Pr);   % 参数区间采样
%     % t_total 仅用于绘图横轴（非物理时间），按"节点数 + 边界复制 - 阶次"生成点数
%     t_total = linspace(0, length(path.pos) + 4 - k, 50 * (length(path.pos) + 4 - k));
% 
%     % 整理控制点矩阵：每列为一个关节配置
%     theta = [];
%     for i = 1:length(path.pos)
%         theta(:, i) = path.pos(i).q;
%     end
% 
%     % 收敛性微调：保留你的逻辑（若倒数第二个点 q1<0，则将末点 q1 置为 -pi）
%     if theta(1, end-1) < 0
%         theta(1, end) = -pi;
%     end
% 
%     % B样条端点复制（首尾各复制一次），保证端点处一阶连续更平滑
%     theta = [repmat(theta(:,1), 1, 2), theta, repmat(theta(:,end), 1, 2)];
% 
%     % 调用外部 b_spline(theta,t,k)：
%     % 要求返回：q（关节角矩阵，列为采样点），qd（角速度），qdd（角加速度）
%     [q, qd, qdd] = b_spline(theta, t, k); 
%     % 
%     % 平滑轨迹的末端位姿（绿色线）
%     T1 = zeros(4, 4, length(q));
%     for i = 1:length(q)
%         T1(:, :, i) = robot.fkine(q(:, i));
%     end
%     plot3(squeeze(T1(1, 4, :)), squeeze(T1(2, 4, :)), squeeze(T1(3, 4, :)), 'g-', 'LineWidth', 2);
%     hold on
% 
%     % 转置为每行一个采样时刻，便于绘图（不改任何计算）
%     q = q';
%     qd = qd';
%     qdd = qdd';
% 
%     % 逐帧播放关节轨迹（fps=5000 为快速播放；可调小便于观察）
%     robot.plot(q, 'fps', 500);
% 
%     % ---- 曲线绘制（与 t_total 对应） ----
%     figure(2)
%     subplot(2, 2, 1)
%     plot(t_total, q(:, 1:7) / degtorad);   % 转成"度”显示
%     title("关节角度");
%     legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7');
% 
%     subplot(2, 2, 2)
%     plot(t_total, qd(:, 1:7));
%     title("关节角速度");
%     legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7');
% 
%     subplot(2, 2, 3)
%     plot(t_total, qdd(:, 1:7));
%     title("关节角加速度");
%     legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7');
% 
% else
%     disp('未找到路径。');  % 规划失败提示

    %%分段多项式，minsnap
    % 整理控制点矩阵：每列为一个关节配置
    waythetas = [];
    for i = 1:length(path.pos)
        waythetas(:, i) = path.pos(i).q;
    end

    % 考虑动力学约束
    % max_torque = [320, 320, 176, 176, 110, 110, 110]; % 单位: Nm (LBR iiwa 7 R800)

    %初始保守估计时间
    v_max = deg2rad(90);     % 最大速度 (rad/s)
    a_max = deg2rad(180);    % 最大加速度 (rad/s²)
    t = trapezoidal_time(waythetas, v_max, a_max);
    n_order = 5;
    polys_q = minimum_snap_7axis(waythetas,t,n_order,0,0,0,0);
    q_opt_total = [];
    qd_opt_total = [];
    qdd_opt_total = [];
    t_total = [];
    for i=1:size(polys_q,2)
        tt = ceil(t(i)*100)/100:0.01:floor(t(i+1)*100)/100;
        t_total = [t_total, tt];
    end
    for k=1:size(polys_q,3)
        q_opt = [];
        qd_opt = [];
        qdd_opt = [];
        for i=1:size(polys_q,2)
            tt = ceil(t(i)*100)/100:0.01:floor(t(i+1)*100)/100;
            q_opt = [q_opt,polys_vals(polys_q(:,:,k),t,tt,0)];
            qd_opt = [qd_opt,polys_vals(polys_q(:,:,k),t,tt,1)];
            qdd_opt = [qdd_opt,polys_vals(polys_q(:,:,k),t,tt,2)];
        end
        q_opt_total = [q_opt_total;q_opt];
        qd_opt_total = [qd_opt_total;qd_opt];
        qdd_opt_total = [qdd_opt_total;qdd_opt];
    end
    save('tmat.mat','t_total')
    save('thetamat.mat', 'q_opt_total'); 
    save('dthetamat.mat', 'qd_opt_total');
    save('ddthetamat.mat', 'qdd_opt_total');
    % %导入逆动力学，以0.01s为间隔
    % ts = 0:0.01:t(end);
    % for k=1:size(polys_q,3)
    %     q_opt = [];
    %     qd_opt = [];
    %     qdd_opt = [];
    %     for i=1:size(polys_q,2)
    %         tt = t(i):0.01:t(i+1);
    %         q_opt = [q_opt,polys_vals(polys_q(:,:,k),t,tt,0)];
    %         qd_opt = [qd_opt,polys_vals(polys_q(:,:,k),t,tt,1)];
    %         qdd_opt = [qdd_opt,polys_vals(polys_q(:,:,k),t,tt,2)];
    %     end
    %     q_opt_total = [q_opt_total;q_opt];
    %     qd_opt_total = [qd_opt_total;qd_opt];
    %     qdd_opt_total = [qdd_opt_total;qdd_opt];
    % end

    % 平滑轨迹的末端位姿（绿色线）
    T1 = zeros(4, 4, size(q_opt_total,2));
    for i = 1:size(q_opt_total,2)
        T1(:, :, i) = robot.fkine(q_opt_total(:, i));
    end
    plot3(squeeze(T1(1, 4, :)), squeeze(T1(2, 4, :)), squeeze(T1(3, 4, :)), 'g-', 'LineWidth', 2);
    hold on

    % 逐帧播放关节轨迹（fps=5000 为快速播放；可调小便于观察）
    robot.plot(q_opt_total', 'fps', 5000);

    %% ---- 曲线绘制（与 t_total 对应） ----
    figure(2)
    subplot(2, 2, 1)
    plot(t_total, q_opt_total(1:7, :) / degtorad);   % 转成"度"显示
    title("关节角度");
    legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7');
    % qd_opt_total = zeros(7, size(q_opt_total,2));
    % qdd_opt_total = zeros(7, size(q_opt_total,2));
    % 
    % for i = 2: size(q_opt_total,2) - 1
    %     qd_opt_total(1:7, i) = (q_opt_total(1:7, i) - q_opt_total(1:7, i-1)) / (t_total(i) - t_total(i-1)) / 2;
    % end
    % for i = 2: size(q_opt_total,2) - 1
    %     qdd_opt_total(1:7, i) = (qd_opt_total(1:7, i) - qd_opt_total(1:7, i-1)) / (t_total(i) - t_total(i-1)) / 2;
    % end

    subplot(2, 2, 2)
    plot(t_total, qd_opt_total(1:7, :));
    title("关节角速度");
    legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7');

    subplot(2, 2, 3)
    plot(t_total, qdd_opt_total(1:7, :));
    title("关节角加速度");
    legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7');

else
    disp('未找到路径。');  % 规划失败提示
end


% ============================ 辅助绘制函数 ==============================
function draw_cuboid(originPoint,cuboidSize,num)
%% 长方体绘制（带半透明与高光）
% originPoint(i,:) 为第 i 个长方体的一角顶点坐标（mm）
% cuboidSize(i,:)  为第 i 个长方体的尺寸 [dx dy dz]（mm）
for i = 1:num
    % 8个角点（单位立方体索引）
    vertexIndex = [0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 1 0;1 1 1];
    vertex = originPoint(i,:) + vertexIndex.*cuboidSize(i,:);
    
    % 6个面（每行4个顶点索引）
    facet = [1 2 4 3;1 2 6 5;1 3 7 5;2 4 8 6;3 4 8 7;5 6 8 7];
    
    % 可视化参数
    patch('Vertices',vertex,'Faces',facet,...
          'FaceColor',[0 0.5 1],...
          'FaceAlpha',0.6,...
          'EdgeColor',[0.2 0.2 0.2],...
          'LineWidth',0.8,...
          'SpecularStrength',0.7);
end
end

function draw_sphere(position, radius, num)
%% 球体绘制（细分更高，半透明）
% position(i,:) 为球心坐标（mm），radius(i) 为半径（mm）
[sphereX, sphereY, sphereZ] = sphere(30); % 细分度（30相对平滑）
for i = 1:num
    X = sphereX * radius(i) + position(i,1);
    Y = sphereY * radius(i) + position(i,2);
    Z = sphereZ * radius(i) + position(i,3);
    
    surf(X,Y,Z,...
        'FaceColor',[0.4 0.8 0.6],...
        'EdgeColor','none',...
        'FaceAlpha',0.7,...
        'SpecularStrength',0.9,...
        'DiffuseStrength',0.5);
end
end


