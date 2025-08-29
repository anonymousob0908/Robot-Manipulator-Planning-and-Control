% 双向 RRT*（BiRRT*）— 含目标偏置与重连
function [path, path_found] = BiRRTStar(robot, q_min, q_max, q_start, q_goal, link_radius, ...
    sphere_center, sphere_radius, cuboid_origin, cuboid_ckg)

    %% 参数
    q_limits          = [q_min; q_max];   % 关节空间边界
    max_iterations    = 500;
    step_size         = 0.2;
    connect_threshold = 0.3;              % 双树连接阈值
    goal_bias_prob    = 0.3;              % 目标偏置概率 
    d                 = size(q_limits,2); % 维度（通常为 6）
    gamma_rrt         = 2.0;              % RRT* 半径系数（可调）
    r_cap             = 4*step_size;      % 半径上限，避免过大邻域

    %% 初始化两棵树（A: start, B: goal）
    treeA = init_tree(q_start);
    treeB = init_tree(q_goal);

    path_found   = false;
    conn_idx_A   = [];
    conn_idx_B   = [];

    %% 主循环（交替扩展）
    for iter = 1:max_iterations
        if mod(iter,10) < 5
            [treeA, treeB, connected, ia, ib] = extend_trees_star( ...
                robot, treeA, treeB, q_limits, link_radius, ...
                sphere_center, sphere_radius, cuboid_origin, cuboid_ckg, ...
                step_size, goal_bias_prob, connect_threshold, ...
                gamma_rrt, r_cap, d);
        else
            [treeB, treeA, connected, ib, ia] = extend_trees_star( ...
                robot, treeB, treeA, q_limits, link_radius, ...
                sphere_center, sphere_radius, cuboid_origin, cuboid_ckg, ...
                step_size, goal_bias_prob, connect_threshold, ...
                gamma_rrt, r_cap, d);
        end

        if connected
            path_found = true;
            conn_idx_A = ia;
            conn_idx_B = ib;
            break;
        end
    end
    % if connected
    %     path_found = true;
    %     conn_idx_A = ia;
    %     conn_idx_B = ib;
    % end
    %% 路径重构
    if path_found
        % 注意：conn_idx_A 属于 treeA，conn_idx_B 属于 treeB
        pathA = reconstruct_from(treeA, conn_idx_A);   % start -> 接点A
        pathB = reconstruct_from(treeB, conn_idx_B);   % goal  -> 接点B
        pathB = flipud(pathB);                         % 反转为 接点B -> goal

        % 合并（去除重复接点）
        if ~isempty(pathA) && ~isempty(pathB) && norm(pathA(end,:) - pathB(1,:)) < 1e-12
            combined_path = [pathA; pathB(2:end,:)];
        else
            combined_path = [pathA; pathB];
        end

        [~, unique_idx] = unique(combined_path, 'rows', 'stable');
        combined_path = combined_path(unique_idx, :);

        path.pos = struct('q', num2cell(combined_path, 2));  % 兼容你此前的输出形式
    else
        path = struct('q', {});
    end
end

%% —— 子函数 —— %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function T = init_tree(q_root)
    T.v = struct('q', {}, 'idx_pre', {}, 'cost', {});
    T.v(1).q       = q_root;
    T.v(1).idx_pre = 0;
    T.v(1).cost    = 0.0;
end

function [current_tree, other_tree, connected, conn_idx_cur, conn_idx_oth] = extend_trees_star( ...
    robot, current_tree, other_tree, q_limits, link_radius, ...
    sphere_center, sphere_radius, cuboid_origin, cuboid_ckg, ...
    step_size, bias_prob, connect_thresh, gamma_rrt, r_cap, d)

    connected     = false;
    conn_idx_cur  = [];
    conn_idx_oth  = [];

    % --- 1) 目标偏置采样（偏向对方树的末端结点） ---
    if rand() < bias_prob
        if rand() < 0.7 && ~isempty(other_tree.v)
            q_target = other_tree.v(end).q;
        else
            q_target = rand_sample(q_limits);
        end
    else
        q_target = rand_sample(q_limits);
    end

    % --- 2) 最近邻 & “拉伸”一步 ---
    nodes   = vertcat(current_tree.v.q);
    [idx_near, ~] = nearest_idx(nodes, q_target);
    q_near  = current_tree.v(idx_near).q;
    q_new   = steer_toward(q_near, q_target, step_size);
    q_new   = clamp_limits(q_new, q_limits);

    % --- 3) 可行性检测 ---
    if check_edge(robot, q_near, q_new, link_radius, sphere_center, sphere_radius, cuboid_origin, cuboid_ckg)
        return; % 碰撞，放弃本次
    end

    % --- 4) RRT* 邻域半径 ---
    n = numel(current_tree.v);
    r = min(r_cap, gamma_rrt * (log(max(2,n))/n)^(1/d));  % 标准公式，防 log(1) 与幂错误

    % --- 5) 选择最优父结点 ---
    idx_near_set = neighbor_indices(nodes, q_new, r);
    if isempty(idx_near_set), idx_near_set = idx_near; end  % 退化为最近邻

    best_parent = [];
    best_cost   = inf;

    for k = 1:numel(idx_near_set)
        ip = idx_near_set(k);
        qp = current_tree.v(ip).q;
        if ~check_edge(robot, qp, q_new, link_radius, sphere_center, sphere_radius, cuboid_origin, cuboid_ckg)
            cand_cost = current_tree.v(ip).cost + norm(q_new - qp);
            if cand_cost < best_cost
                best_cost   = cand_cost;
                best_parent = ip;
            end
        end
    end

    if isempty(best_parent)
        % 邻域里没有可行父结点，放弃
        return;
    end

    % --- 6) 插入新结点 ---
    new_node.q       = q_new;
    new_node.idx_pre = best_parent;
    new_node.cost    = best_cost;
    current_tree.v(end+1) = new_node;
    new_idx = numel(current_tree.v);

    % --- 7) 邻域重连（若经由新结点更优则重设父结点） ---
    for k = 1:numel(idx_near_set)
        j = idx_near_set(k);
        if j == best_parent, continue; end
        qj = current_tree.v(j).q;
        if ~check_edge(robot, q_new, qj, link_radius, sphere_center, sphere_radius, cuboid_origin, cuboid_ckg)
            new_cost = current_tree.v(new_idx).cost + norm(qj - q_new);
            if new_cost + 1e-12 < current_tree.v(j).cost
                % 重新接到 new_idx 并向下更新子树代价
                old_parent = current_tree.v(j).idx_pre; %#ok<NASGU>
                current_tree.v(j).idx_pre = new_idx;
                delta = new_cost - current_tree.v(j).cost;
                current_tree.v(j).cost = new_cost;
                current_tree = propagate_costs(current_tree, j, delta);
            end
        end
    end

    % --- 8) 连接另一棵树（直接检查 q_new 与对方最近点） ---
    if ~isempty(other_tree.v)
        nodesB = vertcat(other_tree.v.q);
        [idxB, distB] = nearest_idx(nodesB, q_new);
        if distB < connect_thresh
            qB = other_tree.v(idxB).q;
            if ~check_edge(robot, q_new, qB, link_radius, sphere_center, sphere_radius, cuboid_origin, cuboid_ckg)
                connected    = true;
                conn_idx_cur = new_idx;
                conn_idx_oth = idxB;
            end
        end
    end
end

function current_tree = propagate_costs(current_tree, root_idx, delta)
    % 从 root_idx 向其后代传播代价增量 delta
    % 简单 O(N) 遍历（可根据需求优化为孩子表）
    n = numel(current_tree.v);
    changed = true;
    while changed
        changed = false;
        for i = 1:n
            p = current_tree.v(i).idx_pre;
            if p == root_idx
                current_tree.v(i).cost = current_tree.v(i).cost + delta;
                % 继续向下层传播：把该结点标为新的“根”之一
                root_idx = i;  % 逐层推进
                changed = true;
            end
        end
    end
end

function q = rand_sample(q_limits)
    q = q_limits(1,:) + (q_limits(2,:)-q_limits(1,:)).*rand(1, size(q_limits,2));
end

function q2 = steer_toward(q1, q_target, step)
    v = q_target - q1;
    nv = norm(v);
    if nv <= step
        q2 = q_target;
    else
        q2 = q1 + (step/nv) * v;
    end
end

function q = clamp_limits(q, q_limits)
    q = max(min(q, q_limits(2,:)), q_limits(1,:));
end

function [idx, dmin] = nearest_idx(nodes, q)
    % nodes: N×D, q: 1×D
    dists = vecnorm(nodes - q, 2, 2);
    [dmin, idx] = min(dists);
end

function ids = neighbor_indices(nodes, q, r)
    dists = vecnorm(nodes - q, 2, 2);
    ids   = find(dists <= r);
end

function path = reconstruct_from(tree, idx_leaf)
    if isempty(tree.v) || isempty(idx_leaf), path = []; return; end
    seq = [];
    cur = idx_leaf;
    while cur ~= 0
        seq(end+1) = cur; %#ok<AGROW>
        cur = tree.v(cur).idx_pre;
    end
    seq = fliplr(seq);
    path = vertcat(tree.v(seq).q);
end
