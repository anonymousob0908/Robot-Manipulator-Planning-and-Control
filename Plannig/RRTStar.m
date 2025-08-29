% 普通 RRT*（单树）— 含目标偏置与重连
function [path, path_found] = RRTStar(robot, q_min, q_max, q_start, q_goal, link_radius, ...
    sphere_center, sphere_radius, cuboid_origin, cuboid_ckg)

    %% 参数
    q_limits        = [q_min; q_max];     % 关节空间边界 (2×D)
    D               = size(q_limits,2);   % 维度（通常为6）
    max_iterations  = 5000;
    step_size       = 0.5;
    goal_threshold  = 0.7;                % 接近目标的判定阈值
    goal_bias_prob  = 0.2;                % 目标偏置概率 
    gamma_rrt       = 2.0;                % RRT* 半径系数
    r_cap           = 4*step_size;        % 邻域半径上限，避免过大
    eps_equal       = 1e-12;

    %% 初始化树（根为 start）
    T = init_tree(q_start);

    goal_idx   = [];
    path_found = false;

    for iter = 1:max_iterations
        %% 1) 采样（带目标偏置）
        if rand() < goal_bias_prob
            q_rand = q_goal;
        else
            q_rand = rand_sample(q_limits);
        end

        %% 2) 最近邻 & 拉伸一步
        nodes  = vertcat(T.v.q);          % N×D
        [idx_near, ~] = nearest_idx(nodes, q_rand);
        q_near = T.v(idx_near).q;
        q_new  = steer_toward(q_near, q_rand, step_size);
        q_new  = clamp_limits(q_new, q_limits);

        %% 3) 可行性检测（边段）
        if check_edge(robot, q_near, q_new, link_radius, ...
                      sphere_center, sphere_radius, cuboid_origin, cuboid_ckg)
            continue; % 碰撞，跳过
        end

        %% 4) RRT* 邻域半径
        n = numel(T.v);
        r = min(r_cap, gamma_rrt * (log(max(2,n))/n)^(1/D));  % 标准公式（标量幂）

        %% 5) 选择最优父结点
        idx_set = neighbor_indices(nodes, q_new, r);
        if isempty(idx_set), idx_set = idx_near; end  % 退化为最近邻

        best_parent = [];
        best_cost   = inf;

        for k = 1:numel(idx_set)
            ip = idx_set(k);
            qp = T.v(ip).q;
            if ~check_edge(robot, qp, q_new, link_radius, ...
                           sphere_center, sphere_radius, cuboid_origin, cuboid_ckg)
                cand = T.v(ip).cost + norm(q_new - qp);
                if cand < best_cost
                    best_cost   = cand;
                    best_parent = ip;
                end
            end
        end
        if isempty(best_parent), continue; end

        %% 6) 插入新结点
        new_node.q       = q_new;
        new_node.idx_pre = best_parent;
        new_node.cost    = best_cost;
        T.v(end+1) = new_node;
        new_idx = numel(T.v);

        %% 7) 邻域重连（若经 new 更优则重设父结点并更新子树代价）
        for k = 1:numel(idx_set)
            j = idx_set(k);
            if j == best_parent, continue; end
            qj = T.v(j).q;
            if ~check_edge(robot, q_new, qj, link_radius, ...
                           sphere_center, sphere_radius, cuboid_origin, cuboid_ckg)
                new_cost = T.v(new_idx).cost + norm(qj - q_new);
                if new_cost + eps_equal < T.v(j).cost
                    T.v(j).idx_pre = new_idx;
                    T.v(j).cost    = new_cost;
                    T = update_subtree_costs(T, j);  % BFS 更新其后代
                end
            end
        end

        %% 8) 终止判据：接近目标且可直连 → 把目标加入树
        if norm(q_new - q_goal) <= goal_threshold && ...
           ~check_edge(robot, q_new, q_goal, link_radius, ...
                       sphere_center, sphere_radius, cuboid_origin, cuboid_ckg)
            goal_node.q       = q_goal;
            goal_node.idx_pre = new_idx;
            goal_node.cost    = T.v(new_idx).cost + norm(q_goal - q_new);
            T.v(end+1) = goal_node;
            goal_idx   = numel(T.v);
            path_found = true;
            break;
        end
    end

    %% 回溯路径
    if path_found
        P = reconstruct_from(T, goal_idx);     % start -> goal
        [~, uniq_idx] = unique(P, 'rows', 'stable');
        P = P(uniq_idx, :);
        path.pos = struct('q', num2cell(P, 2)); % 兼容原输出
    else
        path = struct('q', {});
    end
end

%% ===== 工具函数 =====

function T = init_tree(q_root)
    T.v = struct('q', {}, 'idx_pre', {}, 'cost', {});
    T.v(1).q       = q_root;
    T.v(1).idx_pre = 0;
    T.v(1).cost    = 0.0;
end

function q = rand_sample(q_limits)
    q = q_limits(1,:) + (q_limits(2,:)-q_limits(1,:)).*rand(1, size(q_limits,2));
end

function q2 = steer_toward(q1, q_target, step)
    v = q_target - q1; nv = norm(v);
    if nv <= step, q2 = q_target; else, q2 = q1 + (step/nv)*v; end
end

function q = clamp_limits(q, q_limits)
    q = max(min(q, q_limits(2,:)), q_limits(1,:));
end

function [idx, dmin] = nearest_idx(nodes, q)
    d = vecnorm(nodes - q, 2, 2);
    [dmin, idx] = min(d);
end

function ids = neighbor_indices(nodes, q, r)
    d = vecnorm(nodes - q, 2, 2);
    ids = find(d <= r);
end

function P = reconstruct_from(T, idx_leaf)
    seq = [];
    cur = idx_leaf;
    while cur ~= 0
        seq(end+1) = cur; %#ok<AGROW>
        cur = T.v(cur).idx_pre;
    end
    seq = fliplr(seq);
    P = vertcat(T.v(seq).q);
end

function T = update_subtree_costs(T, root_idx)
    % 基于父指针的孩子遍历，按父代价递推更新整棵子树
    Q = root_idx;
    while ~isempty(Q)
        p = Q(1); Q(1) = [];
        % 找到所有孩子
        for i = 1:numel(T.v)
            if T.v(i).idx_pre == p
                T.v(i).cost = T.v(p).cost + norm(T.v(i).q - T.v(p).q);
                Q(end+1) = i; %#ok<AGROW>
            end
        end
    end
end
