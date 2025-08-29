%--------------------------MPC-----------------------------
thetamat=load('thetamat.mat');
thetamat=thetamat.q_opt_total;
dthetamat=load('dthetamat.mat');
dthetamat=double(dthetamat.dthetamat{1});
T=load('tmat.mat');
T = T.t_total;

N = 10;      % 时间步数
dt = 0.01;          % 时间步长
Q = 100;            % 误差权重
R = 1;              % 控制权重
F = 100;              % 末端误差权重
learning_rate = 0.001; % 学习率
max_iter = 1000;     % 最大迭代次数
tolerance = 1e-2;   % 收敛容差

for k = 1:700%now_step
    % 记录代价函数历史
    J_history = zeros(7, max_iter);
    for dim = 1:7
        fprintf('优化维度 %d...\n', dim);
        
        % 当前维度的参考轨迹
        theta = thetamat(dim, k:k+N-1);
        
        % 初始化控制序列
        u = zeros(1, N);
        if k > 1
            u(1) = u_mpc(dim, k-1); % warm start
        end
        % 梯度下降优化
        for iter = 1:max_iter
            % 1. 前向传播：计算预测状态y和误差e
            y_pred = zeros(1, N);
            e_pred = zeros(1, N);
            if k>1
                y_pred(1) = y_mpc(dim, k-1); % 从当前实际状态开始
                e_pred(1) = y_pred(1) - theta(1);
            end
            for i = 2:N
                y_pred(i) = y_pred(i-1) + (u(i-1) + u(i)) / 2 * dt;
                e_pred(i) = y_pred(i) - theta(i);
            end
            
            % 2. 计算代价函数J
            Je = 0;
            Jr = 0;
            
            for i = 1:N
                % 误差代价项（从当前步开始）
                Je = Je + e_pred(i)^2 * Q;
                % 控制代价项
                Jr = Jr + u(i)^2 * R;
            end
            
            % 添加终端代价
            J = Je + Jr + e_pred(N)^2 * F;
            J_history(dim, iter) = J;
            
            % 3. 反向传播：计算梯度
            grad_u = compute_gradient(y_pred, u, e_pred, theta, dt, Q, R, F, N);
            
            % 4. 更新控制输入
            u = u - learning_rate * grad_u;
            
            % 简单的收敛检查
            if iter > 1 && abs(J_history(dim, iter) - J_history(dim, iter-1)) < tolerance
                fprintf('维度 %d 在第 %d 次迭代收敛\n', dim, iter);
                break;
            end
        end
        
        % 存储优化结果（只取第一个控制量）
        u_mpc(dim, k) = u(1);
    end
    
    % 应用控制并更新实际状态
    for dim = 1:7
        y_mpc(dim,k) = 0;
        if k > 1
            y_mpc(dim, k) = y_mpc(dim, k-1) + (u_mpc(dim, k-1) + u_mpc(dim, k)) / 2 * dt;
        end
    end
end

% 可视化结果（可选）
figure;
for dim = 1:7
    subplot(4, 2, dim);
    plot(T(1:700), thetamat(dim, 1:700), 'r-', 'LineWidth', 2, 'DisplayName', '参考轨迹');
    hold on;
    plot(T(1:700), y_mpc(dim, :), 'b--', 'LineWidth', 1.5, 'DisplayName', '优化轨迹');
    plot(T(1:700), u_mpc(dim, :), 'g-.', 'LineWidth', 1, 'DisplayName', '控制输入');
    title(sprintf('维度 %d', dim));
    xlabel('时间');
    legend;
    grid on;
end

%% 梯度计算函数
function grad_u = compute_gradient(y, u, e, theta, dt, Q, R, F, N)
    grad_u = zeros(1, N);
    lambda = zeros(1, N); % 伴随变量
    
    % 终端伴随变量（包含终端代价）
    lambda(N) = 2 * F * e(N);
    
    % 反向传播伴随变量
    for i = N-1:-1:1
        % 标准伴随方程：λ_i = λ_{i+1} + ∂J/∂y_i
        lambda(i) = lambda(i+1) + 2 * Q * e(i);
        
        % 考虑动力学约束的传播
        % y_{i+1} = y_i + (u_i + u_{i+1})/2 * dt
        % 所以 ∂y_{i+1}/∂y_i = 1
        % 伴随变量传播正确
    end
    
    % 计算控制梯度
    for i = 1:N
        % 直接梯度项：∂J/∂u_i
        direct_grad = 2 * R * u(i);
        
        % 间接梯度项：通过动力学约束
        indirect_grad = 0;
        
        if i < N
            % u_i 影响 y_{i+1}: ∂y_{i+1}/∂u_i = dt/2
            indirect_grad = indirect_grad + lambda(i+1) * (dt/2);
        end
        
        if i > 1
            % u_i 影响 y_i: ∂y_i/∂u_i = dt/2 (从动力学方程推导)
            indirect_grad = indirect_grad + lambda(i) * (dt/2);
        end
        
        grad_u(i) = direct_grad + indirect_grad;
    end
end
