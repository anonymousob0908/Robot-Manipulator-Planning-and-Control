%--------------------------LQR-----------------------------
% 状态方程：
% dtheta = u
% y = theta

% A=0;B=I;C=I;D=0;

thetamat=load('thetamat.mat');
thetamat=thetamat.q_opt_total;
dthetamat=load('dthetamat.mat');
dthetamat=double(dthetamat.dthetamat{1});
T=load('tmat.mat');
T = T.t_total;

N = length(T);      % 时间步数
dt = 0.01;          % 时间步长
Q = 100;            % 误差权重
R = 1;              % 控制权重
learning_rate = 0.001; % 学习率
max_iter = 1000;     % 最大迭代次数
tolerance = 1e-2;   % 收敛容差

% u = zeros(7,length(T));
% y = zeros(7,length(T));
% e = zeros(7,length(T));
% for i = 2:length(T)
%     y(:,i) = y(:,i-1)+(u(:,i-1)+u(:,i))/2*0.01;
%     e(:,i) = y(:,i) - thetamat(:,i);
% end
% Q = 100*eye(1);
% R = 1*eye(1);
% %E'QE
% Je = 0;
% EQE = zeros(1,length(T));
% for i = 2:length(T)
%     EQE(i) = e(:,i)'*Q*e(:,i);
%     Je = Je+(EQE(:,i-1)+EQE(:,i))/2*0.01;
% end
% %U'RU
% Jr = 0;
% URU = zeros(1,length(T));
% for i = 2:length(T)
%     URU(i) = u(:,i)'*R*u(:,i);
%     Jr = Jr+(URU(:,i-1)+URU(:,i))/2*0.01;
% end
% J = Je + Jr;


% 记录代价函数历史
J_history = zeros(7, max_iter);
for dim = 1:7
    fprintf('优化维度 %d...\n', dim);
    
    % 当前维度的参考轨迹
    theta = thetamat(dim, :);
    
    % 初始化控制序列
    u = zeros(1, N);
    
    for iter = 1:max_iter
        % 1. 前向传播：计算状态y和误差e
        y = zeros(1, N);
        e = zeros(1, N);
        y(1) = 0; % 初始状态
        
        for i = 2:N
            y(i) = y(i-1) + (u(i-1) + u(i)) / 2 * dt;
            e(i) = y(i) - theta(i);
        end
        
        % 2. 计算代价函数J
        Je = 0;
        Jr = 0;
        
        for i = 2:N
            % 误差代价项
            Je = Je + (e(i-1)^2 * Q + e(i)^2 * Q) / 2 * dt;
            % 控制代价项
            Jr = Jr + (u(i-1)^2 * R + u(i)^2 * R) / 2 * dt;
        end
        
        J = Je + Jr;
        J_history(dim,iter) = J;
        
        % 3. 反向传播：计算梯度
        grad_u = zeros(1, N);
        
        % 初始化伴随变量
        lambda = zeros(1, N);
        
        % 计算终端伴随变量
        lambda(N) = Q * e(N) * dt;
        
        % 反向传播伴随变量
        for i = N-1:-1:1
            if i == 1
                lambda(i) = lambda(i+1) + Q * e(i) * dt;
            else
                lambda(i) = lambda(i+1) + Q * e(i) * dt * 2;
            end
        end
        
        % 计算梯度
        for i = 1:N
            if i == 1
                grad_u(i) = R * u(i) * dt + lambda(i+1) * dt/2;
            elseif i == N
                grad_u(i) = R * u(i) * dt + lambda(i) * dt/2;
            else
                grad_u(i) = R * u(i) * dt * 2 + lambda(i) * dt/2 + lambda(i+1) * dt/2;
            end
        end
        
        % 4. 更新控制输入
        u = u - learning_rate * grad_u;
        
        % 5. 检查收敛性
        if iter > 1 && abs(J_history(dim,iter) - J_history(dim,iter-1)) < tolerance
            fprintf('维度 %d 在第 %d 次迭代收敛\n', dim, iter);
            break;
        end
        
        if iter == max_iter
            fprintf('维度 %d 达到最大迭代次数\n', dim);
        end
    end
    
    % 存储优化结果
    u_opt(dim, :) = u;
end
% 验证优化结果
fprintf('\n验证优化后的代价函数:\n');
J_optimized = 0;
y_opt = zeros(7, N);
Je_opt=0;
Jr_opt=0;
for i = 2:N
    y_opt(:, i) = y_opt(:, i-1) + (u_opt(:, i-1) + u_opt(:, i)) / 2 * dt;
    e_opt = y_opt(:, i) - thetamat(:, i);
    
    % 误差代价
    Je_opt = Je_opt + (e_opt' * Q * e_opt) * dt;
    % 控制代价
    Jr_opt = Jr_opt + (u_opt(:, i)' * R * u_opt(:, i)) * dt;
end

J_optimized = Je_opt + Jr_opt;
fprintf('优化后的代价函数 J = %.6f\n', J_optimized);

% 可视化结果（可选）
figure;
for dim = 1:7
    subplot(4, 2, dim);
    plot(T, thetamat(dim, :), 'r-', 'LineWidth', 2, 'DisplayName', '参考轨迹');
    hold on;
    plot(T, y_opt(dim, :), 'b--', 'LineWidth', 1.5, 'DisplayName', '优化轨迹');
    plot(T, u_opt(dim, :), 'g-.', 'LineWidth', 1, 'DisplayName', '控制输入');
    title(sprintf('维度 %d', dim));
    xlabel('时间');
    legend;
    grid on;
end
