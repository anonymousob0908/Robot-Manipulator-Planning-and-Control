% 输入参数：
%   waythetas -> 控制点
%   v_max -> 机械臂最大允许速度
%   a_max -> 机械臂最大允许加速度
% 输出参数：
%   t_points -> 分配时间
% 方法：
%   按照梯形运动曲线保守估算时间
function t_points = trapezoidal_time(waythetas, v_max, a_max)
    t_points = zeros(1, size(waythetas,2));
    for i = 2:size(waythetas,2)
        dtheta = max(abs(waythetas(:,i) - waythetas(:,i-1)));
        % 计算梯形速度曲线时间（考虑是否达到最大速度）
        t_accel = v_max / a_max;                 % 加速段时间
        d_accel = 0.5 * a_max * t_accel^2;       % 加速段位移
        if dtheta <= 2 * d_accel                 % 三角形速度曲线
            t = 2 * sqrt(dtheta / a_max);        
        else                                      % 梯形速度曲线
            t_cruise = (dtheta - 2 * d_accel) / v_max;
            t = 2 * t_accel + t_cruise;
        end
        t_points(i) = t_points(i-1) + t;
    end
end

