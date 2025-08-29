function polys = minimum_snap_7axis(waythetas, ts, n_order, v0, a0, ve, ae)
    Q_total = [];
    A_total = [];
    b_total = [];
    for k = 1:size(waythetas,1)
        % 第i个关节
        waypts = waythetas(k,:);
        p0 = waypts(1);
        pe = waypts(end);
        
        n_poly = length(waypts)-1;
        n_coef = n_order+1;
        
        % compute Q
        Q_all = [];
        for i=1:n_poly
            % 构建对角矩阵
            Q_all = blkdiag(Q_all,computeQ(n_order,3,ts(i),ts(i+1))); 
        end
        % b_all = zeros(size(Q_all,1),1);
        
        Aeq = zeros(4*n_poly+2,n_coef*n_poly);
        beq = zeros(4*n_poly+2,1);
        
        % start/terminal pva constraints  (6 equations)
        Aeq(1:3,1:n_coef) = [calc_tvec(ts(1),n_order,0);
                             calc_tvec(ts(1),n_order,1);
                             calc_tvec(ts(1),n_order,2)];
        Aeq(4:6,n_coef*(n_poly-1)+1:n_coef*n_poly) = ...
                            [calc_tvec(ts(end),n_order,0);
                             calc_tvec(ts(end),n_order,1);
                             calc_tvec(ts(end),n_order,2)];
        beq(1:6,1) = [p0,v0,a0,pe,ve,ae]';
        
        % mid p constraints    (n_ploy-1 equations)
        neq = 6;
        for i=1:n_poly-1
            neq=neq+1;
            Aeq(neq,n_coef*i+1:n_coef*(i+1)) = calc_tvec(ts(i+1),n_order,0);
            beq(neq) = waypts(i+1);
        end
        
        % continuous constraints  ((n_poly-1)*3 equations)
        for i=1:n_poly-1
            tvec_p = calc_tvec(ts(i+1),n_order,0);
            tvec_v = calc_tvec(ts(i+1),n_order,1);
            tvec_a = calc_tvec(ts(i+1),n_order,2);
            neq=neq+1;
            Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_p,-tvec_p];
            neq=neq+1;
            Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_v,-tvec_v];
            neq=neq+1;
            Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_a,-tvec_a];
        end
        
        Q_total = blkdiag(Q_total,Q_all); 
        A_total = blkdiag(A_total,Aeq);
        b_total = [b_total;beq];
        
        % p = quadprog(Q_all,b_all,Aieq,bieq,Aeq,beq);
    end
    %动力学约束
    Aieq = [];
    bieq = [];

    %一次项
    f = zeros(size(Q_total,1),1);
    options = optimoptions('quadprog', ...
    'OptimalityTolerance', 1e-8, ...
    'ConstraintTolerance', 1e-8, ...
    'MaxIterations', 1000, ...
    'StepTolerance', 1e-10, ...
    'Display', 'off');
    p = quadprog(Q_total,f,Aieq,bieq,A_total,b_total,[],[],[],options);
    
    polys = reshape(p,n_coef,n_poly,k);

end