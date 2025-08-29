function [S,Sd,Sdd]= b_spline(Poses,t,k)
    S = [];
    Sd = [];
    Sdd = [];
    n = size(Poses,2) - 1;
    if (k==2)
        disp('Using Quadratic B-spline')
        M = 0.5.*[1 -2 1;-2 2 0;1 1 0]; % Basis function matrix
        T = [t.^2;t.^1;t.^0]';
        Td = [2*t;1*t.^0;0*t.^0]';
        Tdd = [2*t.^0;0*t.^0;0*t.^0]';
        for i = 1:n-k+1
            B = T*M*Poses(:,i:i+k)'; % B-spline segment
            Bd = Td*M*Poses(:,i:i+k)';
            Bdd = Tdd*M*Poses(:,i:i+k)';
            S = [S;B];
            Sd = [Sd;Bd];
            Sdd = [Sdd;Bdd];
        end      
        S = S';
        Sd = Sd';
        Sdd = Sdd';
        return

    elseif (k==3)
        disp('Using Cubic B-spline')
        M = (1/6).*[-1 3 -3 1;3 -6 3 0;-3 0 3 0;1 4 1 0]; % Basis function matrix
        T = [t.^3;t.^2;t.^1;t.^0]';
        Td = [3*t.^2;2*t;t.^0;0*t.^0]';
        Tdd = [6*t;2*t.^0;0*t.^0;0*t.^0]';
        for i = 1:n-k+1
            B = T*M*Poses(:,i:i+k)'; % B-spline segment
            Bd = Td*M*Poses(:,i:i+k)';
            Bdd = Tdd*M*Poses(:,i:i+k)';
            S = [S;B];
            Sd = [Sd;Bd];
            Sdd = [Sdd;Bdd];
        end      
        S = S';
        Sd = Sd';
        Sdd = Sdd';
        return

    elseif (k==5)
        disp('Using Quintic B-spline')
        M = (1/120).*[1 -5 10 -10 5 -1;
                      -5 20 -40 40 -20 5;
                      10 -40 80 -80 40 -10;
                      -10 40 -80 80 -40 10;
                      5 -20 40 -40 20 -5;
                      -1 5 -10 10 -5 1]; % Basis function matrix

        T = [t.^5; t.^4; t.^3; t.^2; t.^1; t.^0]';
        Td = [5*t.^4; 4*t.^3; 3*t.^2; 2*t; t.^0; 0*t.^0]';
        Tdd = [20*t.^3; 12*t.^2; 6*t; 2*t.^0; 0*t.^0; 0*t.^0]';
        for i = 1:n-k+1
            B = T*M*Poses(:,i:i+k)'; % B-spline segment
            Bd = Td*M*Poses(:,i:i+k)';
            Bdd = Tdd*M*Poses(:,i:i+k)';
            S = [S; B];
            Sd = [Sd; Bd];
            Sdd = [Sdd; Bdd];
        end
        S = S';
        Sd = Sd';
        Sdd = Sdd';
        return

    else
        disp('This function works with quadratic and cubic B-splines!')
        return 
    end    
end