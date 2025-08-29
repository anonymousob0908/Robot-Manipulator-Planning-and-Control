function theta = myikine(T06,a,d)
    degtorad = pi/180;

    r11 = T06(1, 1); r21 = T06(2, 1); r31 = T06(3, 1);
    r12 = T06(1, 2); r22 = T06(2, 2); r32 = T06(3, 2);
    r13 = T06(1, 3); r23 = T06(2, 3); r33 = T06(3, 3);
    px = T06(1, 4); py = T06(2, 4); pz = T06(3, 4);
    d1 = d(1); d2 = d(2); d3 = d(3); d4 = d(4); d5 = d(5); d6 = d(6);
    a1 = a(1); a2 = a(2); a3 = a(3); a4 = a(4); a5 = a(5); a6 = a(6); 
    
    theta1 = [];
    theta2 = [];
    theta3 = [];
    theta4 = [];
    theta5 = [];
    theta6 = [];
    %% theta1
    m = -px+d6*r13;
    n = py-d6*r23;
%     theta1 = asin(-130/sqrt(m^2+n^2))-atan2(n/m);
    theta1 = zeros(1,2);
    if abs(m^2+n^2-d4^2) < 1e-5
        theta1(1,1) = atan2(-n,m)-pi/2;
        theta1(1,2) = atan2(-n,m)-pi/2;
    elseif m^2+n^2-d4^2<0
            theta1(1,1) = inf;
            theta1(1,2) = inf;
    else
        theta1(1,1) = atan2(-n,m)+atan2(d4,sqrt(m^2+n^2-d4^2));
        theta1(1,2) = atan2(-n,m)+atan2(d4,-sqrt(m^2+n^2-d4^2));
    end
    for i = 1:2
        if theta1(1,i)> pi
            theta1(1,i) = theta1(1,i) - 2*pi;
        elseif theta1(1,i) < -pi
            theta1(1,i) = theta1(1,i) + 2*pi;
        end
    end

    %% theta5
    theta5 = zeros(2,2);
    theta5(1,1) = acos(r13*sin(theta1(1,1))-r23*cos(theta1(1,1)));
    theta5(1,2) = -acos(r13*sin(theta1(1,1))-r23*cos(theta1(1,1)));
    theta5(2,1) = acos(r13*sin(theta1(1,2))-r23*cos(theta1(1,2)));
    theta5(2,2) = -acos(r13*sin(theta1(1,2))-r23*cos(theta1(1,2)));
    theta5_use = [theta5(1,1),theta5(1,2),theta5(2,1),theta5(2,2)];
    theta5 = repmat(theta5_use,1,2);

    for i =1:length(theta5) 
        if abs(theta5(i)) <1e-5
            theta5(i) = 0;
        end
    end
    %% theta6
    for i =1:4
        a = r21*cos(theta1(ceil(i/2)))-r11*sin(theta1(ceil(i/2)));
        b = r22*cos(theta1(ceil(i/2)))-r12*sin(theta1(ceil(i/2)));
        if theta5_use(i) > 0
            theta6(i) = atan2(-b,a);
        elseif theta5_use(i) < 0
            theta6(i) = atan2(b,-a);
        else
            theta6(i) = inf;
        end
    end

    theta6 = [theta6(1,1),theta6(1,2);theta6(1,3),theta6(1,4)];
    theta6_use = [theta6(1,1),theta6(1,2),theta6(2,1),theta6(2,2)];

    theta6 = repmat(theta6_use,1,2);

    for i =1:8 
        if abs(theta6(i)) <1e-5
            theta6(i) = 0;
        end
    end

    %% theta3
    theta3 = zeros(2,2);
    for i =1:4
        x = px*cos(theta1(ceil(i/2))) + py*sin(theta1(ceil(i/2))) - d6*r13*cos(theta1(ceil(i/2))) ...
        - d6*r23*sin(theta1(ceil(i/2))) - d5*r21*sin(theta1(ceil(i/2))).*sin(theta6(i)) ...
        - d5*r12*cos(theta1(ceil(i/2))).*cos(theta6(i)) - d5*r11*cos(theta1(ceil(i/2))).*sin(theta6(i)) ...
        - d5*r22*cos(theta6(i)).*sin(theta1(ceil(i/2)));

        y = pz - d1 - d6*r33 - d5*r32*cos(theta6(i)) - d5*r31*sin(theta6(i));
        if (x.^2+y.^2-a4.^2-a3.^2)/(2*a3*a4) > 1 || (x.^2+y.^2-a4.^2-a3.^2)/(2*a3*a4)<-1
            theta3(1,i) = inf;
            theta3(2,i) = inf;
        else
            theta3(1,i) = acos((x.^2+y.^2-a4.^2-a3.^2)/(2*a3*a4));
            theta3(2,i) = -acos((x.^2+y.^2-a4.^2-a3.^2)/(2*a3*a4));
        end
    end        
    theta3 = [theta3(1,:),theta3(2,:)];
    for i =1:length(theta3) 
        if abs(theta3(i)) <1e-5
            theta3(i) = 0;
        end
    end
    %% theta2
    theta2 = zeros(1,8);
    theta1 = reshape(repmat(theta1,2,1),1,[]);
    theta1 = reshape(repmat(theta1,1,2),1,[]);
    for i =1:8
        x = px*cos(theta1(i)) + py*sin(theta1(i)) - d6*r13*cos(theta1(i)) ...
        - d6*r23*sin(theta1(i)) - d5*r21*sin(theta1(i)).*sin(theta6(i)) ...
        - d5*r12*cos(theta1(i)).*cos(theta6(i)) - d5*r11*cos(theta1(i)).*sin(theta6(i)) ...
        - d5*r22*cos(theta6(i)).*sin(theta1(i));

        y = pz - d1 - d6*r33 - d5*r32*cos(theta6(i)) - d5*r31*sin(theta6(i));

        r = [a4*sin(theta3(i)) a4*cos(theta3(i))+a3;
            a4*cos(theta3(i))+a3 -a4*sin(theta3(i))];
        if theta3(i) < 10^10
            sc2  = inv(r)*[x;y];
            if sc2(1)>1 || sc2(2)>1 || sc2(1)<-1 || sc2(2)<-1
                theta2(i) = inf;
            else
                theta2(i) = atan2(sc2(2),sc2(1));
            end
        else
            theta2(i) = inf;
        end
    end

    for i =1:length(theta2) 
        if abs(theta2(i)) <1e-5
            theta2(i) = 0;
        end
    end
    %% theta4
    theta(1,:) = theta1;
    theta(2,:) = theta2;
    theta(3,:) = theta3;
    theta(5,:) = theta5;
    theta(6,:) = theta6;

    for i =1:8
        c = r13*cos(theta(1,i))*sin(theta(5,i)) + r23*sin(theta(1,i))*sin(theta(5,i)) + r11*cos(theta(1,i))*cos(theta(5,i))*cos(theta(6,i)) - r12*cos(theta(1,i))*cos(theta(5,i))*sin(theta(6,i)) + r21*cos(theta(5,i))*cos(theta(6,i))*sin(theta(1,i)) - r22*cos(theta(5,i))*sin(theta(1,i))*sin(theta(6,i));
        s = r33*sin(theta(5,i)) + r31*cos(theta(5,i))*cos(theta(6,i)) - r32*cos(theta(5,i))*sin(theta(6,i));
        theta(4,i) = atan2(-s,c)-theta(2,i)-theta(3,i);
        if theta(4,i)>3.14
            theta(4,i) = theta(4,i) - 2*pi;
        elseif theta(4,i)<-3.14
            theta(4,i) = theta(4,i) + 2*pi;
        end
    end
    theta = theta';
    for i =1:length(theta4) 
        if abs(theta4(i)) <1e-5
            theta4(i) = 0;
        end
    end
end
