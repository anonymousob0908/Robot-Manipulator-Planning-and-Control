function rpy = rotm2rpy( R )

if abs(R(3 ,1) - 1.0) < 1.0e-15   % singularity
    a = 0.0;
    b = -pi / 2.0;
    c = atan2(-R(1, 2), -R(1, 3));
elseif abs(R(3, 1) + 1.0) < 1.0e-15   % singularity
    a = 0.0;
    b = pi / 2.0;
    c = -atan2(R(1, 2), R(1, 3));
else
    a = atan2(R(3, 2), R(3, 3));
    c = atan2(R(2, 1), R(1, 1));
    %     a = atan2(-R(3, 2), -R(3, 3));  %a另一个解
    %     c = atan2(-R(2, 1), -R(1, 1));  %c另一个解
    cosC = cos(c);
    sinC = sin(c);
    
    if abs(cosC) > abs(sinC)
        b = atan2(-R(3, 1), R(1, 1) / cosC);
    else
        b = atan2(-R(3, 1), R(2, 1) / sinC);
    end
end

rpy = [a, b, c];

end