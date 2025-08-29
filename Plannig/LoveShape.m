% Just some sample face and vertex data.
% platonic_solid by Kevin Moerman: https://www.mathworks.com/matlabcentral/fileexchange/28213-platonic-solid
% helped me generate these numbers
% t=0:2*pi/3600:2*pi;
% x=16*sin(t).^3/30;
% y=(13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t))/30;
% z=0.5*ones(1,length(t));
% plot3(x,y,z)
VO1 = [0 -0.6 -0.3;
      -0.6 0.1 -0.3;
      -0.3 0.4 -0.3;
      -0.1 0.4 -0.3;
      0 0.2 -0.3;
      0 -0.6 -0.1;
      -0.6 0.1 -0.1;
      -0.3 0.4 -0.1;
      -0.1 0.4 -0.1;
      0 0.2 -0.1;]*300;

FO1 = [1 2 3 4 5;
      6 7 8 9 10;
      1 2 7 6 NaN;
      2 3 8 7 NaN;
      3 4 9 8 NaN;
      4 5 10 9 NaN;
      5 1 6 10 NaN];

VO2 = [0 -0.6 -0.3;
      0.6 0.1 -0.3;
      0.3 0.4 -0.3;
      0.1 0.4 -0.3;
      0 0.2 -0.3;
      0 -0.6 -0.1;
      0.6 0.1 -0.1;
      0.3 0.4 -0.1;
      0.1 0.4 -0.1;
      0 0.2 -0.1;]*300;

FO2 = [1 2 3 4 5;
      6 7 8 9 10;
      1 2 7 6 NaN;
      2 3 8 7 NaN;
      3 4 9 8 NaN;
      4 5 10 9 NaN;
      5 1 6 10 NaN];



