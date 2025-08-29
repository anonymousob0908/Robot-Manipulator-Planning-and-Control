% Example script for GJK function
%   Animates two objects on a collision course and terminates animation
%   when they hit each other. Loads vertex and face data from
%   SampleShapeData.m. See the comments of GJK.m for more information
%
%   Most of this script just sets up the animation and transformations of
%   the shapes. The only key line is:
%   collisionFlag = GJK(S1Obj,S2Obj,iterationsAllowed)
%
%   Matthew Sheen, 2016
clc;clear all;close all

%How many iterations to allow for collision detection.
iterationsAllowed = 100;

% Make a figure
fig = figure(2);
hold on

% Load sample vertex and face data for two convex polyhedra
LoveShape;

% Make shape 1
S1.Vertices = VO1;
S1.Faces = FO1;
S1.FaceVertexCData = jet(size(VO1,1));
S1.FaceColor = 'interp';
S1Obj = patch(S1);

% Make shape 2
S2.Vertices = VO2;
S2.Faces = FO2;
S2.FaceVertexCData = jet(size(VO2,1));
S2.FaceColor = 'interp';
S2Obj = patch(S2);

hold off
axis equal
% axis([-500 500 -500 500 -500 500])
% fig.Children.Visible = 'off'; % Turn off the axis for more pleasant viewing.
fig.Color = [1 1 1];
rotate3d on;

collisionFlag = GJK(S1Obj,S2Obj,iterationsAllowed);

