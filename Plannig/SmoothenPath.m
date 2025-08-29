% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the 
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = SmoothenPath(robot, pos, link_radius, sphere_centers, sphere_radii, cuboid_origin, cuboid_ckg)
path = [];
for i = 1:length(pos)
    path = [path;pos(i).q];
end

smoothed_path = path;
outif = 1;

while outif
    outif = 0;
    for i=1:size(smoothed_path,1)-2     
        j = i+2;

% The visualization of the process of optimizing and removing collision-free points.      
% If the commented code section is enabled, the trajectory process from i to i+2 can be observed to check for collisions.    
%         path12 = [smoothed_path(i,:); smoothed_path(j,:)];
%         T3 = zeros(4,4,size(path12,1));
%         for k = 1:size(path12,1)
%             T3(:,:,k) = robot.fkine(path12(k,:));
%         end
%         plot3(squeeze(T3(1, 4, :)), squeeze(T3(2, 4, :)), squeeze(T3(3, 4, :)), 'g-', 'LineWidth', 2);
%         robot.plot(interpolate_path(path12), 'fps', 10);

        if ~check_edge(robot, smoothed_path(i,:), smoothed_path(j,:), link_radius, sphere_centers, sphere_radii, cuboid_origin, cuboid_ckg, 25)
            smoothed_path = [smoothed_path(1:i,:);smoothed_path(j:end,:)];
            outif = 1;
            break;
        end        
    end       
end
% for i=1:length(smoothed_path)
%     q = smoothed_path(i,:);
%     if q==path(end,:)
%         break
%     end
%     for j=[length(smoothed_path):-1:1]
%         if i<j
%             if ~check_edge(robot, smoothed_path(i,:), smoothed_path(j,:), link_radius, sphere_centers, sphere_radii,25)
%                 smoothed_path=[smoothed_path(1:i,:);smoothed_path(j:end,:)]
%             end
%         end
%     end
% end
% end