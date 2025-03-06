function visualize_point_cloud(X, marker_size, marker_prop)
%VISUALIZE_POINT_CLOUD Plot input point cloud
%   X (2 x n): 3D point cloud
  
if nargin < 2, marker_size = 1.0;
end

if nargin < 3, marker_prop = 'ko';
end

switch size(X, 1)
    case 2
    	plot(X(1,:), X(2,:), marker_prop, 'MarkerSize', marker_size);
    case 3
        plot3(X(1,:), X(2,:), X(3,:), marker_prop, 'MarkerSize', marker_size);
    case 6
        plot3(X(1,:), X(2,:), X(3,:), marker_prop, 'MarkerSize', marker_size);        
    otherwise
        error('Input must be a 2D or 3D point array.');
end
axis equal;

end

