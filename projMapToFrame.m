function [proj_map, proj_flag] = projMapToFrame(fusion_map, h, w, tform, cam_param)
    
    %==== Set parameters ====
    fx = cam_param(1);
    fy = cam_param(2);
    cx = cam_param(3);
    cy = cam_param(4);

    % disp(size(fusion_map.pointcloud.Location));
    %==== TODO: Project all terms in the fusion map based on other input parameters ====
    %==== (Hint 1: Only project the points in front of the camera) ====
    %==== (Hint 2: Calculate all the projected indices together and use them in vectorization) ====
    %==== (Hint 3: Discard the indices that exceed the frame boundaries) ====

    % Write your code here...
    
    % convert the global fusion map to current camera frame.
    % this will give us depth of the global points in the current camera frame.
    map_transformed = pctransform(fusion_map.pointcloud, tform.invert);
    % the size of the transformed pt cloud is same as original.
    
    % check if the depth of these points is positive.
    % this will return (num_points, 1)
    is_front = map_transformed.Location(:, 3) > 0;

    % then we convert it to the desired shape i.e (num_points, 3)
    is_front = repmat(is_front, 1, 3);

    % get a mask which we can multiply with transformed fusion map.
    is_front_mask = and(true(size(fusion_map.pointcloud.Location, 1), 3), is_front);

    % perform element wise multiplication with transformed points.
    transformed_pts_valid = map_transformed.Location .* is_front_mask;

    % define the camera intrinsics.
    camera_matrix = [fx 0 cx; 0 fy cy; 0 0 1];

    % lets check which of these points lie inside the image.

    % get points to be projected and reshape to (3, num of pts to be projected).
    
    proj_pts_img_plane = camera_matrix * reshape(transformed_pts_valid(is_front_mask), [], 3)';

    % get homogenised coordinates.
    proj_pts_img_plane = proj_pts_img_plane ./ proj_pts_img_plane(3, :);

    transformed_pts_valid(is_front_mask) = reshape(proj_pts_img_plane', [], 1);

    % disp(size(transformed_pts_valid))
    % check for boundaries.
    within_boundary = transformed_pts_valid(:, 1) > 0 & transformed_pts_valid(:, 2) > 0 & transformed_pts_valid(:, 1) < h & transformed_pts_valid(:, 2) < w;
    
    within_boundary = repmat(within_boundary, 1, 3);
    
    % disp(size(within_boundary));
    % overall valid points.
    proj_flag = and(is_front_mask, within_boundary);

    % form a matrix of the global points which are valid.
    % disp(size(fusion_map.pointcloud.Location(is_front_mask)));
    % disp(size(fusion_map.pointcloud.Location(proj_flag)));
    valid_global_pts = reshape(fusion_map.pointcloud.Location(proj_flag), [], 3);
    
    % retrieve the pixel location that correspond to these valid points.
    % the third column here is irrelevant as it is in homogenous coordinates.
    valid_global_pts_pix = reshape(ceil(transformed_pts_valid(proj_flag)), [], 3); 

    % for indexing in the final answer.
    row_ind = valid_global_pts_pix(:, 1);
    col_ind = valid_global_pts_pix(:, 2);

    proj_points = zeros(h * w, 3);

    proj_points(row_ind * h + col_ind + 1, :) = valid_global_pts;
    proj_points = reshape(proj_points, [h, w, 3]);

    % now we update normal, colors and counts.
    valid_global_colors = reshape(fusion_map.pointcloud.Color(proj_flag), [], 3);
    valid_global_normals = reshape(fusion_map.normals(proj_flag), [], 3);
    valid_global_ccounts = fusion_map.ccounts(proj_flag(:, 1));
    valid_global_times = fusion_map.times(proj_flag(:, 1));

    proj_colors = zeros(h * w, 3);
    proj_normals = zeros(h * w, 3);
    proj_ccounts = zeros(h * w, 1);
    proj_times = zeros(h * w, 1);

    proj_colors(row_ind * h + col_ind + 1, :) = valid_global_colors;
    proj_normals(row_ind * h + col_ind + 1, :) = valid_global_normals;
    proj_ccounts(row_ind * h + col_ind + 1, :) = valid_global_ccounts;
    proj_times(row_ind * h + col_ind + 1, :) = valid_global_times;

    proj_colors = reshape(proj_colors, [h, w, 3]);
    proj_normals = reshape(proj_normals, [h, w, 3]); 
    proj_ccounts = reshape(proj_ccounts, [h, w, 1]); 
    proj_times = reshape(proj_times, [h, w, 1]); 

    proj_flag = proj_flag(:, 1);

    % disp("all set");

    %==== Output the projected map in a struct ====
    %==== (Notice: proj_points[], proj_colors[], and proj_normals[] are all 3D matrices with size h*w*3) ====
    %==== (Notice: proj_ccounts[] and proj_times[] are both 2D matrices with size h*w) ====
    proj_map = struct('points', proj_points, 'colors', proj_colors, 'normals', proj_normals, 'ccounts', proj_ccounts, 'times', proj_times);
        
end
