function updated_map = avgProjMapWithInputData(proj_map, input_data, alpha, h, w, is_use, t)

    %==== Set variables ====
    input_points = input_data.pointcloud.Location;
    input_colors = input_data.pointcloud.Color;
    input_normals = input_data.normals;
    proj_points = proj_map.points;
    proj_colors = proj_map.colors;
    proj_normals = proj_map.normals;
    proj_ccounts = proj_map.ccounts;
    proj_times = proj_map.times;

    %==== TODO: Update all the terms in the projected map using the input data ====
    %==== (Hint: apply is_use[] as a mask in vectorization) ====

    % Write your code here...
   
    % all pts which won't be updated retain the global proj map values.
    updated_points = proj_points;
    updated_colors = proj_colors;
    updated_normals = proj_normals;
    updated_ccounts = proj_ccounts;
    updated_times = proj_times;

    is_use_3d = repmat(is_use, 1, 3);
    proj_ccounts_3d = repmat(proj_ccounts, 1, 3);
    proj_times_3d = repmat(proj_map, 1, 3);
    alpha_3d = repmat(alpha, 1, 3);

    % valid_proj_points = proj_points(is_use_3d);

    updated_points(is_use_3d) =  (proj_ccounts_3d(is_use_3d) .* proj_points(is_use_3d) + alpha_3d(is_use_3d) .* input_points(is_use_3d)) ./ (proj_ccounts_3d(is_use_3d) + alpha_3d(is_use_3d));
    updated_normals(is_use_3d) =  (proj_ccounts_3d(is_use_3d) .* proj_normals(is_use_3d) + alpha_3d(is_use_3d) .* input_normals(is_use_3d)) ./ (proj_ccounts_3d(is_use_3d) + alpha_3d(is_use_3d));

    % disp(alpha_3d(is_use_3d) .* input_colors(is_use_3d));
    updated_colors(is_use_3d) = input_colors(is_use_3d);
    updated_ccounts(is_use) = proj_ccounts(is_use) + alpha(is_use);
    updated_times(is_use) = t;

    % disp('done averaging')
    
    %==== Output the updated projected map in a struct ====
    updated_map = struct('points', updated_points, 'colors', updated_colors, 'normals', updated_normals, 'ccounts', updated_ccounts, 'times', updated_times);
        
end