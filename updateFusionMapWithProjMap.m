function fusion_map = updateFusionMapWithProjMap(fusion_map, updated_map, h, w, proj_flag)

    %==== TODO: Merge the updated map with the remaining part of the old fusion map ====

    % Write your code here...
    % disp("fusion map pc shape");
    % disp(size(fusion_map.pointcloud.Location));

    % disp("updated map pc shape");
    % disp(size(updated_map.points));	

    unchanged_mask = not(proj_flag);
    unchanged_mask_3d = repmat(unchanged_mask, 1, 3);

    % flatten the updated map which is in 3d.
    new_pts_flat = reshape(updated_map.points, [h * w, 3]);
    new_colors_flat = reshape(updated_map.colors, [h * w, 3]);
    new_normals_flat = reshape(updated_map.normals, [h * w, 3]);
    new_ccounts_flat = reshape(updated_map.ccounts, [h * w, 1]);
    new_times_flat = reshape(updated_map.times, [h * w, 1]);

    % get the unchanged points.
    unchanged_pts = reshape(fusion_map.pointcloud.Location(unchanged_mask_3d), [], 3);
    unchanged_colors = reshape(fusion_map.pointcloud.Color(unchanged_mask_3d), [], 3);
    unchanged_normals = reshape(fusion_map.normals(unchanged_mask_3d), [], 3);
    unchanged_ccounts = fusion_map.ccounts(unchanged_mask);
    unchanged_times = fusion_map.times(unchanged_mask);

   	% concatenate vertically.
    map_points = vertcat(unchanged_pts, new_pts_flat);
    map_normals = vertcat(unchanged_normals, new_normals_flat);
    map_colors = vertcat(unchanged_colors, new_colors_flat);
    map_ccounts = vertcat(unchanged_ccounts, new_ccounts_flat);
    map_times = vertcat(unchanged_times, new_times_flat);

    %==== Output the final point-based fusion map in a struct ====
    map_pointcloud = pointCloud(map_points, 'Color', map_colors);
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals, 'ccounts', map_ccounts, 'times', map_times);
      
end
   