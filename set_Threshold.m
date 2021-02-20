function Threshold = set_Threshold()
% Set Threshold
%===================================================

% Copy params from mapper2d_params.h
% Default values 
    % GRAPH parameters
    closures_inlier_threshold = 3.0;
    closures_window = 10;
    closures_min_inliers = 6;
    lcaligner_inliers_distance = 0.05;
    lcaligner_min_inliers_ratio = 0.5;
    lcaligner_min_num_correspondences = 50;
    num_matching_beams = 0.0;
    matching_fov = 0.0;

    % MAPPER parameters
    verbose = false;
    log_times = false;
    use_merger = false;
    Threshold.vertex_translation = 0.5;
    Threshold.vertex_rotation = pi/4;
    
%==========================================================================