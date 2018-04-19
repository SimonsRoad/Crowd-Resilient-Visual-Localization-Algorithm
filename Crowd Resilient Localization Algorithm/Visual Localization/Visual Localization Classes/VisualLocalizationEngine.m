classdef VisualLocalizationEngine < handle
    %VISUALLOCALIZATIONENGINE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        % This is for SIS.
        parent_dir_environment = 'C:\Users\SIS\Documents\MATLAB\Computer Vision Project\Visual-Localization-NSF-CPS\Visual Localization\Environment Database';
        parent_dir_cameras = 'C:\Users\SIS\Documents\MATLAB\Computer Vision Project\Visual-Localization-NSF-CPS\Visual Localization\Camera Intrinsic Parameters Database';
        file_name_SIFT_descriptor = 'SIFT_Descriptor.mat';
        file_name_SIFT_normalized_descriptor = 'SIFT_normalized_descriptors_final.mat';
        file_name_SIFT_info = 'SIFT_Info.mat';
        %         file_name_SIFT_imageInfo = 'SIFT_Keys.mat';
        file_name_SIFT_space = 'SIFT_Space.mat';
        file_name_SIFT_color = 'SIFT_Color.mat';
        file_name_distortion_coefficients = 'Distortion Coefficients.mat';
        file_name_intrinsic_parameters = 'Intrinsic Parameters.mat';
        file_name_resolution = 'resolution.mat';
    end
    
    properties (SetAccess = private)
        % environmental information as global variable
        path_SFM;
        SIFT_descriptors_final;
        SIFT_info;
        %         reference_imageInfo;
        SIFT_space;
        SIFT_color;
        path_camera_intrinsicParamters;
        intrinsic_parameters;
        distortion_coefficients;
        resolution;
        envName;
        deviceName;
        original_width;
        original_height;
        space_matcher;
        dataCollector;
    end
    
    methods
        function engineObj = VisualLocalizationEngine(envName, deviceName)
            import Cameras.*
            import Environement.*
            
            engineObj.envName = envName;
            engineObj.deviceName = deviceName;
            
            switch deviceName
                case Cameras.Iphone_6s
                    engineObj.path_camera_intrinsicParamters = fullfile(VisualLocalizationEngine.parent_dir_cameras, deviceName);
                case Cameras.iphone6_video
                    engineObj.path_camera_intrinsicParamters = fullfile(VisualLocalizationEngine.parent_dir_cameras, deviceName);
                case Cameras.iphone6s_video
                    engineObj.path_camera_intrinsicParamters = fullfile(VisualLocalizationEngine.parent_dir_cameras, deviceName);
                case Cameras.Samsung_S3
                    engineObj.path_camera_intrinsicParamters = fullfile(VisualLocalizationEngine.parent_dir_cameras, deviceName);
                otherwise
                    error('No matched camera information about %s.\n', deviceName);
            end
            
            switch envName
                case Environment.Campus_Center
                    engineObj.path_SFM = fullfile(VisualLocalizationEngine.parent_dir_environment, envName);
                case Environment.campus_center_2017
                    engineObj.path_SFM = fullfile(VisualLocalizationEngine.parent_dir_environment, envName);
                case Environment.unknown
                    engineObj.path_SFM = fullfile(VisualLocalizationEngine.parent_dir_environment, envName);
                otherwise
                    error('No matched enviromental information about %s.\n', envName);
            end
        end
        
        % load the required data
        function load_environment(engineObj)
%             engineObj.SIFT_descriptors_final = importdata(fullfile(engineObj.path_SFM, VisualLocalizationEngine.file_name_SIFT_normalized_descriptor));
            engineObj.SIFT_descriptors_final = importdata(fullfile(engineObj.path_SFM, VisualLocalizationEngine.file_name_SIFT_descriptor));
            engineObj.SIFT_info = importdata(fullfile(engineObj.path_SFM, VisualLocalizationEngine.file_name_SIFT_info));
            %             engineObj.reference_imageInfo = importdata(fullfile(engineObj.path_SFM, VisualLocalizationEngine.reference_imageInfo));
            engineObj.SIFT_space = importdata(fullfile(engineObj.path_SFM, VisualLocalizationEngine.file_name_SIFT_space));
            engineObj.SIFT_color = importdata(fullfile(engineObj.path_SFM, VisualLocalizationEngine.file_name_SIFT_color));
        end
        
        function load_camera_intrinsic_paramters(engineObj)
            engineObj.intrinsic_parameters = importdata(fullfile(engineObj.path_camera_intrinsicParamters, VisualLocalizationEngine.file_name_intrinsic_parameters));
            engineObj.distortion_coefficients = importdata(fullfile(engineObj.path_camera_intrinsicParamters, VisualLocalizationEngine.file_name_distortion_coefficients));
            engineObj.distortion_coefficients = engineObj.distortion_coefficients';
            engineObj.resolution = importdata(fullfile(engineObj.path_camera_intrinsicParamters, VisualLocalizationEngine.file_name_resolution));
            engineObj.original_width = engineObj.resolution(1);
            engineObj.original_height = engineObj.resolution(2);
        end
        
        function prepare_matcher(engineObj)
            % For vmware
            %             addpath('C:\Users\Zhuorui Yang\Documents\opencv3.1_dev\mexopencv');
            %             addpath('C:\Users\Zhuorui Yang\Documents\opencv3.1_dev\mexopencv\opencv_contrib');
            % For SIS-lab
            addpath('C:\Users\SIS\Documents\opencv3.1-dev\mexopencv');
            addpath('C:\Users\SIS\Documents\opencv3.1-dev\mexopencv\opencv_contrib');
            engineObj.space_matcher = cv.DescriptorMatcher('FlannBased');
            for i = 1:size(engineObj.SIFT_descriptors_final, 1)
                % Normalize each input vector to unit length
                engineObj.SIFT_descriptors_final(i, :) = engineObj.SIFT_descriptors_final(i, :)/sqrt(sum(engineObj.SIFT_descriptors_final(i, :).^2));
            end
            engineObj.space_matcher.add(engineObj.SIFT_descriptors_final);
            engineObj.space_matcher.train();
            fprintf('Finished training the ANN structure for reference data.\n');
        end
        
        function prepare_matcher_normalized(engineObj)
            % For vmware
            %             addpath('C:\Users\Zhuorui Yang\Documents\opencv3.1_dev\mexopencv');
            %             addpath('C:\Users\Zhuorui Yang\Documents\opencv3.1_dev\mexopencv\opencv_contrib');
            % For SIS-lab
            addpath('C:\Users\SIS\Documents\opencv3.1-dev\mexopencv');
            addpath('C:\Users\SIS\Documents\opencv3.1-dev\mexopencv\opencv_contrib');
            engineObj.space_matcher = cv.DescriptorMatcher('FlannBased');
            engineObj.space_matcher.add(engineObj.SIFT_descriptors_final);
            engineObj.space_matcher.train();
            fprintf('Finished training the ANN structure for reference data.\n');
        end
        
        function [loc, orient] = visual_localize_solvePnP(engineObj, query_image)
            %% add opencv-related folder here.
            % For vmware
            %             addpath('C:\Users\Zhuorui Yang\Documents\opencv3.1_dev\mexopencv');
            %             addpath('C:\Users\Zhuorui Yang\Documents\opencv3.1_dev\mexopencv\opencv_contrib');
            % For SIS-lab
            addpath('C:\Users\SIS\Documents\opencv3.1-dev\mexopencv');
            addpath('C:\Users\SIS\Documents\opencv3.1-dev\mexopencv\opencv_contrib');
            % Get the resolution (width, heigh) of query image
            current_width = size(query_image, 2);
            current_height = size(query_image, 1);

            % copy and paste from visual odometry
            [keypoints, descriptors, ~] = featureExtract(query_image, 'method', 'opencv');
            SIFT_descriptors = descriptors;
            
            engineObj.dataCollector.sift_features_num = length(keypoints);
            
            locs = keypoints(:, 1:2);
            inlier = 0;
            
            % to check the epipolar geometry condition using fundanmental
            % matrix
            idxpool = containers.Map;
            idxinfo = containers.Map;
            info = [];
            
            space_matches = engineObj.space_matcher.knnMatch(SIFT_descriptors, 2);
            threshold_ratio = 0.8;
            
            for i = 1:size(SIFT_descriptors, 1)
                % Note: distance measured by FLANN is unknow and to be determined from data
                bestmatch = space_matches{1, i}(1,1);
                if bestmatch.distance < threshold_ratio*space_matches{1, i}(1,2).distance
                    inlier = inlier + 1;
                    for j = 1:size(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall, 2)
                        if ~isKey(idxpool, num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j)))
                            idxpool(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j))) = 1;
                            idxinfo(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j))) = [double(i); double(bestmatch.trainIdx+1); engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(3, j); engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(4, j)];
                            % the image coordinates system from sfm software is
                            % different, needs to confirm it later.
                        else
                            number = idxpool(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j)));
                            number = number + 1;
                            idxpool(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j))) = number;
                            info = idxinfo(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j)));
                            info = double([info, [double(i); double(bestmatch.trainIdx+1); engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(3, j); engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(4, j)]]);
                            idxinfo(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j))) = info;
                        end
                    end
                end
            end
            
            % Choose the image idx that with the most vote and choose the location of
            % feature point from it.
            fprintf('The number of inliers after distance threshold is %d.\n', inlier);
            engineObj.dataCollector.sift_matches_threshold_multiple_images = inlier;
            
            if inlier == 0
                fprintf('No result is obtained.\n');
                loc = [0; 0; 0];
                orient = eye(3);
                return;
            end
            
            KeySet = keys(idxpool);
            voting = intmin;
            for i = 1:size(KeySet, 2)
                if cell2mat(values(idxpool, KeySet(1, i))) > voting
                    voting = cell2mat(values(idxpool, KeySet(1, i)));
                    winner = str2double(cell2mat(KeySet(1, i)));
                end
            end
            
            fprintf('The voting result is %d.\n', winner);
            engineObj.dataCollector.matched_ref_image_id = winner;
            
            winnerinfo = idxinfo(num2str(winner));
            x1 = [];
            x2 = [];
            % here is the image coordinates change from sfm software to opencv,
            % double check this
            for i = 1:size(winnerinfo, 2)
                x1(:, i) = transpose(locs(winnerinfo(1, i), [1, 2])).*[1; -1] + [-(current_width - 1)/2; (current_height - 1)/2];
                x2(:, i) = ([double(winnerinfo(3, i)); double(winnerinfo(4, i))]);
                % fundamental matrix needs to work with homogeneous image
                % coordinates.
            end
            
            fprintf('The number of features on most single image is %d.\n', size(winnerinfo, 2));
            engineObj.dataCollector.sift_matches_single_image = size(winnerinfo, 2);
            
            point1 = cell(1, size(x1, 2));
            point2 = cell(1, size(x2, 2));
            for ff = 1:size(x1, 2)
                point1(1, ff) = {x1(:, ff)'};
                point2(1, ff) = {x2(:, ff)'};
            end
            if size(point1, 2) >= 8
                [F, mask] = cv.findFundamentalMat(point1, point2, 'Method', 'Ransac', 'Param1', 1.0, 'Param2', 0.99);
            else
                fprintf('No result is obtained.\n');
                loc = [0; 0; 0];
                orient = eye(3);
                return;
            end
            inliers = mask';
            
            convert_matrix = [current_width/engineObj.original_width, current_width/engineObj.original_width, current_width/engineObj.original_width; 0, current_height/engineObj.original_height, current_height/engineObj.original_height; 0, 0, 1];
            
            cameraMatrix = engineObj.intrinsic_parameters.*convert_matrix;
            %
            worldPoints = [];
            imagePoints = [];
            fundinlier = 0;
            for i = 1:size(inliers,2)
                if inliers(1, i) == 1
                    fundinlier = fundinlier + 1;
                    worldPoints(fundinlier, :) = engineObj.SIFT_space(winnerinfo(2, i), :).*[1, -1, -1];
                    imagePoints(fundinlier, :) = transpose(locs(winnerinfo(1, i), [1, 2]));
                end
            end
            
            fprintf('The number of inliers in Fundamental RANSAC is %d.\n', fundinlier);
            engineObj.dataCollector.sift_matches_epipolar_checked = fundinlier;
            
            if size(worldPoints, 1) >= 3
                [rvec, tvec, ~, poseinliers] = cv.solvePnPRansac(worldPoints, imagePoints, cameraMatrix, 'DistCoeffs', engineObj.distortion_coefficients, 'Method', 'Iterative');
                % todo: check why poseinliers is not shown properly.
            else
                fprintf('No result is obtained.\n');
                loc = zeros(3, 1);
                orient = eye(3);
                return;
            end
            
            fprintf('The number of inliers in pose estimation RANSAC is %d.\n', size(poseinliers,1));
            engineObj.dataCollector.solvepnp_inliers = size(poseinliers,1);
            
            R = cv.Rodrigues(rvec);
            R_transpose = transpose(R);
            loc = (-R_transpose)*tvec;
            orient = R_transpose*[0; 0; 1];
            
            fprintf('computation is done.\n');
        end
        
        function figureHandle = visualizeCameraPoses(engineObj, poses)
            % read through the poses and display them on top of the
            % 3D environment using bird view.
            figureHandle = figure;
            for i = 1:length(engineObj.SIFT_space)
                scatter(engineObj.SIFT_space(i, 1), -engineObj.SIFT_space(i, 3))
                hold on;
            end
            
            for i = 1:length(poses)
                pose = poses(i);
                plot(pose.loc(1), pose.loc(3), 'r*');
                quiver(pose.loc(1), pose.loc(3), pose.orient(1), pose.orient(3), 'b-', 'linewidth', 2);
            end
            hold off;
            
            title('Visualization of Camera Poses');
            xlabel('X axis in 3D world coordinate system');
            ylabel('Z axis in 3D world coordinate system');
        end
        
        function recordedData = get_recorded_data(engineObj)
            recordedData = engineObj.dataCollector;
        end
        
        function prepare_matcher_ICASSP(engineObj, filePath)
            % For vmware
            %             addpath('C:\Users\Zhuorui Yang\Documents\opencv3.1_dev\mexopencv');
            %             addpath('C:\Users\Zhuorui Yang\Documents\opencv3.1_dev\mexopencv\opencv_contrib');
            % For SIS-lab
            % overwritten the feature file
            engineObj.SIFT_descriptors_final = importdata(filePath);
            addpath('C:\Users\SIS\Documents\opencv3.1-dev\mexopencv');
            addpath('C:\Users\SIS\Documents\opencv3.1-dev\mexopencv\opencv_contrib');
            engineObj.space_matcher = cv.DescriptorMatcher('FlannBased');
            engineObj.space_matcher.add(engineObj.SIFT_descriptors_final);
            engineObj.space_matcher.train();
            fprintf('Finished training the ANN structure for reference data.\n');
        end
        
        function [loc, orient] = visual_localize_solvePnP_ICASSP(engineObj, query_image)
            %% add opencv-related folder here.
            % For vmware
            %             addpath('C:\Users\Zhuorui Yang\Documents\opencv3.1_dev\mexopencv');
            %             addpath('C:\Users\Zhuorui Yang\Documents\opencv3.1_dev\mexopencv\opencv_contrib');
            % For SIS-lab
            addpath('C:\Users\SIS\Documents\opencv3.1-dev\mexopencv');
            addpath('C:\Users\SIS\Documents\opencv3.1-dev\mexopencv\opencv_contrib');
            % Get the resolution (width, heigh) of query image
            current_width = size(query_image, 2);
            current_height = size(query_image, 1);

            % copy and paste from visual odometry
            [keypoints, descriptors, ~] = featureExtract(query_image, 'method', 'opencv');
            SIFT_descriptors = descriptors;
            
            engineObj.dataCollector.sift_features_num = length(keypoints);
            
            locs = keypoints(:, 1:2);
            inlier = 0;
            
            % to check the epipolar geometry condition using fundanmental
            % matrix
            idxpool = containers.Map;
            idxinfo = containers.Map;
            info = [];
            
            space_matches = engineObj.space_matcher.knnMatch(SIFT_descriptors, 2);
            threshold_ratio = 0.8;
            
            for i = 1:size(SIFT_descriptors, 1)
                % Note: distance measured by FLANN is unknow and to be determined from data
                bestmatch = space_matches{1, i}(1,1);
                if bestmatch.distance < threshold_ratio*space_matches{1, i}(1,2).distance
                    inlier = inlier + 1;
                    for j = 1:size(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall, 2)
                        if ~isKey(idxpool, num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j)))
                            idxpool(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j))) = 1;
                            idxinfo(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j))) = [double(i); double(bestmatch.trainIdx+1); engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(3, j); engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(4, j)];
                            % the image coordinates system from sfm software is
                            % different, needs to confirm it later.
                        else
                            number = idxpool(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j)));
                            number = number + 1;
                            idxpool(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j))) = number;
                            info = idxinfo(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j)));
                            info = double([info, [double(i); double(bestmatch.trainIdx+1); engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(3, j); engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(4, j)]]);
                            idxinfo(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j))) = info;
                        end
                    end
                end
            end
            
            % Choose the image idx that with the most vote and choose the location of
            % feature point from it.
            fprintf('The number of inliers after distance threshold is %d.\n', inlier);
            engineObj.dataCollector.sift_matches_threshold_multiple_images = inlier;
            
            if inlier == 0
                fprintf('No result is obtained.\n');
                loc = [0; 0; 0];
                orient = eye(3);
                return;
            end
            
            KeySet = keys(idxpool);
            voting = intmin;
            for i = 1:size(KeySet, 2)
                if cell2mat(values(idxpool, KeySet(1, i))) > voting
                    voting = cell2mat(values(idxpool, KeySet(1, i)));
                    winner = str2double(cell2mat(KeySet(1, i)));
                end
            end
            
            fprintf('The voting result is %d.\n', winner);
            engineObj.dataCollector.matched_ref_image_id = winner;
            
            winnerinfo = idxinfo(num2str(winner));
            x1 = [];
            x2 = [];
            % here is the image coordinates change from opencv software to sfm,
            % doesn't affect following computation since it is a one-time
            % change
            for i = 1:size(winnerinfo, 2)
                x1(:, i) = transpose(locs(winnerinfo(1, i), [1, 2])).*[1; -1] + [-(current_width - 1)/2; (current_height - 1)/2].*[1280/current_width; 853/current_height];
                x2(:, i) = ([double(winnerinfo(3, i)); double(winnerinfo(4, i))]);
                % fundamental matrix needs to work with homogeneous image
                % coordinates.
            end
            
            fprintf('The number of features on most single image is %d.\n', size(winnerinfo, 2));
            engineObj.dataCollector.sift_matches_single_image = size(winnerinfo, 2);
            
            point1 = cell(1, size(x1, 2));
            point2 = cell(1, size(x2, 2));
            for ff = 1:size(x1, 2)
                point1(1, ff) = {x1(:, ff)'};
                point2(1, ff) = {x2(:, ff)'};
            end
            if size(point1, 2) >= 8
                [~, mask] = cv.findFundamentalMat(point1, point2, 'Method', 'Ransac', 'Param1', 1.0, 'Param2', 0.99);
            else
                fprintf('No result is obtained.\n');
                loc = [0; 0; 0];
                orient = eye(3);
                return;
            end
            inliers = mask';
            
            convert_matrix = [current_width/engineObj.original_width, current_width/engineObj.original_width, current_width/engineObj.original_width; 0, current_height/engineObj.original_height, current_height/engineObj.original_height; 0, 0, 1];
            
            cameraMatrix = engineObj.intrinsic_parameters.*convert_matrix;
            %
            worldPoints = [];
            imagePoints = [];
            fundinlier = 0;
            for i = 1:size(inliers,2)
                if inliers(1, i) == 1
                    fundinlier = fundinlier + 1;
                    worldPoints(fundinlier, :) = engineObj.SIFT_space(winnerinfo(2, i), :).*[1, -1, -1];
                    imagePoints(fundinlier, :) = transpose(locs(winnerinfo(1, i), [1, 2]));
                end
            end
            
            fprintf('The number of inliers in Fundamental RANSAC is %d.\n', fundinlier);
            engineObj.dataCollector.sift_matches_epipolar_checked = fundinlier;
            
            if size(worldPoints, 1) >= 3
                [rvec, tvec, ~, poseinliers] = cv.solvePnPRansac(worldPoints, imagePoints, cameraMatrix, 'DistCoeffs', engineObj.distortion_coefficients, 'Method', 'Iterative');
                % todo: check why poseinliers is not shown properly.
            else
                fprintf('No result is obtained.\n');
                loc = zeros(3, 1);
                orient = eye(3);
                return;
            end
            
            fprintf('The number of inliers in pose estimation RANSAC is %d.\n', size(poseinliers,1));
            engineObj.dataCollector.solvepnp_inliers = size(poseinliers,1);
            
            R = cv.Rodrigues(rvec);
            R_transpose = transpose(R);
            loc = (-R_transpose)*tvec;
            orient = R_transpose*[0; 0; 1];
            
            fprintf('computation is done.\n');
        end
        
        function [loc, orient] = visual_localize_solvePnP_Sensor_Fusion(engineObj, query_image)
            %% add opencv-related folder here.
            % For vmware
            %             addpath('C:\Users\Zhuorui Yang\Documents\opencv3.1_dev\mexopencv');
            %             addpath('C:\Users\Zhuorui Yang\Documents\opencv3.1_dev\mexopencv\opencv_contrib');
            % For SIS-lab
            addpath('C:\Users\SIS\Documents\opencv3.1-dev\mexopencv');
            addpath('C:\Users\SIS\Documents\opencv3.1-dev\mexopencv\opencv_contrib');
            % Get the resolution (width, heigh) of query image
            current_width = size(query_image, 2);
            current_height = size(query_image, 1);

            % copy and paste from visual odometry
            [keypoints, descriptors, ~] = featureExtract(query_image, 'method', 'opencv');
            SIFT_descriptors = descriptors;
            
%             engineObj.dataCollector.sift_features_num = length(keypoints);
            
            locs = keypoints(:, 1:2);
            inlier = 0;
            
            % to check the epipolar geometry condition using fundanmental
            % matrix
            idxpool = containers.Map;
            idxinfo = containers.Map;
            info = [];
            
            space_matches = engineObj.space_matcher.knnMatch(SIFT_descriptors, 2);
            threshold_ratio = 0.8;
            
            for i = 1:size(SIFT_descriptors, 1)
                % Note: distance measured by FLANN is unknow and to be determined from data
                bestmatch = space_matches{1, i}(1,1);
                if bestmatch.distance < threshold_ratio*space_matches{1, i}(1,2).distance
                    inlier = inlier + 1;
                    for j = 1:size(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall, 2)
                        if ~isKey(idxpool, num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j)))
                            idxpool(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j))) = 1;
                            idxinfo(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j))) = [double(i); double(bestmatch.trainIdx+1); engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(3, j); engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(4, j)];
                            % the image coordinates system from sfm software is
                            % different, needs to confirm it later.
                        else
                            number = idxpool(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j)));
                            number = number + 1;
                            idxpool(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j))) = number;
                            info = idxinfo(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j)));
                            info = double([info, [double(i); double(bestmatch.trainIdx+1); engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(3, j); engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(4, j)]]);
                            idxinfo(num2str(engineObj.SIFT_info(1, bestmatch.trainIdx+1).overall(1, j))) = info;
                        end
                    end
                end
            end
            
            % Choose the image idx that with the most vote and choose the location of
            % feature point from it.
            fprintf('The number of inliers after distance threshold is %d.\n', inlier);
%             engineObj.dataCollector.sift_matches_threshold_multiple_images = inlier;
            
            if inlier == 0
                fprintf('No result is obtained.\n');
                loc = [0; 0; 0];
                orient = eye(3);
                return;
            end
            
            KeySet = keys(idxpool);
            voting = intmin;
            for i = 1:size(KeySet, 2)
                if cell2mat(values(idxpool, KeySet(1, i))) > voting
                    voting = cell2mat(values(idxpool, KeySet(1, i)));
                    winner = str2double(cell2mat(KeySet(1, i)));
                end
            end
            
            fprintf('The voting result is %d.\n', winner);
%             engineObj.dataCollector.matched_ref_image_id = winner;
            
            winnerinfo = idxinfo(num2str(winner));
            x1 = [];
            x2 = [];
            % here is the image coordinates change from opencv software to sfm,
            % doesn't affect following computation since it is a one-time
            % change
            for i = 1:size(winnerinfo, 2)
                x1(:, i) = transpose(locs(winnerinfo(1, i), [1, 2])).*[1; -1] + [-(current_width - 1)/2; (current_height - 1)/2].*[1280/current_width; 853/current_height];
                x2(:, i) = ([double(winnerinfo(3, i)); double(winnerinfo(4, i))]);
                % fundamental matrix needs to work with homogeneous image
                % coordinates.
            end
            
            fprintf('The number of features on most single image is %d.\n', size(winnerinfo, 2));
%             engineObj.dataCollector.sift_matches_single_image = size(winnerinfo, 2);
            
            point1 = cell(1, size(x1, 2));
            point2 = cell(1, size(x2, 2));
            for ff = 1:size(x1, 2)
                point1(1, ff) = {x1(:, ff)'};
                point2(1, ff) = {x2(:, ff)'};
            end
            if size(point1, 2) >= 8
                [~, mask] = cv.findFundamentalMat(point1, point2, 'Method', 'Ransac', 'Param1', 1.0, 'Param2', 0.99);
            else
                fprintf('No result is obtained.\n');
                loc = [0; 0; 0];
                orient = eye(3);
                return;
            end
            inliers = mask';
            
            convert_matrix = [current_width/engineObj.original_width, current_width/engineObj.original_width, current_width/engineObj.original_width; 0, current_height/engineObj.original_height, current_height/engineObj.original_height; 0, 0, 1];
            
            cameraMatrix = engineObj.intrinsic_parameters.*convert_matrix;
            %
            worldPoints = [];
            imagePoints = [];
            fundinlier = 0;
            for i = 1:size(inliers,2)
                if inliers(1, i) == 1
                    fundinlier = fundinlier + 1;
                    worldPoints(fundinlier, :) = engineObj.SIFT_space(winnerinfo(2, i), :).*[1, 1, 1];
                    imagePoints(fundinlier, :) = transpose(locs(winnerinfo(1, i), [1, 2]));
                end
            end
            
            fprintf('The number of inliers in Fundamental RANSAC is %d.\n', fundinlier);
%             engineObj.dataCollector.sift_matches_epipolar_checked = fundinlier;
            
            if size(worldPoints, 1) >= 3
                [rvec, tvec, ~, poseinliers] = cv.solvePnPRansac(worldPoints, imagePoints, cameraMatrix, 'DistCoeffs', engineObj.distortion_coefficients, 'Method', 'Iterative');
                % todo: check why poseinliers is not shown properly.
            else
                fprintf('No result is obtained.\n');
                loc = zeros(3, 1);
                orient = eye(3);
                return;
            end
            
            fprintf('The number of inliers in pose estimation RANSAC is %d.\n', size(poseinliers,1));
%             engineObj.dataCollector.solvepnp_inliers = size(poseinliers,1);
            if ~isempty(poseinliers)
                R = cv.Rodrigues(rvec);
                R_transpose = transpose(R);
                loc = (-R_transpose)*tvec;
                orient = R_transpose*[0; 0; 1];
                engineObj.dataCollector.R_W2B = R;
                engineObj.dataCollector.Q_W2B = rotm2quat(R);
                engineObj.dataCollector.R_B2W = R';
                engineObj.dataCollector.Q_B2W = rotm2quat(R');
                engineObj.dataCollector.t_W2B = tvec;
            else
                fprintf('No result is obtained.\n');
                loc = zeros(3, 1);
                orient = eye(3);
                return;
            end
            fprintf('computation is done.\n');
        end
    end
end

