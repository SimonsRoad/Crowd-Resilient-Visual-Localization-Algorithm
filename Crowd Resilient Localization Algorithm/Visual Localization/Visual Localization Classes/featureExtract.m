function [keypoints, descriptors, flag] = featureExtract( query_image, option_name, option )
%FEATUREEXTRACTION Summary of this function goes here
%   Detailed explanation goes here
% INPUT:    
% image is the RGB image
%
% option_name is 'method'
%
% option is the choice of option_name, either 'opencv' or 'lowe'
%
% OUTPUT:
%
% desctiptor is in N*128, N is the number of the feature detected on input
% image

   opencv_flag = false;
   lowe_flag = false;

    if nargin < 3
        error('not enough inputs');
    elseif nargin == 3
        if strcmp(option_name, 'method')
            if strcmp(option, 'opencv')
                opencv_flag = true;
            elseif strcmp(option, 'lowe')
                lowe_flag = true;  
            else
                error('no such option existed');
            end
        else
            error('2nd input should be method');
        end
    else
        error('too many inputs');
    end

    if opencv_flag
        % OpenCV SIFT
        src = query_image;
        dst = cv.cvtColor(src, 'RGB2GRAY');
        SIFTobj = cv.SIFT();
        [keypoints, descriptors] = SIFTobj.detectAndCompute(dst);
        % The image coordinates system in OpenCV is top left is (0, 0)
        kpts = zeros(size(descriptors, 1), 2);
        for i = 1:size(descriptors, 1)
            % Normalize each input vector to unit length
            descriptors(i, :) = descriptors(i, :)/sqrt(sum(descriptors(i, :).^2));
            kpts(i, :) = keypoints(i).pt;
        end
        keypoints = kpts;
        fprintf('Found %d features from opencv sift.\n', size(descriptors, 1));
        flag = 'opencv';
    else
        %% not working after modification 4/18/17
        dir = pwd;
        cd('siftDemoV4');
        % The image coordinates system in Matlab is top left is (1, 1)
        % In this Matlab simulation environment, the homogeneous
        % coordinates system will be OPENCV on. so shift X, Y by 1 pixel in
        % the following operation.
        [image, descriptors, keypoints] = sift(query);
        % the location of keypoints is stored as [Y, X, orientation,
        % response] originially from SIFTdemoV4
        % put it in the homogeneous order as [X, Y]
        keypoints = [keypoints(:, 2) - 1, keypoints(:, 1) - 1];
        flag = 'lowe';
        fprintf('Found %d features from lowe sift.\n', size(descriptors, 1));
        cd(dir);
    end
        
end

