clc;
clear;
close all;

addpath('C:\Users\SIS\Documents\MATLAB\Computer Vision Project\Visual-Localization-NSF-CPS\Visual Localization\Environment Database\Campus_Center');
features = importdata('SIFT_normalized_descriptors_final.mat');
images = importdata('SIFT_keys.mat');
correspondences = importdata('SIFT_Info.mat');

video_image_id = 9;
numOfFeatures = 0;
for i = 1:length(correspondences)
    featureOnImage = false;
    for j = 1:size(correspondences(i).overall, 2)
        if correspondences(i).overall(1, j)==video_image_id
            featureOnImage = true;
            numOfFeatures = numOfFeatures+1;
            break;
        end
    end
    if ~featureOnImage
        features(i, :) = features(i, :)*100;
    end
end

save('featuresOnImageId9.mat', 'features');
