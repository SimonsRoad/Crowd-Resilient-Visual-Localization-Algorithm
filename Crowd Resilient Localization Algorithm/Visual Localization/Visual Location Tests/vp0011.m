clc;
clear;
close all;

addpath(genpath('C:\Users\SIS\Documents\MATLAB\Computer Vision Project\Visual-Localization-NSF-CPS\Visual Localization\Visual Localization Classes'));

dir_generated_data = 'C:\Users\SIS\Documents\MATLAB\Computer Vision Project\Visual-Localization-NSF-CPS\Background Recovery\Generated Data\result-IMG-0011';
listing = dir(dir_generated_data);
pattern_seg_number_IMG = '[seg_0-9_IMG]';
pattern_seg_number_underline = '_[0-9]([0-9])?_';
pattern_seg_number = '[0-9]([0-9])?';

poses_original_total = {};
poses_background_recovered_total = {};

engine = VisualLocalizationEngine('Campus_Center', 'Iphone_6s');
engine.load_environment();
engine.load_camera_intrinsic_paramters();
engine.prepare_matcher_ICASSP('C:\Users\SIS\Documents\MATLAB\Computer Vision Project\Visual-Localization-NSF-CPS\Extra Result Analysis\featuresOnImageId69.mat');

for i = 1:length(listing)
    file = listing(i);
    fileName = file.name;
    seg_num_IMG = regexp(fileName, pattern_seg_number_IMG, 'match');
    if isempty(seg_num_IMG)
        continue;
    end
    seg_number_underline = regexp(num2str(cell2mat(seg_num_IMG)), pattern_seg_number_underline, 'match');
    seg_num = regexp(num2str(cell2mat(seg_number_underline)), pattern_seg_number, 'match');
    seg_num = str2double(seg_num);
    %% localize 200 frames that have been background recovered
    if strfind(fileName, 'background_recovered')
        vr_br = VideoReader(fullfile(dir_generated_data, fileName));
        poses_original_videoClip = ([]);
        while hasFrame(vr_br)
            videoFrame = readFrame(vr_br);
            [loc, orient] = engine.visual_localize_solvePnP_ICASSP(videoFrame);
            pose_background_original.loc = loc;
            pose_background_original.orient = orient;
            pose_background_original.data = engine.get_recorded_data();
            poses_original_videoClip = [poses_original_videoClip, pose_background_original];
        end
        poses_original_total{seg_num+1} = poses_original_videoClip;
%         h = engine.visualizeCameraPoses(poses_original_videoClip);
        %% localize 200 frames that are not background recovered.
    elseif isempty(strfind(fileName, 'foreground_detected'))
        vr = VideoReader(fullfile(dir_generated_data, fileName));
        poses_background_recovered_videoClip = ([]);
        while hasFrame(vr)
            videoFrame = readFrame(vr);
            [loc, orient] = engine.visual_localize_solvePnP_ICASSP(videoFrame);
            pose_background_recovered.loc = loc;
            pose_background_recovered.orient = orient;
            pose_background_recovered.data = engine.get_recorded_data();
            poses_background_recovered_videoClip = [poses_background_recovered_videoClip, pose_background_recovered];
        end
        poses_background_recovered_total{seg_num+1} = poses_background_recovered_videoClip;
%         h = engine.visualizeCameraPoses(poses_background_recovered_videoClip);
    end
    
end
%% save the result
save('IMG0011_poses_imageId69.mat', 'poses_original_total', 'poses_background_recovered_total');









