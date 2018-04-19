clc;
clear;
close all;

addpath('/home/zhuorui/Documents/MATLAB/Visual-Localization-NSF-CPS/Background Recovery/Background Recovery Functions')

video_parent_dir = '/home/zhuorui/Documents/MATLAB/Visual-Localization-NSF-CPS/Dataset-10-24-2017/';
videoFormat = '.MOV';
videoScale = 0.25;
num_video_clip_frames = 200;
method = 'proximal_gradient_rpca';

videoName = 'IMG_0010';
background_recovery_general(video_parent_dir, videoName, videoFormat, videoScale, num_video_clip_frames, method);

videoName = 'IMG_0011';
background_recovery_general(video_parent_dir, videoName, videoFormat, videoScale, num_video_clip_frames, method);

videoName = 'IMG_0012';
background_recovery_general(video_parent_dir, videoName, videoFormat, videoScale, num_video_clip_frames, method);

videoName = 'IMG_0013';
background_recovery_general(video_parent_dir, videoName, videoFormat, videoScale, num_video_clip_frames, method);
