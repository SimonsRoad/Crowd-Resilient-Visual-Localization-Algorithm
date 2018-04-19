function background_recovery_general( video_parent_dir, videoName, videoFormat, videoScale, num_video_clip_frames, method )
%BACKGROUND_RECOVERY_GENERAL Summary of this function goes here
%   generate the tailored video clips in avi format without background recovery
%   generate the tailored video clips in avi format with background recovery

addpath(genpath('/home/zhuorui/Documents/MATLAB/Visual-Localization-NSF-CPS/Background Recovery/RPCA'));
%% generate tailored videos according to the input about requirement
vr = VideoReader(strcat(video_parent_dir, videoName, videoFormat));
num_frames_video = vr.NumberOfFrames;
fprintf('This video has %d frames.\n', num_frames_video);
vr = VideoReader(strcat(video_parent_dir, videoName, videoFormat));


videoClips = cell(1, ceil(num_frames_video/num_video_clip_frames));
frameCounter = 0;
while hasFrame(vr)
    if mod(frameCounter, num_video_clip_frames) == 0
        if num_frames_video - frameCounter >= num_video_clip_frames
            video_clip_name = strcat('seg_', num2str(floor(frameCounter/num_video_clip_frames)), '_', videoName, '.avi');
        else
            video_clip_name = strcat('seg_', num2str(ceil(frameCounter/num_video_clip_frames)), '_', videoName, '.avi');
        end
        vw = VideoWriter(video_clip_name);
        open(vw);
        videoFramesMat = zeros(vr.Height*videoScale, vr.Width*videoScale, num_video_clip_frames);
    end
    
    frame = readFrame(vr);
    resized_gray_frame = imresize(rgb2gray(frame), videoScale);
    writeVideo(vw, resized_gray_frame);
    frameCounter = frameCounter+1;
    if mod(frameCounter, num_video_clip_frames) == 0
        videoFramesMat(:, :, mod(frameCounter, num_video_clip_frames)+num_video_clip_frames) = resized_gray_frame;
    else
        videoFramesMat(:, :, mod(frameCounter, num_video_clip_frames)) = resized_gray_frame;
    end
    
    if mod(frameCounter, num_video_clip_frames) == 0
        close(vw);
        videoClips{floor(frameCounter/num_video_clip_frames)} = videoFramesMat;
    end
end
if mod(frameCounter, num_video_clip_frames) ~= 0
    close(vw);
    videoClips{ceil(frameCounter/num_video_clip_frames)} = videoFramesMat;
end

%% convert 3D matrix of videoClips to the format RPCA required
Ds = cell(1, length(videoClips));
for i = 1:length(videoClips)
    videoClip = videoClips{i};
    D = zeros(size(videoClip, 1)*size(videoClip, 2), num_video_clip_frames);
    for j = 1:size(videoClip, 3)
        frame = reshape(videoClip(:, :, j), [size(videoClip, 1)*size(videoClip, 2), 1]);
        D(:, j) = frame;
    end
    Ds{i} = D;
end

%% recover the background with selective method and save the results
for i = 1:length(Ds)
    D = Ds{i};
    A_hat_3d = zeros(size(videoClip, 1), size(videoClip, 2), num_video_clip_frames);
    E_hat_3d = zeros(size(videoClip, 1), size(videoClip, 2), num_video_clip_frames);
    br_video_clip_name = strcat('seg_', num2str(i-1), '_', videoName, '_background_recovered.avi');
    fg_video_clip_name = strcat('seg_', num2str(i-1), '_', videoName, '_foreground_detected.avi');
    vw_br = VideoWriter(br_video_clip_name);
    open(vw_br);
    vw_fd = VideoWriter(fg_video_clip_name);
    open(vw_fd);
    if strcmp(method, 'proximal_gradient_rpca')
        [A_hat, E_hat, numIter] = proximal_gradient_rpca(D, 1/sqrt(size(D, 1)), 10000, 1e-7, 0, 1,  0, 1e-3);
    else
        % blablabla
    end
    
    for j = 1:num_video_clip_frames
        A_hat_3d(:, :, j) = reshape(A_hat(:, j), [size(videoClip, 1), size(videoClip, 2)]);
        E_hat_3d(:, :, j) = reshape(E_hat(:, j), [size(videoClip, 1), size(videoClip, 2)]);
        writeVideo(vw_br, mat2gray(A_hat_3d(:, :, j)));
        writeVideo(vw_fd, mat2gray(E_hat_3d(:, :, j)));
    end
    
    close(vw_br);
    close(vw_fd);
end

end

