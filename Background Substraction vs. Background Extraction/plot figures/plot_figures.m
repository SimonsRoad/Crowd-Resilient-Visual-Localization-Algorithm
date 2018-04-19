% visualize the preprocessing_comparison data

clc;
clear;
close all;

window_size = 10;
num_groupofdata = 37+26;
load('SIFTmatching_comparison_all200frames.mat');
num_frame = size(num_SIFTfeature,3);
num_windows = floor(num_frame/window_size);

total_sift_features = zeros(num_windows,3,2);
matched_sift_features = zeros(num_windows,3,2);

% 4 is for 50th frame, 100th frame, 150th frame, 200th frame.
% 26 is the number of the data(video)

for j = 1:num_windows
    
    matched_sift_features(j,:,1) = mean(mean(num_SIFTmatch(1:3,1:26,(j-1)*window_size+(1:window_size)),3),2)';
    matched_sift_features(j,:,2) = mean(mean(num_SIFTmatch(1:3,27:end,(j-1)*window_size+(1:window_size)),3),2)';
    
    total_sift_features(j,:,1) = mean(mean(num_SIFTfeature(1:3,1:26,(j-1)*window_size+(1:window_size)),3),2)';
    total_sift_features(j,:,2) = mean(mean(num_SIFTfeature(1:3,27:end,(j-1)*window_size+(1:window_size)),3),2)';

end

co = [0 0 1;
      0 0.5 0;
      1 0 0;
      0 0.75 0.75;
      0.75 0 0.75;
      0.75 0.75 0;
      0.25 0.25 0.25];
set(groot,'defaultAxesColorOrder',co)

figure,plot(window_size*(1:num_windows)',total_sift_features(:,:,1),'LineWidth',2),axis tight
% axisfortex('','Frame Index','Number of Extracted SIFT Features',20)
xlabel('Frame Index', 'FontSize', 20);
ylabel('Number of Extracted SIFT Features', 'FontSize', 20);
legend('Original Image','FuzzySOB','RPCA','Location','SouthWest')
figure,plot(window_size*(1:num_windows)',matched_sift_features(:,:,1),'LineWidth',2),axis tight
% axisfortex('','Frame Index','Number of Matched SIFT Features',20)
xlabel('Frame Index', 'FontSize', 20);
ylabel('Number of Mathced SIFT Features', 'FontSize', 20);
lgd = legend('Original Image','FuzzySOB','RPCA','Location','SouthWest');
lgd.FontSize = 15;
figure,plot(window_size*(1:num_windows)',100*matched_sift_features(:,:,1)./total_sift_features(:,:,1),'LineWidth',2),axis tight
% axisfortex('','Frame Index','Percentage of Matched SIFT Features',20)
xlabel('Frame Index', 'FontSize', 20);
ylabel('Percentage of Matched SIFT Features', 'FontSize', 20);
lgd = legend('Original Image','FuzzySOB','RPCA','Location','SouthWest');
lgd.FontSize = 15;

figure,plot(window_size*(1:num_windows)',total_sift_features(:,:,2),'--','LineWidth',2),axis tight
% axisfortex('','Frame Index','Number of Extracted SIFT Features',20)
xlabel('Frame Index', 'FontSize', 20);
ylabel('Number of Extracted SIFT Features', 'FontSize', 20);
legend('Original Image','FuzzySOB','RPCA','Location','SouthWest')
figure,plot(window_size*(1:num_windows)',matched_sift_features(:,:,2),'--','LineWidth',2),axis tight
% axisfortex('','Frame Index','Number of Matched SIFT Features',20)
xlabel('Frame Index', 'FontSize', 20);
ylabel('Number of Mathced SIFT Features', 'FontSize', 20);
lgd = legend('Original Image','FuzzySOB','RPCA','Location','SouthWest');
lgd.FontSize = 15;
figure,plot(window_size*(1:num_windows)',100*matched_sift_features(:,:,2)./total_sift_features(:,:,2),'--','LineWidth',2),axis tight
% axisfortex('','Frame Index','Percentage of Matched SIFT Features',20)
xlabel('Frame Index', 'FontSize', 20);
ylabel('Percentage of Matched SIFT Features', 'FontSize', 20);
lgd = legend('Original Image','FuzzySOB','RPCA','Location','SouthWest');
lgd.FontSize = 15;

print -dpng -f2 SIFT-Count-Medium.png
print -dpng -f3 SIFT-Percent-Medium.png
print -dpng -f5 SIFT-Count-Large.png
print -dpng -f6 SIFT-Percent-Large.png