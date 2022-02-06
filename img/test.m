close all;
clc;

I = imread('1.png');

I2  = I(:, :, 1);

I3 = im2double(I2);

[mdat, mpos] = max(I(:));

[mdat2, mpos2] = max(I2(:));

% tic;
J = svd_mean_recompute_denoise_first_eigen_zero(I);

[mdat3, mpos3] = max(J(:));


% toc;
imshow(J);
imwrite(J, '1-equ.png');