function m = svd_mean_recompute_denoise_first_eigen_zero(img)
% ***************************************************
% Set the first eigenval zero!
% Assign each eigenval into eigen matrix !
% Change eigen value in decend with index increasing
% ***************************************************
if size(img,3) == 3
    img = img(:,:,1); 
end
a = im2double(img);
tic;
[U,E,V] = svd(a);
toc;

E(1,1) = 0;
% mean the eigen value
mean_val = 0;
for i = 1:480
    mean_val = mean_val + E(i,i);
end
mean_val = mean_val/480;
tic;
% rebuild eigen matrix 
for i = 1:480
    E(i,i) = mean_val + (480-i+1);
end
toc;

mi = U*E*V';

mi = imnlmfilt(mi, ...
              'ComparisonWindowSize', 3, ...
              'SearchWindowSize',11);
m = medfilt2(mi,[2 2]);
m = m+100;
m = mat2gray(m, [0, 200]);

end