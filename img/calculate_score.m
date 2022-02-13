close all;
clc;

img = imread(['21.png']);

if size(img,3) == 3
    img = rgb2gray(img);
end

a = im2double(img);

figure;
imshow(a);

[w, h] = size(img);

[U,E,V] = svd(a);
Ei = E;
% calculate score 
sum = 0;
for i = 1:30
    sum = sum + E(i,i);
end

score = E(1, 1)/(sum);


% zero image
E(1, 1) = 0;
mzero = U*E*V';
mzero = mat2gray(mzero);
figure;
imshow(mzero);
imwrite(mzero, 'zero.png');

% mean the eigen value
mean_val = 0;
for i = 1:(min(w, h))
    mean_val = mean_val + E(i,i);
end
mean_val = mean_val/w;

% rebuild eigen matrix 
for i = 1:min(w, h)
    E(i,i) = mean_val + (w-i+1);
end


mi = U*E*V';
mi2 = mi + 100;
mishow = mat2gray(mi2, [0, 200]);
figure;
imshow(mishow);
imwrite(mishow, 'rebuild.png');

mi = imnlmfilt(mi, ...
              'ComparisonWindowSize', 3, ...
              'SearchWindowSize',21);
m = medfilt2(mi,[2 2]);

m = m+100;
m = mat2gray(m, [0, 200]);


figure;
imshow(m);
imwrite(m, 'denoise.png');



