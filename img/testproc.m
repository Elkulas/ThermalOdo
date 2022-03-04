close all;
clc;

img = imread(['36' ...
    '.png']);

if size(img,3) == 3
    img = rgb2gray(img);
end

a = im2double(img);

a = imnlmfilt(a, ...
              'ComparisonWindowSize', 3, ...
              'SearchWindowSize',21);
a = medfilt2(a,[3 3]);

a= mat2gray(a);

imshow(a);
imwrite(a, 'a.png');