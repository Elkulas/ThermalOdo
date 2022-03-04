close all;
clc;

img = imread(['47.png']);

if size(img,3) == 3
    img = rgb2gray(img);
end

a = im2double(img);

figure;
imshow(a);

[w, h] = size(img);

[U,E,V] = svd(a);
Ei = E;
E2 = E;
% calculate score 
sum = 0;
for i = 1:30
    sum = sum + E(i,i);
end

score = E(1, 1)/(sum);
% E1 img
for i = 2:(min(w, h))
    Ei(i,i) = 0;
end
m1 = U*Ei*V';
m1 = mat2gray(m1);
figure;
imshow(m1);
imwrite(m1, 'm1.png');



m2 = a - m1;

m2 = mat2gray(m2);
figure;
imshow(m2);
imwrite(m2, 'm2.png');

