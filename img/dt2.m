clear
clc
%% 路径加载
org_path = "./data/";
img_path = org_path + "thermal";
save_name_path = org_path + "times.txt";
%% 数据加载
img_reader = imageDatastore(img_path, 'IncludeSubfolders', true);
txt_reader = importdata(save_name_path);

for i = 1:length(img_reader.Files)
%     image_n = data{i,1};
%     image_chen_show = readImage(image_n);
    image_chen_show = imread(img_reader.Files{i});
    name_cache = txt_reader{i};
    name = name_cache(1:17);
%     path_num = num2str(counter,16);
%     if length(path_num) < 17
%         for i = 1:(17-length(path_num))
%             path_num = path_num + "0";
%         end
%     end
%     res_path = "./res/ORG/" + path_num + ".png";
%     oo = mat2gray(image_chen_show);
%     imwrite(oo,res_path);
    % SVD Direct
%     m_svd = svd_denoise(image_chen_show);
%     m_svd = final_denoise(m_svd,3,21,2);
%     res_path_SVD = "./res2/SVD/" + name + ".png";
%     imwrite(m_svd,res_path_SVD);
    
%     m_mean_svd = svd_mean_denoise(image_chen_show);
%     m_mean_svd = final_denoise(m_mean_svd,3,21,2);
%     res_path_mean_SVD = "./res/mean_SVD/" + name + ".png";
%     imwrite(m_mean_svd,res_path_mean_SVD);
    
    m_recompute_svd_zero = svd_mean_recompute_denoise_first_eigen_zero(image_chen_show);
    m_recompute_svd_zero = final_denoise(m_recompute_svd_zero,3,21,2);
    res_path_recompute_SVD_zero = "./res/" + name + ".png";
    imwrite(m_recompute_svd_zero,res_path_recompute_SVD_zero); 
    
    
%     m_recompute_svd_unzero = svd_mean_recompute_denoise_first_eigen_unzero(image_chen_show);
%     m_recompute_svd_unzero = final_denoise(m_recompute_svd_unzero,3,21,2);
%     res_path_recompute_SVD_unzero = "./res16/recompute_SVD_unzero/" + name + ".png";
%     imwrite(m_recompute_svd_unzero,res_path_recompute_SVD_unzero);   
%     counter = counter + 0.033037;
end
%% test part
% q = imread("./" + num2str(9) + ".png");

image_n = data{1,1};
q = readImage(image_n);

q = q(:,:,1);

p_1 = svd_denoise(q);
p_1 = final_denoise(p_1,3,21,2);

p_2 = svd_mean_denoise(q);
p_2 = final_denoise(p_2,3,21,2);

p_3 = svd_mean_recompute_denoise_first_eigen_unzero(q);
p_3 = final_denoise(p_3,3,21,2);

p_4 = svd_mean_recompute_denoise_first_eigen_zero(q);
p_4 = final_denoise(p_4,3,21,2);

imshow(p_1, [])
% p = imtile(p_1, p_2,'GridSize',[1 3]);
% imshow(p, [])
p_1 = mat2gray(p_1);
imwrite(p_1,'./res2/1.png')
%% 去噪函数
function m = svd_denoise(img)
% *****************************************
% Set the first eigenval zero!
% *****************************************
if size(img,3) == 3
    img = img(:,:,1); 
end
a = im2double(img);
[U,E,V] = svd(a);
E(1,1) = 0;
m = U*E*V';
end

function m = svd_mean_denoise(img)
% *****************************************
% Set the first eigenval zero!
% Assign each eigenval into eigen matrix !
% *****************************************
if size(img,3) == 3
    img = img(:,:,1); 
end
a = im2double(img);
[U,E,V] = svd(a);

E(1,1) = 0;
% mean the eigen value
mean_val = 0;
for i = 1:480
    mean_val = mean_val + E(i,i);
end
mean_val = mean_val/480;
% rebuild eigen matrix 
for i = 1:480
    E(i,i) = mean_val;
end
% rebuild image
m = U*E*V';
end

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
[U,E,V] = svd(a);

E(1,1) = 0;
% mean the eigen value
mean_val = 0;
for i = 1:480
    mean_val = mean_val + E(i,i);
end
mean_val = mean_val/480;
% rebuild eigen matrix 
for i = 1:480
    E(i,i) = mean_val + (480-i+1);
end

m = U*E*V';



end

function m = svd_mean_recompute_denoise_first_eigen_unzero(img)
% ***************************************************
% Assign each eigenval into eigen matrix !
% Change eigen value in decend with index increasing
% ***************************************************
if size(img,3) == 3
    img = img(:,:,1); 
end
a = im2double(img);
[U,E,V] = svd(a);

% mean the eigen value
mean_val = 0;
for i = 1:480
    mean_val = mean_val + E(i,i);
end
mean_val = mean_val/480;
% rebuild eigen matrix 
for i = 1:480
    E(i,i) = mean_val + (480-i+1);
end

m = U*E*V';
end

function m = final_denoise(img, ComparisonWindowSize, ...
                           SearchWindowSize, midsize)
p = imnlmfilt(img, ...
              'ComparisonWindowSize', ComparisonWindowSize, ...
              'SearchWindowSize',SearchWindowSize);
m = medfilt2(p,[midsize midsize]);
m = m+100;
m = mat2gray(m, [0, 200]);
end