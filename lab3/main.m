clc

% read raw images
images = cell(1, 6);
for i = 1 : 6
    image_name = strcat('image', num2str(i), '.jpg');
    images{1, i} = imread(image_name);
%     figure, imshow(images[50, 50, 300, 300]{1, i});
end

%colorize and aligning images
for i =1 : 6
    raw_image = images{1,i};
    double_image = im2double(raw_image);
    height = floor(size(double_image) / 3);
    % split 3 color channels
    blue_channel = double_image(1:height, : );
    green_channel = double_image(height * 1 + 1 : height * 2, :);
    red_channel = double_image(height * 2 + 1 : height * 3, :);
    
    % combine 3 channels to form a colorized image without alignment
    color_image = cat(3, red_channel, green_channel, blue_channel);
    figure, imshow(color_image);
    image_name_color = strcat('image', num2str(i), '-color.jpg');
    imwrite(color_image, image_name_color);
       
    % SSD
    % align red channel to blue channel
    align_vector_r = im_align1(red_channel, blue_channel, i);
    r_b = circshift(red_channel, align_vector_r);
    % align green channel to blue channel
    align_vector_g = im_align1(green_channel, blue_channel, i);
    g_b = circshift(green_channel, align_vector_g);
    fprintf('image%d SSD calculated alignment:\n', i);
    fprintf('r -> b:(%d, %d)\t g -> b:(%d, %d)\n ', align_vector_r, align_vector_g);
    % show color image aligning by ssd
    ssd_color_image = cat(3, r_b, g_b, blue_channel);
    figure, imshow(ssd_color_image);
    ssd_image = strcat('image', num2str(i), '-ssd.jpg');
    imwrite(ssd_color_image, ssd_image)


    % NCC 
    % align red channel to blue channel
    align_vector_r2 = im_align2(red_channel, blue_channel, i);
    r_b2 = circshift(red_channel, align_vector_r2);
    % align green channel to blue channel
    align_vector_g2 = im_align2(green_channel, blue_channel, i);
    g_b2 = circshift(green_channel, align_vector_g2);
    fprintf('image%d NCC calculated alignment:\n', i);
    fprintf('r -> b:(%d, %d)\t g -> b:(%d, %d)\n ', align_vector_r2, align_vector_g2);
    % show color image aligning by ssd
    ssd_color_image2 = cat(3, r_b2, g_b2, blue_channel);
    figure, imshow(ssd_color_image2);
    ncc_image = strcat('image', num2str(i), '-ncc.jpg');
    imwrite(ssd_color_image2, ncc_image)    
end

for i =1 : 6
    raw_image = images{1,i};
    double_image = im2double(raw_image);
    height = floor(size(double_image) / 3);
    % split 3 color channels
    blue_channel = double_image(1:height, : );
    green_channel = double_image(height * 1 + 1 : height * 2, :);
    red_channel = double_image(height * 2 + 1 : height * 3, :);
    
    % Harris
    corner_det_blue = harris(blue_channel);
    corner_det_green = harris(green_channel);
    corner_det_red = harris(red_channel);  
    
    pixel_shift1 = im_align3(corner_det_blue, corner_det_red);
    r_b3 = circshift(red_channel, -pixel_shift1);

    pixel_shift2 = im_align3(corner_det_blue, corner_det_green);
    g_b3 = circshift(green_channel, -pixel_shift2);
    fprintf('image%d Harris corner detection and ransac calculated alignment:\n', i);
    fprintf('r -> b:(%d, %d)\t g -> b:(%d, %d)\n ', pixel_shift1, pixel_shift2);
    ransac_image = cat(3, r_b3, g_b3, blue_channel);   
    figure, imshow(ransac_image);
    ransac_color = strcat('image', num2str(i), '-corner.jpg');
    imwrite(ransac_image, ransac_color);
end
