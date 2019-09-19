function [detected_corner] = harris(channel_image)

k = 0.04;

% transfer image from double to uint8 form
image = uint8(255 * mat2gray(channel_image));
rows_number = size(image, 1);
cols_number = size(image, 2);

% sigma = 1
% halfwindown = sigma
[x1, y1] = meshgrid(-1:1, -1:1);
Gx = x1 .* exp(-(x1 .^ 2 + y1 .^ 2) / (2 * 1 ^ 2));
Gy = y1 .* exp(-(x1 .^ 2 + y1 .^ 2) / (2 * 1 ^ 2));
Gxy = exp(-(x1 .^ 2 + y1 .^ 2) / (2 * 1 ^ 2));

% 1) Compute x and y derivatives of image
Ix = conv2(Gx, image);
Iy = conv2(Gy, image);

% 2) Compute products of derivatives at every pixel
Ix2 = Ix .^ 2;
Iy2 = Iy .^ 2;
Ixy = Ix .* Iy;

% 3)Compute the sums of the products of derivatives at each pixel
sum_x2 = conv2(Gxy, Ix2);
sum_y2 = conv2(Gxy, Iy2);
sum_xy = conv2(Gxy, Ixy);

im = zeros(rows_number, cols_number);
for x=1:rows_number
   for y=1:cols_number
%        x,y
       % 4) Define at each pixel(x, y) the matrix H
       H = [sum_x2(x, y) sum_xy(x, y); sum_xy(x, y) sum_y2(x, y)];
       
       % 5) Compute the response of the detector at each pixel
       R = det(H) - k * (trace(H) ^ 2);
       im(x, y) = R;
     
   end
end

% 6) Threshold on value of R
avg_r = mean(mean(im));
threshold = abs(5 * avg_r);
% find harris corners points
[rows, cols] = find(im > threshold);
% show corner points on image
figure, imshow(image), hold on,
plot(cols, rows, 'c.');
detected_corner = [rows, cols];
end