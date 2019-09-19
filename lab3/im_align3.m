function [pixel_shift]= im_align3(corner_det_green, corner_det_blue)
 
inlier = 0;
shift_vector = [0, 0];
xgreen_size = size(corner_det_green(:, 1));
xblue_size = size(corner_det_blue(:, 1));
x_size = min(xgreen_size, xblue_size);

for m = 1:180
    index1 = randi(x_size(:, 1));
    index2 = randi(x_size(:, 1));
    x1_feature = corner_det_green(index1, 1);
    y1_feature = corner_det_green(index1, 2);

    x2_feature = corner_det_blue(index2, 1);
    y2_feature = corner_det_blue(index2, 2);

    x_shift = x2_feature - x1_feature;
    y_shift = y2_feature - y1_feature;

    x_green_shift = corner_det_green(:, 1) + x_shift;
    y_green_shift = corner_det_green(:, 2) + y_shift;
    green_shift = [x_green_shift, y_green_shift];

    for i = -1 : 1
        for j = -1 : 1
            window_x = corner_det_blue(:, 1) + i;
            window_y = corner_det_blue(:, 2) + j;
            shift_image2 = [window_x, window_y];
            [com, index_green, index_blue] = intersect(green_shift, shift_image2);
            if inlier < size(com)
                inlier = size(com);
                shift_vector = [x_shift, y_shift];
            end
        end
    end
end
pixel_shift = shift_vector;
end






