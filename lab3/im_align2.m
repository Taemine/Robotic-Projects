function [align_vector] = im_align2(plate, base_plate, image_number)
% template of image3 is distinct from the other 5 pictures
if image_number == 3
    plate = imcrop(plate, [10, 10, 300, 300]);
    base_plate = imcrop(base_plate, [10, 10, 300, 300]);
else
    % templates of the other 5 images
    plate = imcrop(plate, [50, 50, 150, 150]);
    base_plate = imcrop(base_plate, [50, 50, 150, 150]);
end

ncc = 0;
for i = -15 : 15
    for j = -15 : 15
        %window moving 
        template = circshift(plate, [i j]);
        shift_baseplate = circshift(base_plate, [i j]);
        % NCC calculation
        % mean of all pixels in template
        mean_template = mean(template(:));
        % mean of all pixels in image to match
        mean_baseplate = mean(shift_baseplate(:));
        sum1 = sum((base_plate - mean_baseplate).*(template - mean_template));
        sum2 = (sum((base_plate - mean_baseplate).^2).*sum((template - mean_template).^2)).^0.5;
        temp_ncc = sum1 / sum2;
        % select pixel that NCC value is maximum
        if ncc < temp_ncc
            ncc = temp_ncc;
            max_ncc = [i j];
        end      
     end
end
align_vector = max_ncc;
end