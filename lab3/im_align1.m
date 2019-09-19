function [align_vector] = im_align1(plate, base_plate, image_number)
% template of image3 is distinct from the other 5 pictures
if image_number == 3
    plate = imcrop(plate, [10, 10, 300, 300]);
    base_plate = imcrop(base_plate, [10, 10, 300, 300]);
else
    % templates of the other 5 images
    plate = imcrop(plate, [50, 50, 150, 150]);
    base_plate = imcrop(base_plate, [50, 50, 150, 150]);
end

sum_ssd = inf;
for i = -15 : 15
    for j = -15 : 15
        %window moving 
        shift_image = circshift(plate, [i j]);
        % SSD calculation
        ssd = sum(sum((base_plate - shift_image).^2));
        % select pixel where the SSD value is minimum
        if ssd < sum_ssd
            sum_ssd = ssd;
            min_ssd = [i j];
        end
     end
end
align_vector = min_ssd;
end