function [out_img] = fixImage(in_img, gamma, swap_wh_bh, rescale)
    out_img = im2double(im2gray(in_img));
    out_img = (double(out_img) - double(min(out_img(:)))) ./ ...
        (double(max(out_img(:))) - double(min(out_img(:))));
    out_img = out_img.^gamma;

    %Itmp = adapthisteq(Itmp);
    
    out_img = imadjust(out_img);
    if swap_wh_bh
        out_img = imcomplement(out_img);
    end
    if rescale ~= 1
        out_img = imresize(out_img, rescale);
    end
end