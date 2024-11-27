function [J] = mapThermalData(I, T, H, i_new_orig, t_new_orig)
    J = zeros([size(I,1),size(I,2)]);
    for u = 1:size(I, 2)
        for v = 1:size(I, 1)
            uv_l = [u, v, 1] - [1,1,0] + [i_new_orig, 0];
            uv_t = H * uv_l';% - lt;
            uv_t = [1, 1, 0] - [t_new_orig, 0] + uv_t / uv_t(3);
            if floor(uv_t(1)) > 0 && ...
               floor(uv_t(2)) > 0 && ...
               ceil(uv_t(1)) <= size(T,2) && ...
               ceil(uv_t(2)) <= size(T,1)
                J(v,u) = T(round(uv_t(2)), round(uv_t(1)));
            end
        end
    end
end