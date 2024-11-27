function showResults(img_Set, data_A, num_cols, num_rows)
    num_images = data_A.numImages;

    if nargin == 2
        num_cols = 6;
        num_rows = 5;
    elseif nargin == 3
        num_rows = 5;
    end

    img_per_fig = num_rows * num_cols;
    num_figures = ceil((num_images/img_per_fig));

    hFig_A = figure;
    ff_A = 0;
    set(hFig_A, 'Name', [data_A.name]);

    function slideshowA(~, event)
        if strcmp(event.Key, 'leftarrow') == 1
            if ff_A > 1
                ff_A = ff_A - 1;
                set(hFig_A, 'Name', [data_A.name, ' ', num2str(ff_A),'/',num2str(num_figures)]);
                clf(hFig_A);
            else
                ff_A = 1;
                return
            end
        elseif strcmp(event.Key, 'rightarrow') == 1
            if ff_A < num_figures
                ff_A = ff_A + 1;
                set(hFig_A, 'Name', [data_A.name, ' ', num2str(ff_A),'/',num2str(num_figures)]);
                clf(hFig_A);
            else
                ff_A = num_figures;
                return
            end
        else
            return
        end
        img_count_A = ff_A * img_per_fig - img_per_fig + 1;
        for i_A=1:img_per_fig
            if img_count_A > num_images
                break
            end
            if ~data_A.excluded(img_count_A)
                img = readimage(img_Set,img_count_A);
                img = fixImage(img, data_A.gamma, data_A.swapWhBh, data_A.rescale);
                subp_A = subplot(num_rows,num_cols,i_A,'Parent', hFig_A);
                imshow(img, 'Parent', subp_A);
                title(['Image: ', num2str(img_count_A)]);
                hold on;
                plot(data_A.points(:, 1, img_count_A), data_A.points(:, 2, img_count_A), 'r*', 'Parent', subp_A);
                hold off;
            end
            img_count_A = img_count_A + 1;
        end
        drawnow();
    end
    set(hFig_A,'KeyPressFcn',@slideshowA);
    
end