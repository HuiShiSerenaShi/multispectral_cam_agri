function [new_relax, skip, points] = buildGrid(Img, UnPerspectI, bbBox, orig_bbBox, rows, columns, H, relax, total_vertices, wholePoints, debug)
    if size(wholePoints, 1) ~= total_vertices
        new_relax = relax + 0.5;
        skip = true;
        points = [];
        
        %disp(['Give a look here, blobs count: ' num2str(length(wholePoints))]);
        return;
    elseif size(wholePoints, 1) > total_vertices
        new_relax = Inf;
        skip = true;
        points = [];
        
        %disp(['We had excedded the maximum number, blobs count: ' num2str(length(wholePoints))]);
        return;
    else
        new_relax = relax;
        skip = false;
        
        refined_corners = vertcat(wholePoints);
        
        tmpImagePoints = double(refined_corners);
        UnPerspectPointsGood = uint16(tmpImagePoints);
        [UnPerspectPointsGood, indices] = sortrows(UnPerspectPointsGood, 1);
        tmpImagePoints = tmpImagePoints(indices,:);

        counter = 1;       
        for k=1:total_vertices
            if mod(k, rows) == 0
                UnPerspectPointsGood(counter:k) = mean(UnPerspectPointsGood(counter:k));
                counter = k + 1;
            end
        end

        [UnPerspectPointsGood, indices] = sortrows(UnPerspectPointsGood, [1 2]);
        tmpImagePoints = tmpImagePoints(indices,:);
               
        % Check if there's an error on the columns subdivision.
        % If on the same column exists a point far from the x_mean more
        % than a threshold, skip.
        counter = 1;
        x_mean = [];
        for k=1:total_vertices
            if mod(k, rows) == 0
                sub_arr = tmpImagePoints(counter:k,1);
                x_mean = [x_mean mean(sub_arr)];
                counter = k + 1;
            end
        end
        
        % Check if there's an error on the rows subdivision.
        % If on the same column exists a point far from the y_mean more
        % than a threshold, skip.
        y_mean = [];
        for k=1:rows
            sub_arr = tmpImagePoints(mod(0:total_vertices-1, rows) == k-1, 2);
            y_mean = [y_mean mean(sub_arr)];
        end
        
        vfun =@(x)findLines(x,tmpImagePoints(:,:), columns, rows);

        % Initial guess x0: [a b c]        
        m = [zeros(1,rows) ones(1,columns)];
        q = [y_mean x_mean];
        x0 = [m ones(1,rows) zeros(1,columns) -q];

        options = optimoptions('lsqnonlin','Display','off','MaxIterations',1000);       
        [x,~,~,~,~] = lsqnonlin(vfun,x0,[],[],options);
        
        a = x(1:columns+rows);
        b = x(columns+rows+1:2*(columns+rows));
        c = x(2*(columns+rows)+1:3*(columns+rows));
        
        IntersectedPoints = getIntersections(a,b,c,columns,rows);       
        tmpImagePoints = IntersectedPoints;
        
        % Apply inverse homography to the computed centroids    
        tmp = [tmpImagePoints ones(total_vertices,1)] * pinv(H.T);
        tmp = tmp ./ tmp(:,3);
        points = tmp(:,1:2);

        if debug
            figure(24);
            imshow(UnPerspectI);
            hold on;
            for f=1:rows+columns
                fimplicit(@(x,y) a(f)*x + b(f)*y + c(f),[-640 640 -640 640])
                hold on;
            end
            plot(tmpImagePoints(:,1),tmpImagePoints(:,2),'go');
            hold on
            plot(IntersectedPoints(:,1),IntersectedPoints(:,2),'ro');
            hold off;            
            
            figure(42)
            clf;            
            subplot(2,2,1);
            imshow(Img(:,:));
            title('Working image');    

            subplot(2,2,2);
            imshow(UnPerspectI);
            hold on;
            plot(refined_corners(:,1),refined_corners(:,2), 'r*');
            title('Rectified image');
            hold off;

            subplot(2,2,3);
            imshow(insertText(UnPerspectI, UnPerspectPointsGood, 1:length(UnPerspectPointsGood(:,:)), 'FontSize', 8));    
            title('Unperspected points')
            hold on;
            rectangle('Position', orig_bbBox, 'EdgeColor','g', 'LineWidth', 1)
            rectangle('Position', bbBox, 'EdgeColor','r', 'LineWidth', 1)

            subplot(2,2,4);
            imshow(insertText(Img(:,:), points, 1:length(points), 'FontSize', 8));
            title('Image points')
            hold on
            plot(points(:,1),points(:,2),'go');
            hold off
            
            pause;
        end
    end
end