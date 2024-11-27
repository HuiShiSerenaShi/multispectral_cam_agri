function points = getIntersections(a,b,c,columns,rows)
    
    points = zeros(columns*rows,2);
    
    count = 1;
    for j=1:columns
        for i=1:rows
            
            % First line is always the row line          
            a1 = a(i);
            b1 = b(i);
            c1 = c(i);

            % Second line is always the column line             
            a2 = a(j + rows);
            b2 = b(j + rows);
            c2 = c(j + rows);
            
            % Computing intersection point
            p = [(b1*c2) - (b2*c1), (a2*c1) - (a1*c2), (a1*b2) - (a2*b1)];
            p(1) = p(1)/p(3);
            p(2) = p(2)/p(3);
            
            points(count,:) = [p(1) p(2)];
            count = count + 1;
        end
    end
end