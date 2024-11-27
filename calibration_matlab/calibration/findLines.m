function out = findLines(x, p_opt, columns, rows)

    %  p_opt is orderd by column
    %  f1, ... ,f_i row lines
    %  f_i+1, ... ,f_j column lines
    %
    %  p_opt sorting example:
    %
    %  1   5   9   13  17
    %  2   6   10  14  18
    %  3   7   11  15  19
    %  4   8   12  16  20
    %
    %  ax + by + c = 0

    a = x(1:columns+rows);
    b = x(columns+rows+1:2*(columns+rows));
    c = x(2*(columns+rows)+1:3*(columns+rows));
    
    out = [];

    % Lines intersections to points
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
            
            % Computing error
            out = [out, p(1) - p_opt(count,1), p(2) - p_opt(count,2)];
            count = count + 1;
        end
    end
    
    % Lines parallellism over rows
    for i=1:rows-1
        m1 = a(i)/b(i);
        m2 = a(i+1)/b(i+1);
        out = [out, abs(m1-m2)];
    end
    
    % Lines parallellism over columns
    for i=rows+1:rows+columns-1
        m1 = b(i)/a(i);
        m2 = b(i+1)/a(i+1);        
        out = [out, abs(m1-m2)];
    end
end