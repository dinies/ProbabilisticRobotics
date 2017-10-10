function drawOrientation(map_, row_, col_, ori_,color_code_)

    hold on;

    switch(ori_)
        case 1
            rectangle("Position", [col_-1 rows(map_)-row_  0.2 0.2], "FaceColor", color_code_);
        case 3
            rectangle("Position", [col_-1 rows(map_)-row_-0.8  0.2 0.2], "FaceColor", color_code_);
        case 2
            rectangle("Position", [col_-1+0.8 rows(map_)-row_  0.2 0.2], "FaceColor", color_code_);
        case 4
            rectangle("Position", [col_-1-0.8 rows(map_)-row_  0.2 0.2], "FaceColor", color_code_);
        otherwise
        return;
    endswitch

    hold off;


endfunction