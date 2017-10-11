function drawOrientation(map_, row_, col_, ori_,color_code_)

    hold on;

    fixed_x= col_-1;
    fixed_y= rows(map_)-row_;


    switch(ori_)
        case 1
            rectangle("Position", [fixed_x+0.4 fixed_y+0.8  0.2 0.2], "FaceColor", color_code_, "EdgeColor", "white");
        case 2
            rectangle("Position", [fixed_x+0.8 fixed_y+0.4  0.2 0.2], "FaceColor", color_code_, "EdgeColor", "white");
        case 3
            rectangle("Position", [fixed_x+0.4 fixed_y  0.2 0.2], "FaceColor", color_code_, "EdgeColor", "white");
        case 4
            rectangle("Position", [fixed_x fixed_y+0.4  0.2 0.2], "FaceColor", color_code_, "EdgeColor", "white");
        otherwise
        return;
    endswitch

    hold off;


endfunction