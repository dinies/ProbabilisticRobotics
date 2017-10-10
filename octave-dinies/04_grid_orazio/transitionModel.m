function transition_probability_matrix = transitionModel(map_, row_from_, col_from_, orientation_, control_input_)
  map_rows = rows(map_);
  map_cols = columns(map_);
  transition_probability_matrix = zeros(map_rows, map_cols, 4);

  #against each other cell and itself
	for row_to = 1:map_rows
		for col_to = 1:map_cols
      for orientation = 1:4
	
      #available robot controls (corresponding to keyboard key values)
      global MOVE_FORWARD
      global MOVE_BACKWARD
      global ROTATE_LEFT
      global ROTATE_RIGHT

      #compute resulting position difference
      translation_rows = row_to - row_from_;
      translation_cols = col_to - col_from_;      
      % # UP
      % # 1 - 1 -> yes = 0
      % # 1 - 2 -> yes = -1
      % # 1 - 3 -> no  = -2
      % # 1 - 4 -> yes = -3

      % # RIGHT
      % # 2 - 1 -> yes = 1
      % # 2 - 2 -> yes = 0
      % # 2 - 3 -> yes = -1
      % # 2 - 4 -> no  = -2

      % # DOWN
      % # 3 - 1 -> no  = 2
      % # 3 - 2 -> yes = 1
      % # 3 - 3 -> yes = 0
      % # 3 - 4 -> yes = -1

      % # LEFT
      % # 4 - 1 -> yes = 3
      % # 4 - 2 -> no  = 2
      % # 4 - 3 -> yes = 1
      % # 4 - 4 -> yes = 0
      translation_orientation= orientation - orientation_;


      #allow only unit motions (1 cell): check if we have a bigger motion
      if(abs(translation_rows) > 1 || abs(translation_cols) > 1 || abs(translation_orientation) == 2)
	      continue;
      endif

      #compute target robot position according to input
      target_row = row_from_;
      target_col = col_from_;
      target_ori = orientation_;

      switch (control_input_)
        case MOVE_FORWARD
          switch(orientation_)
            case 1
              target_row--;
            case 3
              target_row++;
            case 2
              target_col++;
            case 4
              target_col--;
            otherwise
                return;
          endswitch

        case MOVE_BACKWARD
          switch(orientation_)
            case 1
              target_row++;
            case 3
              target_row--;
            case 2
              target_col--;
            case 4
              target_col++;
            otherwise
                return;
          endswitch

        case ROTATE_RIGHT
          switch(orientation_)
            case 1
              target_ori= 2;
            case 3
              target_ori= 4;
            case 2
              target_ori= 3;
            case 4
              target_ori= 1;
            otherwise
                return;
          endswitch

          
        case ROTATE_LEFT
          switch(orientation_)
            case 1
              target_ori= 4;
            case 3
              target_ori= 2;
            case 2
              target_ori= 1;
            case 4
              target_ori= 3;
            otherwise
                return;
          endswitch
        
        otherwise
          return;
      endswitch

	    #check if the desired motion is infeasible
	    invalid_motion = false;
	    if (target_row < 1 || target_row > map_rows || target_col < 1 || target_col > map_cols) #if we're going over the border
		    invalid_motion = true;
	    elseif (map_(target_row, target_col) == 1 || map_(row_to, col_to) == 1) #obstacle in the goal cell
		    invalid_motion = true;
	    endif
	    if (invalid_motion)
	
	      #if the desired translation is zero
	      if (translation_rows == 0 && translation_cols == 0 && translation_orientation == 0)
          transition_probability_matrix(row_to, col_to, orientation) = 1; #we stay with certain probability (no motion has full confidence)
		      continue;
	      else
	        continue; #we cannot move
	      endif
	    endif

      #our motion is feasible - compute resulting transition
      switch (control_input_)
        case MOVE_FORWARD
          switch(orientation_)
            case 1
              if (translation_rows     == -1 && translation_cols ==  0 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.8;
               elseif (translation_rows == -1 && translation_cols ==  1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
               elseif (translation_rows == -1 && translation_cols == -1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
              endif;
            case 3
              if (translation_rows     == 1 && translation_cols ==  0 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.8;
               elseif (translation_rows == 1 && translation_cols ==  1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
               elseif (translation_rows == 1 && translation_cols == -1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
              endif;
            case 2
              if (translation_rows     == 0 && translation_cols ==  1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to,orientation) = 0.8;
               elseif (translation_rows == 1 && translation_cols == 1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to,orientation) = 0.1;
               elseif (translation_rows == -1 && translation_cols == 1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to,orientation) = 0.1;
              endif;
            case 4
              if (translation_rows     == 0 && translation_cols ==  -1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.8;
               elseif (translation_rows == 1 && translation_cols == -1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
               elseif (translation_rows == -1 && translation_cols == -1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
              endif;
            otherwise
              return;
          endswitch
        case MOVE_BACKWARD
          switch(orientation_)
            case 1
              if (translation_rows     == 1 && translation_cols ==  0 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.8;
               elseif (translation_rows == 1 && translation_cols ==  1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
               elseif (translation_rows == 1 && translation_cols == -1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
              endif;
            case 3
              if (translation_rows     == -1 && translation_cols ==  0 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.8;
               elseif (translation_rows == -1 && translation_cols ==  1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
               elseif (translation_rows == -1 && translation_cols == -1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
              endif;
            case 2
              if (translation_rows     == 0 && translation_cols ==  -1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.8;
               elseif (translation_rows == 1 && translation_cols == -1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
               elseif (translation_rows == -1 && translation_cols == -1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
              endif;
            case 4
              if (translation_rows     == 0 && translation_cols ==  1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.8;
               elseif (translation_rows == 1 && translation_cols == 1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
               elseif (translation_rows == -1 && translation_cols == 1 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
              endif;
            otherwise
              return;
          endswitch	
        case ROTATE_RIGHT
         
          switch(orientation_)
            case 1
              if (translation_rows == 0 && translation_cols == 0 && translation_orientation == -1) transition_probability_matrix(row_to, col_to, orientation) = 0.8;
               elseif (translation_rows == 0 && translation_cols == 0 && translation_orientation == -3) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
               elseif (translation_rows == 0 && translation_cols == 0 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
              endif;                     
            case 3
              if (translation_rows == 0 && translation_cols == 0 && translation_orientation == -1) transition_probability_matrix(row_to, col_to, orientation) = 0.8;
               elseif (translation_rows == 0 && translation_cols == 0 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
               elseif (translation_rows == 0 && translation_cols == 0 && translation_orientation == 1) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
              endif;
            case 2
              if (translation_rows == 0 && translation_cols == 0 && translation_orientation == -1) transition_probability_matrix(row_to, col_to, orientation) = 0.8;
               elseif (translation_rows == 0 && translation_cols == 0 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
               elseif (translation_rows == 0 && translation_cols == 0 && translation_orientation == 1) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
              endif;
            case 4
              if (translation_rows == 0 && translation_cols == 0 && translation_orientation == 3) transition_probability_matrix(row_to, col_to, orientation) = 0.8;
               elseif (translation_rows == 0 && translation_cols == 0 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
               elseif (translation_rows == 0 && translation_cols == 0 && translation_orientation == 1) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
              endif;            otherwise
                return;
          endswitch
        case ROTATE_LEFT  
          switch(orientation_)
            case 1
              if (translation_rows == 0 && translation_cols == 0 && translation_orientation == -3) transition_probability_matrix(row_to, col_to, orientation) = 0.8;
               elseif (translation_rows == 0 && translation_cols == 0 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
               elseif (translation_rows == 0 && translation_cols == 0 && translation_orientation == -1) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
              endif;                     
            case 3
              if (translation_rows == 0 && translation_cols == 0 && translation_orientation == 1) transition_probability_matrix(row_to, col_to, orientation) = 0.8;
               elseif (translation_rows == 0 && translation_cols == 0 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
               elseif (translation_rows == 0 && translation_cols == 0 && translation_orientation == -1) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
              endif;
            case 2
              if (translation_rows == 0 && translation_cols == 0 && translation_orientation == 1) transition_probability_matrix(row_to, col_to, orientation) = 0.8;
               elseif (translation_rows == 0 && translation_cols == 0 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
               elseif (translation_rows == 0 && translation_cols == 0 && translation_orientation == -1) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
              endif;
            case 4
              if (translation_rows == 0 && translation_cols == 0 && translation_orientation == 1) transition_probability_matrix(row_to, col_to, orientation) = 0.8;
               elseif (translation_rows == 0 && translation_cols == 0 && translation_orientation == 0) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
               elseif (translation_rows == 0 && translation_cols == 0 && translation_orientation == 3) transition_probability_matrix(row_to, col_to, orientation) = 0.1;
              endif;            otherwise
                return;
          endswitch
        endswitch
      endfor
    endfor
  endfor
endfunction

