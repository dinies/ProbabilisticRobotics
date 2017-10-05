function new_state_ground_truth = getNextState(map_, state_ground_truth_, control_input_)

  #obtain transition probabilities (covering motions over the complete map)
  transition_probability_matrix = transitionModel(map_, state_ground_truth_(1), state_ground_truth_(2), control_input_);

	#extract a uniform sample between 0 and 100% as minimum probability to move
	minimum_probability = unifrnd(0, 1);

	#available motion range
	min_row = state_ground_truth_(1)-1; #MOVE_DOWN
  	max_row = state_ground_truth_(1)+1; #MOVE_UP
	min_col = state_ground_truth_(2)-1; #MOVE_LEFT
	max_col = state_ground_truth_(2)+1; #MOVE_RIGHT

	#over for the available motion range check if probability is higher than the extracted sample
	cumulative_probability = 0;
	for (row = min_row:max_row)
		for (col = min_col:max_col)
			cumulative_probability += transition_probability_matrix(row, col);
			if(cumulative_probability > minimum_probability)
			
			  #return with new position
				new_state_ground_truth = [row, col];
				return;
			endif
		endfor
	endfor
	
	#stick to initial solution if minimum probability could not be achieved
	new_state_ground_truth = state_ground_truth_;
endfunction

