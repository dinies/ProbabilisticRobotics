function state_ground_truth = getNextState(map_, state_ground_truth_, control_input_)
  state_ground_truth = state_ground_truth_;

  #sampling setup
  minimum_probability    = unifrnd(0, 1);
  cumulative_probability = 0;


  #obtain all transition probabilities covering motions over the complete map
  transition_probability = transitionModel(map_, state_ground_truth_(1), state_ground_truth_(2), state_ground_truth_(3), control_input_);

  #available motion range
  min_row = state_ground_truth_(1)-1;
  max_row = state_ground_truth_(1)+1;
  min_col = state_ground_truth_(2)-1;
  max_col = state_ground_truth_(2)+1;

  switch(state_ground_truth_(3))
            case 1
              possible_ori= [1,2,4];
            case 2
              possible_ori= [1,2,3];
            case 3
              possible_ori= [2,3,4];
            case 4
              possible_ori= [1,3,4];
            otherwise
                return;
          endswitch



  #over for the available motion range check if probability is higher than the extracted sample
  for (row = min_row:max_row)
	  for (col = min_col:max_col)
      for ( ori = possible_ori)
		    cumulative_probability += transition_probability(row, col, ori);
		    if(cumulative_probability > minimum_probability)
		
		    #return with new ground_state
        state_ground_truth = [row, col, ori];

			   return;
		    endif
     endfor
	  endfor
  endfor
endfunction

