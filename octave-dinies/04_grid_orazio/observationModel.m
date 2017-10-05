function observation_probability = observationModel(map_, row_, col_, observations_)

  #evaluate cell occupancy
	up_occupied    = 1;
	down_occupied  = 1;
	left_occupied  = 1;
	right_occupied = 1;
	if (row_-1 > 0)
		up_occupied    = map_(row_-1, col_);
	endif
	if (row_+1 <= rows(map_))
		down_occupied  = map_(row_+1, col_);
	endif
	if (col_-1 > 0)
		left_occupied  = map_(row_, col_-1);
	endif
	if (col_+1 <= columns(map_))
		right_occupied = map_(row_, col_+1);
	endif

	#update probability depending on observations: [up_occupied, down_occupied, left_occupied, right_occupied];
	observation_probability = 1;
	if (up_occupied == observations_(1))
		observation_probability *= .8;
	else
		observation_probability *= .2;
	endif	    
	if (down_occupied == observations_(2))
		observation_probability *= .8;
	else
		observation_probability *= .2;
	endif
	if (left_occupied == observations_(3))
		observation_probability *= .8;
	else
		observation_probability *= .2;
	endif	
	if (right_occupied == observations_(4))
		observation_probability *= .8;
	else
		observation_probability *= .2;
	endif	
endfunction

