function drawBelief(state_belief_, map_)

  	pooled_belief= poolingMatrix( state_belief_);


  	flipped_map= flipud(map_);
  	 
	
	#invert belief value for plotting (0:black: 100% confidence, 1:white: 0% confidence)
	plotted_state_belief_ = flipud(ones(size(pooled_belief)) - pooled_belief);

  #plot a colormap with the respective belief values
	colormap(gray(64));
	hold on;
	image([0.5, columns(map_)-0.5], [0.5, rows(map_)-0.5], plotted_state_belief_*64);

	for row = 1:rows(map_)
		for col = 1:columns(map_)
			if ( map_(row,col) ==0 )
				orientation_prob= state_belief_(row, col, :);
				[x, ix] = max(orientation_prob);
				drawOrientation(flipped_map, row, col, ix, "white","black");
			endif
		endfor
	endfor

	hold off;


endfunction

