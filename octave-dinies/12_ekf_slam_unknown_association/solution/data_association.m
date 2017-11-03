# perform the data associations
# 
# inputs:
#   mu: current mean
#   sigma: current covariance
#   observations: current observation set, from G2oWrapper
#   state_to_id_map: mapping vector state_position-id
#   heuristics: may be used to input the desired level of heuristics
#
# outputs:
#   observations: with the respective, aka associated, ids

function observations = data_association(mu, sigma, observations, state_to_id_map)

	measures = observations.observation;

	# determine how many landmarks we have seen in this step
	num_landmarks_measured = length(measures);

	#set all the observations to unknown
	for i=1:num_landmarks_measured
		observations.observation(i).id = -1;
	endfor

	# dimension of the state (robot pose + landmark positions)
	state_dim = size(mu,1);

	# dimension of the current map (how many landmarks we have already seen)
	num_landmarks = (state_dim-3)/2;

	#if 0 observations are made or current map is empty, return
	if(num_landmarks == 0 || num_landmarks_measured == 0)
		return;
	endif

	# mu_landmark part
	mu_landmarks = mu(4:end);
	# mu_robot part
	mu_robot = mu(1:3);
	# current_land_id in mu
	curr_land_id = 1;

	#build the association matrix [(current observations) x (number of landmarks)]
	A = ones(num_landmarks_measured, num_landmarks)*1e3;
	
	#now we have to populate the association matrix
	for n=1:num_landmarks

		#extract landmark from mu
		mu_curr_landmark = mu_landmarks(curr_land_id:curr_land_id+1);
	
		#compute measure function
		[h, C] = measurement_function(state_dim,mu_robot, mu_curr_landmark, curr_land_id);
		sigma_zx = eye(2,2)*0.01;
		sigma_nn = C*sigma*C' + sigma_zx;
		omega_nn = inv(sigma_nn);

		for m=1:num_landmarks_measured
			measure = measures(m);

			#current measurement
			z = [measure.x_pose; measure.y_pose];

			#look at the indices [(current observations) x (number of landmarks)]
			A(m,n) = (z - h)' * omega_nn * (z - h);
		endfor

		curr_land_id += 2;
	endfor #num_landmarks

	
	#now associate the measurement to the most promising landmark
	# proposed associations will be stored in a [Mx3] matrix composed
	# in this way
	#
	#	[ number of measure , proposed landmark id , association matrix value] 	
	#
	# we will populate such a matrix with the associations surviving
	# the gating heuristic.
	# 
	# In the best friends and lonely best friend heuristics we will keep
	# the association as is in case of success, otherwise we will put
	# an id=0 to that measurment, meaning that the association is doubtful

	## Heuristics
	tau_accept = 1;

	# gating
  for m=1:num_landmarks_measured
		#return the min and index on the 'm-th' row
		[min_arg, min_index] = min(A(m,:));

		if(min_arg < tau_accept)			
			# add the possible association
			associations(end+1,:) = [m, min_index, min_arg];
		endif
  endfor

	#associations survied to gating
	dim_gated_associations = size(associations,1);

	#best friends
	for i=1:dim_gated_associations
		current_associations_value = associations(i,3);
		proposed_landmark = associations(i,2);
		min_on_column = min(A(:, proposed_landmark));
		if(current_associations_value != min_on_column)
			associations(i,2) = 0; #discard association, it is doubtful
		endif
	endfor

	
	#lonely best friend
	if(num_landmarks_measured > 1)
		gamma_threshold = 1e-3;

		for i=1:dim_gated_associations
			current_associations_value = associations(i,3);
			measure_number = associations(i,1);
			proposed_landmark = associations(i,2);
			if(proposed_landmark == 0)
				continue; #this association is doubtful, continue
			endif

			#obtain second best(aka min) value of the row
			ordered_row = unique(A(measure_number,:));
			second_row_min_value = ordered_row(2);
			#obtain second best(aka min) value of the column
			ordered_col = unique(A(:,proposed_landmark));
			second_col_min_value = ordered_col(2);

			#check if the association is ambiguous
			if( (second_row_min_value - current_associations_value) < gamma_threshold ||
		 	    (second_col_min_value - current_associations_value) < gamma_threshold )
				associations(i,2) = 0; #discard association, it is doubtful
			endif
	
		endfor
	endif

	#assign the associations to the observations
	for i=1:dim_gated_associations
		measure_number = associations(i,1);
		proposed_landmark = associations(i,2);		

		##assign the proposed association OR a 0 value that means doubt
		observations.observation(measure_number).id = proposed_landmark;
	endfor
end
