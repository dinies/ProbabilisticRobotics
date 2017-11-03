1,11c1,12
< # perform the data associations
< # 
< # inputs:
< #   mu: current mean
< #   sigma: current covariance
< #   observations: current observation set, from G2oWrapper
< #   state_to_id_map: mapping vector state_position-id
< #   heuristics: may be used to input the desired level of heuristics
< #
< # outputs:
< #   observations: with the respective, aka associated, ids
---
> % perform the data associations
> % 
> % inputs:
> %   mu: current mean
> %   sigma: current covariance
> %   observations: current observation set, from G2oWrapper
> %   state_to_id_map: mapping vector state_position-id
> %   heuristics: may be used to input the desired level of heuristics
> %
> % outputs:
> %   observations: with the respective, aka associated, ids
> 
17c18
< 	# determine how many landmarks we have seen in this step
---
> 	% determine how many landmarks we have seen in this step
20c21
< 	#set all the observations to unknown
---
> 	%set all the observations to unknown
25c26
< 	# dimension of the state (robot pose + landmark positions)
---
> 	% dimension of the state (robot pose + landmark positions)
28c29
< 	# dimension of the current map (how many landmarks we have already seen)
---
> 	% dimension of the current map (how many landmarks we have already seen)
31c32
< 	#if 0 observations are made or current map is empty, return
---
> 	%if 0 observations are made or current map is empty, return
36c37
< 	# mu_landmark part
---
> 	% mu_landmark part
38c39
< 	# mu_robot part
---
> 	% mu_robot part
40c41
< 	# current_land_id in mu
---
> 	% current_land_id in mu
43c44
< 	#build the association matrix [(current observations) x (number of landmarks)]
---
> 	%build the association matrix [(current observations) x (number of landmarks)]
46,47c47,48
< 	#now we have to populate the association matrix
< 	for n=1:num_landmarks
---
> 	%now we have to populate the association matrix scanning n indexes
> 	for i=1:num_landmarks
49c50
< 		#extract landmark from mu
---
> 		%extract landmark from mu
52c53
< 		#compute measure function
---
> 		%compute measure function
54,56d54
< 		sigma_zx = eye(2,2)*0.01;
< 		sigma_nn = C*sigma*C' + sigma_zx;
< 		omega_nn = inv(sigma_nn);
58,59c56,63
< 		for m=1:num_landmarks_measured
< 			measure = measures(m);
---
> 		%compute Sigma_z and its inverse
> 		Sigma_zx = eye(2,2)*0.01;
> 		Sigma_z = C*sigma*C' + Sigma_zx;
> 		Omega_z = inverse(Sigma_z);
> 
>         % scanning m indexes
> 		for j=1:num_landmarks_measured
> 			measure = measures(j);
61c65
< 			#current measurement
---
> 			%current measurement
64,65c68,70
< 			#look at the indices [(current observations) x (number of landmarks)]
< 			A(m,n) = (z - h)' * omega_nn * (z - h);
---
> 			%look at the indices [(current observations) x (number of landmarks)]
> 			%populate the association matrix
> 			A(j,i) = (z - h)' * Omega_z * ( z - h);
69c74
< 	endfor #num_landmarks
---
> 	endfor %num_landmarks
72,83c77,88
< 	#now associate the measurement to the most promising landmark
< 	# proposed associations will be stored in a [Mx3] matrix composed
< 	# in this way
< 	#
< 	#	[ number of measure , proposed landmark id , association matrix value] 	
< 	#
< 	# we will populate such a matrix with the associations surviving
< 	# the gating heuristic.
< 	# 
< 	# In the best friends and lonely best friend heuristics we will keep
< 	# the association as is in case of success, otherwise we will put
< 	# an id=0 to that measurment, meaning that the association is doubtful
---
> 	%now associate the measurement to the most promising landmark
> 	% proposed associations will be stored in a [Mx3] matrix composed
> 	% in this way
> 	%
> 	%	[ number of measure , proposed landmark id , association matrix value] 	
> 	%
> 	% we will populate such a matrix with the associations surviving
> 	% the gating heuristic.
> 	% 
> 	% In the best friends and lonely best friend heuristics we will keep
> 	% the association as is in case of success, otherwise we will put
> 	% an id=0 to that measurment, meaning that the association is doubtful
85c90
< 	## Heuristics
---
> 	%% Heuristics
88,90c93,95
< 	# gating
<   for m=1:num_landmarks_measured
< 		#return the min and index on the 'm-th' row
---
> 	% gating
> 	for m=1:num_landmarks_measured
> 		%return the min and index on the 'm-th' row
93,94c98,100
< 		if(min_arg < tau_accept)			
< 			# add the possible association
---
> 		if(min_arg < tau_accept) %gating condition			
> 			% add the possible association
> 			% as [number of measure, landmark id, association matrix value]
97c103
<   endfor
---
> 	endfor
99c105
< 	#associations survied to gating
---
> 	%associations survied to gating
102c108
< 	#best friends
---
> 	%best friends
106,108c112,114
< 		min_on_column = min(A(:, proposed_landmark));
< 		if(current_associations_value != min_on_column)
< 			associations(i,2) = 0; #discard association, it is doubtful
---
>         min_of_col = min(A(:,proposed_landmark)); 
> 		if(min_of_col != current_associations_value ) % NEGATIVE of the best friends condition
> 			associations(i,2) = 0; %discard association, it is doubtful
109a116
> 	
113c120
< 	#lonely best friend
---
> 	%lonely best friend
122c129
< 				continue; #this association is doubtful, continue
---
> 				continue; %this association is doubtful, continue
125c132
< 			#obtain second best(aka min) value of the row
---
> 			%obtain second best(aka min) value of the row
128c135
< 			#obtain second best(aka min) value of the column
---
> 			%obtain second best(aka min) value of the column
132,135c139,141
< 			#check if the association is ambiguous
< 			if( (second_row_min_value - current_associations_value) < gamma_threshold ||
< 		 	    (second_col_min_value - current_associations_value) < gamma_threshold )
< 				associations(i,2) = 0; #discard association, it is doubtful
---
> 			%check if the association is ambiguous
> 			if( ( current_associations_value - second_row_min_value ) < gamma_threshold ||  ( current_associations_value - second_col_min_value) < gamma_threshold ) %lonely best friend condition
> 				associations(i,2) = 0; %discard association, it is doubtful
141c147
< 	#assign the associations to the observations
---
> 	%assign the associations to the observations
146c152
< 		##assign the proposed association OR a 0 value that means doubt
---
> 		%%assign the proposed association OR a 0 value that means doubt
147a154
> 
148a156,157
> 	
> 
