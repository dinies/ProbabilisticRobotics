function pooledMatrix = poolingMatrix(matrix_)
	size= size(matrix_);
	pooledMatrix = zeros(size(1),size(2));

	for(row =1:size(1))
		for(col = 1:size(2))
			count= 0;
			for (ori = 1:size(3))
				count += matrix_(row,col,ori);
			endfor
			pooledMatrix(row,col)= count;
		endfor
	endfor


endfunction


%!assert (poolingMatrix(ones(2,2,2)), ones(2,2)+1);