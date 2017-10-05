function drawMap(map_)
	hold on;
	for row = 1:rows(map_)
		for col = 1:columns(map_)
		
		  #draw a black block if the map element is occupied (e.g. wall)
			if(map_(row, col) == 1)
				rectangle("Position", [col-1 row-1 1 1], "FaceColor", "black");
			else
				rectangle("Position", [col-1 row-1 1 1], "FaceColor", "white");
			endif
		endfor
	endfor
	hold off;
endfunction

