function clearObservations(observations_, robot_position_, map_)
  row = robot_position_(1)-1;
  col = robot_position_(2)-1;
	hold on;

	#check the observation array
	if (observations_(1)) #UP
	  if (map_(row+2, col+1)) 
	    rectangle("Position", [col row+1 1 1], "FaceColor", "black");
	  else
      rectangle("Position", [col row+1 1 1], "FaceColor", "white");
    endif
	endif
	if (observations_(2)) #DOWN
	  if (map_(row, col+1))
	    rectangle("Position", [col row-1 1 1], "FaceColor", "black");
	  else
      rectangle("Position", [col row-1 1 1], "FaceColor", "white");
    endif
	endif
	if (observations_(3)) #LEFT
	  if (map_(row+1, col))
	    rectangle("Position", [col-1 row 1 1], "FaceColor", "black");
	  else
      rectangle("Position", [col-1 row 1 1], "FaceColor", "white");
    endif
	endif
	if (observations_(4)) #RIGHT
	  if (map_(row+1, col+2))
	    rectangle("Position", [col+1 row 1 1], "FaceColor", "black");
	  else
      rectangle("Position", [col+1 row 1 1], "FaceColor", "white");
    endif
	endif
	
	hold off;
endfunction

