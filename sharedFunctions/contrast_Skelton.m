% Converting the visual input into a contrast representation. 

function contrast = contrast_Skelton(input_frame)
 
 row = size(input_frame, 1);
 col = size(input_frame, 2);
 I_mean = 1/mean(input_frame(:));
 N_nn = [2, 3, 4];               
 N_nnn = [1, 2, 4];
 denom = zeros(1, 3);
 for i=1:3
     denom(1, i) = (N_nn(1, i)+1/sqrt(2)*N_nnn(1, i));
 end
  contrast = zeros(row,col);
 
 % for 'corner' pixels  
 C_nn_11 = abs(input_frame(1,1)-input_frame(1,2))+abs(input_frame(1,1)-input_frame(2,1));
 C_nn_1col = abs(input_frame(1,col)-input_frame(1,col-1))+abs(input_frame(1,col)-input_frame(2,col));
 C_nn_row1 = abs(input_frame(row,1)-input_frame(row,2))+abs(input_frame(row,1)-input_frame(row-1,1));
 C_nn_rowcol = abs(input_frame(row,col)-input_frame(row,col-1))+abs(input_frame(row,col)-input_frame(row-1,col));
 contrast(1, 1) = (C_nn_11+ 1/sqrt(2)*(abs(input_frame(1,1)-input_frame(2,2))))/denom(1, 1);
 contrast(1, col) = (C_nn_1col+ 1/sqrt(2)*(abs(input_frame(1,col)-input_frame(2,col-1))))/denom(1, 1);
 contrast(row, 1) = (C_nn_row1+ 1/sqrt(2)*(abs(input_frame(row,1)-input_frame(row-1,2))))/denom(1, 1);
 contrast(row,col) = (C_nn_rowcol+ 1/sqrt(2)*(abs(input_frame(row,col)-input_frame(row-1,col-1))))/denom(1, 1);
 
% for 'vertical edge' pixels
 for i = 2:(row-1)
        % cardinal directions 
         up_1 = abs(input_frame(i,1)-input_frame(i-1,1));
         down_1 = abs(input_frame(i,1)-input_frame(i+1,1));
         right_1 = abs(input_frame(i,1)-input_frame(i,2));
         up_col = abs(input_frame(i,col)-input_frame(i-1,col));
         down_col = abs(input_frame(i,col)-input_frame(i+1,col));
         left_col = abs(input_frame(i,col)-input_frame(i,col-1));         
        % ordinal directions 
         up_right_1 = abs(input_frame(i,1)-input_frame(i-1,2));
         down_right_1 = abs(input_frame(i,1)-input_frame(i+1,2));
         up_left_col = abs(input_frame(i,col)-input_frame(i-1,col-1));
         down_left_col = abs(input_frame(i,col)-input_frame(i+1,col-1));

         c_nn_1 = up_1+right_1+down_1;
         c_nnn_1 = up_right_1+down_right_1;
         contrast(i,1) = (1*c_nn_1 + 1/sqrt(2)*c_nnn_1)/denom(1, 2);
         
         c_nn_col = up_col+left_col+down_col;
         c_nnn_col = up_left_col+down_left_col;
         contrast(i,col) = (1*c_nn_col + 1/sqrt(2)*c_nnn_col)/denom(1, 2);
 end
 
 % for 'horizontal edge' pixels
 for j = 2:(col-1)
        % cardinal directions 
         Right_1 = abs(input_frame(1,j)-input_frame(1,j-1));
         Down_1 = abs(input_frame(1,j)-input_frame(2,j));
         Left_1 = abs(input_frame(1,j)-input_frame(1,j+1));
         Right_row = abs(input_frame(row,j)-input_frame(row,j-1));
         Up_row = abs(input_frame(row,j)-input_frame(row-1,j));
         Left_row = abs(input_frame(row,j)-input_frame(row,j+1));         
        % ordinal directions 
         Down_left_1 = abs(input_frame(1,j)-input_frame(2,j-1));
         Down_right_1 = abs(input_frame(1,j)-input_frame(2,j+1));
         Up_left_row = abs(input_frame(row,j)-input_frame(row-1,j-1));
         Up_right_row = abs(input_frame(row,j)-input_frame(row-1,j+1));

         C_nn_1 = Left_1+Right_1+Down_1;
         C_nnn_1 = Down_left_1+Down_right_1;
         contrast(1, j) =  (1*C_nn_1 + 1/sqrt(2)*C_nnn_1)/denom(1, 2);
         
         C_nn_col = Right_row+Up_row+Left_row;
         C_nnn_col = Up_left_row+Up_right_row;
         contrast(row, j) = (1*C_nn_col + 1/sqrt(2)*C_nnn_col)/denom(1, 2);
 end 
 
% for 'fill' pixels
 for i = 2:(row-1)
    for j = 2:(col-1)
        % cardinal directions 
         up = abs(input_frame(i,j)-input_frame(i-1,j));
         down = abs(input_frame(i,j)-input_frame(i+1,j));
         right = abs(input_frame(i,j)-input_frame(i,j+1));
         left = abs(input_frame(i,j)-input_frame(i,j-1));
        % ordinal directions 
         up_right = abs(input_frame(i,j)-input_frame(i-1,j+1));
         up_left = abs(input_frame(i,j)-input_frame(i-1,j-1));
         down_right = abs(input_frame(i,j)-input_frame(i+1,j+1));
         down_left = abs(input_frame(i,j)-input_frame(i+1,j-1));

         C_nn = up+down+right+left;
         C_nnn = up_right+up_left + down_right+down_left;
         contrast(i,j) = (1*C_nn + 1/sqrt(2)*C_nnn)/denom(1, 3);
    end
 end 
 contrast = I_mean*contrast;    
end
