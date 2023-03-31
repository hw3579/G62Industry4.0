function result_matrix = combine_matrices(mat1, mat2, mat3, mat4)

for i=10:12
 % Duplicate the last row of each input matrix and add it as the 10.11.12th row
 mat1(i,:) = mat1(9,:);
 mat2(i,:) = mat2(9,:);
 mat3(i,:) = mat3(9,:);
 mat4(i,:) = mat4(9,:);
end
% Combine the four matrices into a single 40x3 matrix
result_matrix = [mat1; mat2; mat3; mat4];

end
