function result_matrix = combine_matrices(mat1, mat2, mat3, mat4)

% Duplicate the last row of each input matrix and add it as the 10th row
mat1(10,:) = mat1(9,:);
mat2(10,:) = mat2(9,:);
mat3(10,:) = mat3(9,:);
mat4(10,:) = mat4(9,:);

% Combine the four matrices into a single 40x3 matrix
result_matrix = [mat1; mat2; mat3; mat4];

end
