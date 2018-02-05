function [ Mat_inv ] = InvertMatrix( Mat )
%INVERTMATRIX Summary of this function goes here
%   Detailed explanation goes here

eps = 0.001;
% CHeck determinant
determinant = det(Mat);
if determinant > eps
    % Okay, matrix is inversible.
    Mat_inv = inv(Mat);
else
    % Find the component that is problematic with singular value decomp.
    [U,S,V] = svd(Mat)
    S_value = eye(size(S));
    [a,~] = size(S);
    for J = 1:a
        if(S(J,J) < 0.0005)
            S_value(J,J) = 0;
        else
            S_value(J,J) = 1/S(J);
        end
    end
    Mat_inv = V * (S * U');
end


end

