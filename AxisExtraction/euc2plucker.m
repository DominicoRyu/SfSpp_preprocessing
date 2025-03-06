function line_plucker = euc2plucker(line_euc, normalize)
%EUC2PLUCKER Convert a line in Euclidean coordinates to Plucker
%coordinates.
%   Input:  line_euc = [l; c] where l and c define 3D direction and
%   position vectors respectively. (i.e. 6-D)
%   Output: line_plucker = [l; lbar] where l is the direction vector and
%   lbar is the Plucker moment vector.

if nargin < 2, normalize = true;
end

if normalize
    line_euc(1:3,:) = bsxfun(@rdivide, line_euc(1:3,:), sqrt(sum(line_euc(1:3,:).^2)));
end

line_plucker = [line_euc(1:3,:); cross(line_euc(4:6,:), line_euc(1:3,:))];

end

