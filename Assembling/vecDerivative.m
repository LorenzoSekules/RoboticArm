function df = vecDerivative(f, h)
% Compute derivative of a vector using finite differences
% Output has the same length as input

n = numel(f);
df = zeros(size(f));

% Forward difference (first element)
df(1) = (f(2) - f(1)) / h;

% Central differences (interior)
df(2:n-1) = (f(3:n) - f(1:n-2)) / (2*h);

% Backward difference (last element)
df(n) = (f(n) - f(n-1)) / h;

end