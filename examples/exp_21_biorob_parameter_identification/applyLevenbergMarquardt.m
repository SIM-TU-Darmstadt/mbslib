% ------------------------------------------------------
% This function applies the Levenberg-Marquardt algorithm.
% f:        Matlab function which returns a variable containing a vector of
%           the residues (n_r x 1) and a variable containing an array of
%           the Jacobians (n_p x n_r)
% p0:       Initial value
% mu0:      Initial factor
% beta0:    Lower bound
% beta1:    Upper bound
% k_max:    Iteration limit
% T:        Tolerance limit with respect to ||d||
% p:        Optimal solution
% hist:     Array containing the history of the optimization process with
%           the current iteration count, the current value of the ojective
%           function and the current step width per row
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization, and Robotics Group
% Written by Janis Wojtusch, 2015
% ------------------------------------------------------

function [p, hist] = applyLevenbergMarquardt(f, p0, mu0, beta0, beta1, k_max, T)

n = length(p0);
k = 0;
mu = mu0;
p = p0;
[r, J] = feval(f, p);
d = -(J * J' + mu^2 * eye(n)) \ (J * r);
hist((k + 1), :) = [k, (r' * r), norm(d)];
while (norm(d) > T) && (k < k_max)

    fprintf('STATUS: Running iteration %i of %i mit p = [%.1f %.1f %.1f %.1f %.2f %.2f %.2f %.2f].\n', k, k_max, p(1), p(2), p(3), p(4), p(5), p(6), p(7), p(8));
    [d, mu, r] = correct(f, p, mu, beta0, beta1);
    p = p + d;
    k = k + 1;
    hist((k + 1), :) = [k, (r' * r), norm(d)];

end
if k >= k_max
    
    fprintf('ERROR: Iteration limit is exceeded!\n');
    return;

end

end

function [d, mu, r] = correct(f, p, mu, beta_0, beta_1)

n = length(p);
[r_1, J_1] = feval(f, p);
d = -(J_1 * J_1' + mu^2 * eye(n)) \ (J_1 * r_1);
[r_2, ~] = feval(f, (p + d));
r = r_2;
eps_mu = (r_1' * r_1 - r_2' * r_2) / (r_1' * r_1 - (r_1 + J_1' * d)' * (r_1 + J_1' * d));
if eps_mu < 0
    
    r = r_1;
    d = zeros(size(d));
    
elseif eps_mu <= beta_0
    
    [d, mu, r] = correct(f, p, (2 * mu), beta_0, beta_1);

elseif eps_mu >= beta_1

    mu = mu / 2;

end

end
