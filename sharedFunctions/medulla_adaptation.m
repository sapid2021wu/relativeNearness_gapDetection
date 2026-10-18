% The contrast adaptation mechanism.

function    [On_norm, Off_norm] = medulla_adaptation(On0, Off0, b)
% filter_cn: receptive field for the contrast adaptation mechanism.  
% b: sensitivity factor.

filter_cn = fspecial('gaussian', 17, 3);           
On_cn = conv2(On0, filter_cn, 'same');
Off_cn = conv2(Off0, filter_cn, 'same');
x_on = On0./(On_cn + b);
x_off = Off0./(Off_cn + b);
On_norm = tanh_function(x_on);
Off_norm = tanh_function(x_off);

end

function y = tanh_function(x)
y = 2./(1 + exp(-2*x)) - 1;
end

