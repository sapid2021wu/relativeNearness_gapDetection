% Half-wave rectification of input signal.

function  R_output = Rect(type, input, threshold)
% threshold: the cutoffs of the half-wave rectifiers.

R_output = input-threshold;
if type == -1
    R_output(R_output>0) = 0;
    R_output = abs(R_output);
else 
    R_output(R_output<0) = 0;
end

