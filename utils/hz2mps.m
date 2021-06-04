function [mps] = hz2mps(hz, freq)

mps = hz .* Constants.CELERITY ./ freq;

end