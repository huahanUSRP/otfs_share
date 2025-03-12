function [output_symbols,preamble_freq, preamble] = addPreamble(input_symbols,N)
    % Generate LTF symbols
    preamble_freq = genFrequencySymbols();
    preamble = genTimeSymbols(preamble_freq,N);
    
    % Append to the input symbols
    output_symbols = [preamble, input_symbols];
 
end

function preamble_freq = genFrequencySymbols()
  preamble_freq = complex(zeros(1, 64));
    preamble_freq(39:64) = [1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1, 1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1];
    preamble_freq(1:27) =    [0, 1,-1,-1, 1, 1,-1, 1,-1, 1,-1,-1,-1,-1,-1, 1, 1,-1,-1, 1,-1, 1,-1, 1, 1, 1, 1];
end

function preamble = genTimeSymbols(preamble_freq,N)
    preamble_single = ifft(preamble_freq)*sqrt(N);
    preamble = [preamble_single(end-31:end), preamble_single, preamble_single];
end