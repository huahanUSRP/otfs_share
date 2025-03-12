function preamble = GetPreambleOnly(N)
    % Generate LTF symbols
    preamble_freq = complex(zeros(1, 64));
    preamble_freq(39:64) = [1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1, 1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1];
    preamble_freq(1:27) =    [0, 1,-1,-1, 1, 1,-1, 1,-1, 1,-1,-1,-1,-1,-1, 1, 1,-1,-1, 1,-1, 1,-1, 1, 1, 1, 1];
    preamble_single = ifft(preamble_freq)*sqrt(N);
    preamble = [preamble_single(end-31:end), preamble_single, preamble_single];
end
