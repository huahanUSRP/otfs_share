function TimeSymbols = OTFS_modulate(N,DDsymbols)
% FUNCTION: Convert DD grid symbols into the time domain using ifft
TimeSymbols = reshape(ifft(DDsymbols,N,2).*sqrt(N), [], 1);

end