function TimeSymbols = OTFS_modulate_phaseAdjust(M,N,DDsymbols)
% FUNCTION: Convert DD grid symbols into the time domain using ifft

MN = M*N;
phaseAdjust = exp(-1j*pi*2*(1:MN)/2);
phaseAdjust = reshape(phaseAdjust,[],1);
TimeSymbols = reshape(ifft(DDsymbols,N,2).*sqrt(N), [], 1) .* phaseAdjust;

end