function packet_data = addDDchannel(TimeSymbols_L, chDelay, chGainPow, chDoppler)

numTimeSymbols = length(TimeSymbols_L);
numPath = length(chGainPow);
chGainMag = db2mag(chGainPow);
chGainMag = chGainMag ./ sqrt(sum(chGainMag.^2));
chGain = complex(randn(1, numPath), randn(1, numPath)) .* chGainMag;
rxSigPathAll = zeros(numPath, numTimeSymbols + max(chDelay));
for indxPath = 1:numPath
    doppPhases = exp(1j*2*pi * chDoppler(indxPath) * (0:numTimeSymbols-1) );
    rxSigPathAll(indxPath, 1+chDelay(indxPath):numTimeSymbols+chDelay(indxPath)) = ...
        doppPhases .* TimeSymbols_L .* chGain(indxPath);
end
packet_data = sum(rxSigPathAll,1);
packet_data = packet_data(1:numTimeSymbols);

end