function rxSig = passChannel(txSig,channelParas,Ts,initIndex)

l_x = length(txSig);
pathDelayIntervals = round(channelParas.pathDelays/Ts);
rxSig = zeros(1,l_x + max(pathDelayIntervals));
for indxPath = 1:channelParas.numPath
    rxSig_p = channelParas.pathGains(indxPath)*txSig;
    doppPhases = Phase_Doppler(channelParas.pathDopplers(indxPath),Ts,l_x);
    doppPhases = reshape(doppPhases,1,[]);
    rxSig_p = doppPhases.*rxSig_p;
    rxSig(1+pathDelayIntervals(indxPath):l_x + pathDelayIntervals(indxPath)) = ...
                        rxSig(1+pathDelayIntervals(indxPath):l_x + pathDelayIntervals(indxPath)) + rxSig_p;
end
rxSig = rxSig(initIndex:end);

end

function phases = Phase_Doppler(Doppler, Ts, signalLength)
phases = 0:(signalLength-1);
phases = phases.*Doppler*Ts;
phases = exp(1j*2*pi*phases);
end