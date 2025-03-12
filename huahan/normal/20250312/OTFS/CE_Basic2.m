function [H_DATA,pow_N] = CE_Basic2(rxPilotSyms,M,N,pilotPara)
% Basic channel estimation for single point pilot

% parameters
dftmtx_N = dftmtx(N)./sqrt(N);
idftmtx_N = conj(dftmtx_N);

MN = M*N;
pow_symbols = abs(rxPilotSyms(:)).^2;
pow_N = (sum(pow_symbols) - max(pow_symbols)) / (numel(pow_symbols)-1);

% rebuild the DD domain rxSymbols
noiseIndex = abs(rxPilotSyms) <= 1.5*sqrt(pow_N);
rxPilotSyms(noiseIndex) = 0;
rxSymsDD = zeros(M,N);
rxSymsDD(pilotPara.rxPilotIndices) = rxPilotSyms;

% convert rxSymsDD back to time domain
rxSymsTime = reshape(rxSymsDD*idftmtx_N,[],1);

% calculate the pilot spread in time domain
pilotSyms = sqrt(db2pow(pilotPara.pilotPow) + 1).*...
    idftmtx_N(pilotPara.Position(2),:)./sqrt(pilotPara.pilotPow);
pilotIndices = pilotPara.Position(1):M:MN;
pilotShiftNum = ceil(M/2) - pilotPara.Position(1);

% cut the rxSymsTime into sequences
rxSymsTimeShift = circshift(rxSymsTime,pilotShiftNum);
rxSyms = reshape(rxSymsTimeShift,M,N)./pilotSyms;

% reconstruct the H_DATA
H_DATA = zeros(MN);
for indxN = 1:N
    estPilotSyms = idftmtx_N(indxN,:);
    for indxM = 1:M
        pilotShiftNum = indxM - ceil(M/2);
        % do linear combination
        pilotEstIndices = indxM:M:MN;
        estRxsyms = interp1(pilotIndices,rxSyms.',pilotEstIndices,"linear","extrap");
        estRxsyms = estRxsyms.';
        estRxSyms = estRxsyms.*estPilotSyms;
        estRxSymsTime = reshape(estRxSyms,[],1);
        estRxSymsTime = circshift(estRxSymsTime,pilotShiftNum);
        H_DATA(:,indxM + (indxN-1)*M) = reshape(reshape(estRxSymsTime,M,N)*dftmtx_N,[],1);
    end
end

H_DATA = H_DATA(pilotPara.rxDataIndices,pilotPara.dataIndices);

end