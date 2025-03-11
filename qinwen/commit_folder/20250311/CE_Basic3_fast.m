function [H_DATA,pow_N] = CE_Basic3_fast(rxPilotSyms,M,N,pilotPara)
% Basic channel estimation for single point pilot, use about 13% runtime of CE_Basic2

% parameters
dftmtx_N = dftmtx(N)./sqrt(N);
idftmtx_N = conj(dftmtx_N);

MN = M*N;
pow_symbols = abs(rxPilotSyms(:)).^2;
pow_N = (sum(pow_symbols) - max(pow_symbols)) / (numel(pow_symbols)-1);

% rebuild the DD domain rxSymbols
noiseIndex = pow_symbols <= 1.2*pow_N;
if numel(pow_symbols) - sum(noiseIndex) >= 2*N
    % pow_symbols_sort = sort(pow_symbols,'descend');
    % noiseIndex = pow_symbols <= pow_symbols_sort(2*N);
    noiseIndex = pow_symbols <= 0.08*max(pow_symbols);
end
rxPilotSyms(noiseIndex) = 0;

% update pow_N
pow_N = mean(pow_symbols(noiseIndex));

% convert DD domain pilotSyms into time domain
rxPilotSyms_Time = ifft(reshape(rxPilotSyms,[],N),N,2).*sqrt(N);
pilotSyms = sqrt(db2pow(pilotPara.pilotPow) + 1).*...
    idftmtx_N(pilotPara.Position(2),:)./sqrt(pilotPara.pilotPow);
rxPilotSyms_Time = rxPilotSyms_Time./pilotSyms;
rxPilotSyms_Time_abs = abs(rxPilotSyms_Time);
rxPilotSyms_Time_phase = angle(rxPilotSyms_Time);
rxPilotSyms_Time_phase = unwrap(rxPilotSyms_Time_phase,[],2);

% interpolate the rest columns
pilotIndices = pilotPara.Position(1):M:MN;
estRxSyms_TimeAll_abs = interp1(pilotIndices,rxPilotSyms_Time_abs.',1:MN,"linear","extrap").';
estRxSyms_TimeAll_phase = interp1(pilotIndices,rxPilotSyms_Time_phase.',1:MN,"linear","extrap").';
estRxSyms_TimeAll = estRxSyms_TimeAll_abs.*exp(1j.*estRxSyms_TimeAll_phase);

% Build the sparse matrix H_DATA efficiently
[rowIdx, colIdx, values] = deal([]);

% reconstruce H_DATA
for indxN = 1:N
    estPilotSyms = idftmtx_N(indxN,:);
    for indxM = 1:M
        pilotShiftNum = indxM - ceil(M/2);
        pilotEstIndices = indxM:M:MN;
        estRxSyms = estRxSyms_TimeAll(:,pilotEstIndices).*estPilotSyms;
        % estRxSyms = fft(estRxSyms,N,2)./sqrt(N);
        % estRxSyms = estRxSyms*dftmtx_N;

        rowRange = mod(pilotPara.rxPilotIndices + pilotShiftNum - 1, MN) + 1;
        [rowRange,estRxSymsIdx] = sort(rowRange,'ascend');
        estRxSyms = reshape(estRxSyms(estRxSymsIdx),[],N)*dftmtx_N;
        % estRxSyms = fft(reshape(estRxSyms(estRxSymsIdx),[],N),N,2).*0.5;
        
        rowIdx = [rowIdx, rowRange];
        colIdx = [colIdx, repmat(indxM + (indxN-1)*M, 1, numel(rowRange))];
        values = [values, reshape(estRxSyms,[],1)];
    end
end
H_DATA = sparse(rowIdx, colIdx, double(values), MN, MN);
H_DATA = H_DATA(pilotPara.rxDataIndices,pilotPara.dataIndices);

end