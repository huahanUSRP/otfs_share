function [H_DATA,pow_N] = CE_Basic3_fast_multiframe(rxPilotSyms,M,N,pilotPara)
% FUCNTION: Basic channel estimation for single point pilot based on
% CE_Basic3_fast.m
%
% UPDATE: Input parameter rxPilotSyms is a matrix, column contains pilot
% symbols in one frame, row represents different frames

nFramePerBlock = size(rxPilotSyms,2);

% parameters
dftmtx_N = dftmtx(N)./sqrt(N);
idftmtx_N = conj(dftmtx_N);

MN = M*N;
pow_symbols = abs(rxPilotSyms).^2;
pow_symbols_avgFrame = mean(pow_symbols,2);
pow_N = (sum(pow_symbols_avgFrame) - max(pow_symbols_avgFrame)) / (numel(pow_symbols_avgFrame)-1);

% rebuild the DD domain rxSymbols
noise_threshold = 1.5*pow_N;
noiseIndex = pow_symbols_avgFrame <= noise_threshold;
if numel(pow_symbols_avgFrame) - sum(noiseIndex) >= 2*N
    % pow_symbol_avgFrame_sort = sort(pow_symbols_avgFrame,'descend');
    % noise_threshold = pow_symbol_avgFrame_sort(2*N);
    noise_threshold = 0.08*max(pow_symbols_avgFrame);
    noiseIndex = pow_symbols_avgFrame <= noise_threshold;
end
rxPilotSyms(noiseIndex,:) = 0;

% update pow_N
pow_N = mean(pow_symbols(noiseIndex,:),"all");

% convert DD domain pilotSyms into time domain
rxPilotSyms_Time = ifft(reshape(rxPilotSyms,[],N,nFramePerBlock),N,2);
pilotSyms = sqrt(db2pow(pilotPara.pilotPow) + 1).*...
    idftmtx_N(pilotPara.Position(2),:)./sqrt(pilotPara.pilotPow);
% pilotSyms = reshape(pilotSyms,1,[]);
rxPilotSyms_Time = rxPilotSyms_Time./pilotSyms;
rxPilotSyms_Time_abs = abs(rxPilotSyms_Time);
rxPilotSyms_Time_phase = angle(rxPilotSyms_Time);
rxPilotSyms_Time_phase = unwrap(rxPilotSyms_Time_phase,[],2);

% interpolate the rest columns
pilotIndices = pilotPara.Position(1):M:MN;
estRxSyms_TimeAll_abs = permute(interp1(pilotIndices,permute(rxPilotSyms_Time_abs,[2,1,3]),1:MN,"linear","extrap"),[2,1,3]);
estRxSyms_TimeAll_phase = permute(interp1(pilotIndices,permute(rxPilotSyms_Time_phase,[2,1,3]),1:MN,"linear","extrap"),[2,1,3]);
estRxSyms_TimeAll = estRxSyms_TimeAll_abs.*exp(1j.*estRxSyms_TimeAll_phase);

% Build the sparse matrix H_DATA efficiently
[rowIdx, colIdx] = deal([]);
values = cell(nFramePerBlock);
% reconstruce H_DATA
for indxN = 1:N
    estPilotSyms = idftmtx_N(indxN,:);
    for indxM = 1:M
        pilotShiftNum = indxM - ceil(M/2);
        pilotEstIndices = indxM:M:MN;
        estRxSyms = estRxSyms_TimeAll(:,pilotEstIndices,:).*estPilotSyms;

        rowRange = mod(pilotPara.rxPilotIndices + pilotShiftNum - 1, MN) + 1;
        [rowRange,estRxSymsIdx] = sort(rowRange,'ascend');
        estRxSyms = reshape(estRxSyms,[],nFramePerBlock);
        estRxSyms = estRxSyms(estRxSymsIdx,:);
        estRxSyms = fft(reshape(estRxSyms,[],N,nFramePerBlock),N,2);
        % estRxSyms = reshape(estRxSyms(estRxSymsIdx,:,:),[],N,nFramePerBlock)*dftmtx_N;
        
        rowIdx = [rowIdx, rowRange];
        colIdx = [colIdx, repmat(indxM + (indxN-1)*M, 1, numel(rowRange))];
        for indxFrame = 1:nFramePerBlock
            values{indxFrame} = [values{indxFrame}, reshape(estRxSyms(:,:,indxFrame),[],1)];
        end
    end
end

H_DATA = cell(nFramePerBlock,1);
for indxFrame = 1:nFramePerBlock
    H_DATA_temp = sparse(rowIdx, colIdx, double(values{indxFrame}), MN, MN);
    H_DATA{indxFrame} = H_DATA_temp(pilotPara.rxDataIndices,pilotPara.dataIndices);
end

end