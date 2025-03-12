function pilotPara = BlockPilotParameters(M,N,L,blockPilotPosition,totalPilotPow)
% FUNCTION: determine the pilot parameters for block pilot design for OTFS


% block pilot parameters position
pilotPara = struct();
pilotPara.Position = blockPilotPosition;
pilotPara.GuardRange = [pilotPara.Position(1)-2*L, ...
    pilotPara.Position(2)+2*L, ...
    1,...
    N];
pilotPara.RxPilotRange = [pilotPara.Position(1)-L, ...
    pilotPara.Position(2)+L, ...
    1,...
    N];
numPilotBins = (pilotPara.Position(2) - pilotPara.Position(1) + 1)*...
    (pilotPara.Position(4) - pilotPara.Position(3) + 1);
pilotPara.numGuardBins = (pilotPara.GuardRange(2) - pilotPara.GuardRange(1) + 1)*...
    (pilotPara.GuardRange(4) - pilotPara.GuardRange(3) + 1);
pilotPara.pilotPow = pow2db(db2pow(totalPilotPow)/numPilotBins); % pilot relative power(dB)

% find linear indices for Pilot, rxPilot and data
[allRows, allCols] = ndgrid(1:M, 1:N);
[pilotRows,pilotCols] = ndgrid(pilotPara.Position(1):pilotPara.Position(2),...
    pilotPara.Position(3):pilotPara.Position(4));
[guardRows, guardCols] = ndgrid(pilotPara.GuardRange(1):pilotPara.GuardRange(2), ...
    pilotPara.GuardRange(3):pilotPara.GuardRange(4));
[rxPilotRows, rxPilotCols] = ndgrid(pilotPara.RxPilotRange(1):pilotPara.RxPilotRange(2), ...
    pilotPara.RxPilotRange(3):pilotPara.RxPilotRange(4)); 
allIndices = sub2ind([M,N],allRows(:),allCols(:));
pilotIndex = sub2ind([M,N],pilotRows(:),pilotCols(:));
guardIndices = sub2ind([M,N],guardRows(:),guardCols(:));
rxPilotIndices = sub2ind([M,N],rxPilotRows(:),rxPilotCols(:));
dataIndices = setdiff(allIndices,guardIndices);
rxDataIndices = setdiff(allIndices,rxPilotIndices);

pilotPara.rxPilotIndices = rxPilotIndices;
pilotPara.pilotIndex = pilotIndex;
pilotPara.dataIndices = dataIndices;
pilotPara.rxDataIndices = rxDataIndices;

end