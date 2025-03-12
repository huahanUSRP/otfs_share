
txSaveName = 'tx_fifo';
doppler_flag = true;        % flag to add artifical Doppler in static testing
otfsSymbolRate = 1e6;
message = 'Orthogonal Time Frequency Space (OTFS) is an communication technique that enhances data transmission in high-mobility environments,ensuring improved reliability and efficiency';
text_input = message';
% USRP configure parameters
nPointsFFT = 64;            % number of total subcarriers used in OFDM codes
cp_size = nPointsFFT/4;          % Cyclic Prefix Length
carrierFreq = 5.9e9;        % Carrier Frequency, default as 5.9 GHz
usrpSymbolRate = otfsSymbolRate;       % usrp symbol rate, must be integer times of the otfsSymbolRate


Ts = 1/usrpSymbolRate;
rolloffFactor = 0.25;
Q = 8;
srrcFilt = comm.RaisedCosineTransmitFilter(...
  'Shape',                  'Square root', ...
  'RolloffFactor',          rolloffFactor, ...
  'FilterSpanInSymbols',    2*Q, ...
  'OutputSamplesPerSymbol', upSampleRate);
srrcPulse = srrcFilt.impz;

% OTFS specific parameters
nBlocks = 1;                % number of OTFS blocks to be generated at the same time for higher speed communication
M = 200;                    % number of delay bins, set to be 200, not sure if this works on USRP
N = 4;                      % number of Doppler bins in DD grid, also the FFT size for OTFS
L = 6;                      % maximum supported channel delay in bins
preambleLength = 160;       % preamble length, fixed for all different bandwidth

% DD grid pilot parameters
pilotPowRelativeDB = 13;            % relative pilot power in dB
pilotPara = SinglePilotParameters(M,N,L,pilotPowRelativeDB);

% Doppler simulation parameters
maxSpeed = 120;             % maximum speed in km/h
maxDoppler = maxSpeed/3.6*carrierFreq/3e8; % maximum speed in Hz
chGainPow = db2pow([-0.2, -13.5, -18.8, -21, -22.8, -17.9,...
            -20.1, -21.9, -22.9, -27.8, -23.6, -24.8, -30.0, -27.7]);                   % channel gains in dB
chGainPow = chGainPow./sum(chGainPow);
rng(1)
numPath = numel(chGainPow);
chGain = complex(randn(1,numPath),randn(1,numPath));
chGain = chGain./abs(chGain);
chGain = sqrt(chGainPow).*chGain;
randAoA = 2*pi*rand(1,numPath);
randDoppler = maxDoppler.*cos(randAoA);
randDoppler(1) = 600; %90km/hour
channelParas = struct();
channelParas.numPath = numPath;
channelParas.pathGains = chGain;
channelParas.pathDelays = 363*[0, 0, 0.035, 0.612, 1.363, 1.405, 1.804, 2.596, 1.775,...
        4.042, 7.937, 9.424, 9.708, 12.525]./1e9; % channel delay in s                   
channelParas.pathDopplers = randDoppler;        % channel Doppler in times of subcarrier spacing

%% Generate OTFS Tx symbols and packet data
rng(0);
% generate bit stream
% codeBits = randi([0,1],nBlocks*2*(M*N - pilotPara.numGuardBins),1);
fid = fopen('tx_fifo','wb');
preamble = zadoffChuSeq(1,161);
preamble = preamble(1:end-1);
frame_rep_times = 8;

% Create UDP receiver for text input and bandwidth input
udpReceiver_text = dsp.UDPReceiver('LocalIPPort', 12345,'MaximumMessageLength', 175,'MessageDataType', 'uint8');         % The type of incoming data (you can change this)
udpReceiver_bandwidth = dsp.UDPReceiver('LocalIPPort', 34567,'MaximumMessageLength', 8,'MessageDataType', 'uint8');         % The type of incoming data (you can change this)

packet_data = zeros(1,976);

while(1)
     
    data = step(udpReceiver_text);  % Receive the data packet from UDP socket
    if ~isempty(data)
        %disp(['Received ' num2str(length(data)) ' bytes of data']);
        text_input=data';
    end
 
    bw = step(udpReceiver_bandwidth);
    if ~isempty(bw)
        %disp(['Received ' num2str(length(bw)) ' bytes of data']);
        bw_input=bw(4:5,:)';
        decimal_bw = typecast(bw_input, 'uint16');
        usrpSymbolRate = uint32(decimal_bw)*1000;
        Ts = 1/double(usrpSymbolRate);
        frame_rep_times = round(decimal_bw/250);    %get packet_data repeat times
    end

    %% Merged code: setup parameters based on usrpSymbolRate
    paraScaleRate = usrpSymbolRate/1e6;
    cp_size = round(16*paraScaleRate);             % cp size
    nFramePerBlock = round(2/paraScaleRate);       % number of OTFS frames in one block
    M = round(200*paraScaleRate);                  % number of delay bins, set to be 200, not sure if this works on USRP
    L = ceil(6*paraScaleRate);                     % maximum supported channel delay in bins

    nBitsPerBlock = 2*numel(pilotPara.dataIndices)*nFramePerBlock;  % calculate the number of data bits that can be fit into one block
    numAddEndSpaces = floor(nBitsPerBlock/8) - numel(message);      % number of added spaces to fill the rest of data bits in one block
    if numAddEndSpaces < 0
        error('message is too long!')
    end
    messageBinary = [reshape(dec2bin(text_input, 8)', 1, []), repmat('00100000',1,numAddEndSpaces)];
    messageData = messageBinary == '1';
    codeBits = messageData.'; %[messageData zeros(6,4)]';
    codeBits = repmat(codeBits,nBlocks,1);
    
    % mapping to QPSK symbols
    txSyms = lteSymbolModulate(codeBits,'QPSK');
    txSyms = reshape(txSyms,[],nFramePerBlock);
    txSyms = repmat(txSyms,1,1,nBlocks);
    
    % DD grid formation
    txDDMtx = zeros(M*N,nFramePerBlock,nBlocks); % move out of loop
    txDDMtx(pilotPara.dataIndices,:,:) = txSyms;
    txDDMtx(pilotPara.pilotIndex,:,:) = db2mag(pilotPara.pilotPow);
    txDDMtx = reshape(txDDMtx, M, N, nFramePerBlock, nBlocks);
    
    % OTFS modulation, note that here time symbol for one block is a column
    % vector, not a row vector
    TimeSymbols = reshape(ifft(txDDMtx,N,2).*sqrt(N), [], nFramePerBlock, nBlocks);
    
    % Add CP
    TimeSymbols_CP = cat(1, TimeSymbols(end-cp_size+1:end,:,:), TimeSymbols);
    TimeSymbols_CP = reshape(TimeSymbols_CP, [], nBlocks);
    
    % Add preamble, preamble is fixed for different frames
    TimeSymbols_L = [repmat(preamble(:),1,nBlocks); TimeSymbols_CP];
    TimeSymbols_L = reshape(TimeSymbols_L,1,[]);
    
    % upsampling and apply srrc pulse
    % if upSampleRate ~= 1
    %     TimeSymbols_L = upsample(TimeSymbols_L, upSampleRate);
    %     TimeSymbols_L = conv(TimeSymbols_L, srrcPulse);
    %     cutStart = ceil(numel(srrcPulse)/2);
    %     cutEnd = cutStart - numel(srrcPulse);
    %     TimeSymbols_L = TimeSymbols_L(cutStart:end+cutEnd);
    % end
    
    % delay-Doppler channel 
    if doppler_flag
        packet_data = passChannel(TimeSymbols_L, channelParas, Ts, 1);
        packet_data_overlength = numel(packet_data) - numel(TimeSymbols_L);
        packet_data(1:packet_data_overlength) = packet_data(1:packet_data_overlength) + ...
            packet_data(end-packet_data_overlength+1:end);
        packet_data = packet_data(1:numel(TimeSymbols_L));
    else
        packet_data = TimeSymbols_L;
    end
    
    % normalize packet_data (if needed)
    packet_data = packet_data./max(abs(packet_data));
    %repeat sending data based on bandwidth to make realtime transmisson has proper data feed speed
    packet_data_full = repmat(packet_data, 1, frame_rep_times);
    new_save = reshape([real(packet_data_full);imag(packet_data_full)],1,[]);
    F = fwrite(fid, new_save, 'float');
%toc
end
fclose all;

% profile off
% profile viewer
