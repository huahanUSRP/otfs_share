function [packet_data, usrpSymbolRate] = OTFS_USRP_Tx_text_FUNC_new_noupsample_fixSubSpacing(otfsSymbolRate, doppler_flag)
% OTFS USPR CODE for Transmitter, new preamble with 160 length, use TDL-D 
% parameters for multipath and Doppler simulation, modified based on 
% OTFS_USRP_Tx_text_FUNC_new_noupsample.m. 
% INPUT: 
% otfsSymbolRate: symbol rate/bandwidth (Hz) before upsampling.
% doppler_flag: set true to simulate multipath and Doppler channel.
%
% OUTPUT: 
% packet_data: USRP Tx vector containing multiple frames, symboling rate is
% defined as usrpSymbolRate. 
% usrpSymbolRate: USRP Tx sampling rate (Hz)
%
% UPDATE (compared to OTFS_USRP_Tx_text_FUNC_new_noupsample):
% OTFS frame structure changed according to the otfsSymbolRate to keep the
% same subcarrier spacing for different bandwidth.
%
% NOTE:
% The output of this Tx function only work in pair with 
% OTFS_USRP_Rx_Text_fixSubSpacing.m, can not work in previous Rx functions! 

% clc;
% close all;
% clear
% profile on

% otfsSymbolRate = 0.25e6;
% doppler_flag = false;
%% Key parameters

% control parameters
% doppler_flag = true;        % flag to add artifical Doppler in static testing
if doppler_flag
    switch round(otfsSymbolRate)
        case 0.25e6 % 250 kHz BW
            % save names
            rxSaveName = 'rx_otfs_USRP_Tx_text_250k_d_fixSub.mat';
            txSaveName = 'tx_otfs_USRP_Tx_text_250k_d_fixSub.sc';
        case 0.5e6 % 500 kHz BW
            rxSaveName = 'rx_otfs_USRP_Tx_text_500k_d_fixSub.mat';
            txSaveName = 'tx_otfs_USRP_Tx_text_500k_d_fixSub.sc';
        case 1e6 % 1 MHz BW
            rxSaveName = 'rx_otfs_USRP_Tx_text_1m_d_fixSub.mat';
            txSaveName = 'tx_otfs_USRP_Tx_text_1m_d_fixSub.sc';
        case 2e6
            rxSaveName = 'rx_otfs_USRP_Tx_text_2m_d_fixSub.mat';
            txSaveName = 'tx_otfs_USRP_Tx_text_2m_d_fixSub.sc';
        otherwise
            error('Bandwidth not supported! Please use 250 kHz, 500 kHz or 1 MHz.')
    end
else
    switch round(otfsSymbolRate)
        case 0.25e6 % 250 kHz BW
            % save names
            rxSaveName = 'rx_otfs_USRP_Tx_text_250k_nd_fixSub.mat';
            txSaveName = 'tx_otfs_USRP_Tx_text_250k_nd_fixSub.sc';
        case 0.5e6 % 500 kHz BW
            rxSaveName = 'rx_otfs_USRP_Tx_text_500k_nd_fixSub.mat';
            txSaveName = 'tx_otfs_USRP_Tx_text_500k_nd_fixSub.sc';
        case 1e6 % 1 MHz BW
            rxSaveName = 'rx_otfs_USRP_Tx_text_1m_nd_fixSub.mat';
            txSaveName = 'tx_otfs_USRP_Tx_text_1m_nd_fixSub.sc';
        case 2e6
            rxSaveName = 'rx_otfs_USRP_Tx_text_2m_nd_fixSub.mat';
            txSaveName = 'tx_otfs_USRP_Tx_text_2m_nd_fixSub.sc';
        otherwise
            error('Bandwidth not supported! Please use 250 kHz, 500 kHz or 1 MHz.')
    end
end


% Frame configure parameters
paraScaleRate = otfsSymbolRate/1e6;
CP = round(16*paraScaleRate);          % Cyclic Prefix Length, 32 for 2 MHz, 16 for 1 MHz, 8 for 500 kHz, and 4 for 250 kHz
carrierFreq = 5.9e9;        % Carrier Frequency, default as 5.9 GHz, used for simulating Doppler channel
% otfsSymbolRate = 1e6;       % otfs symbol rate, 1 MHz as default value
% usrpSymbolRate = 4e6;
usrpSymbolRate = otfsSymbolRate;       % usrp symbol rate, must be integer times of the otfsSymbolRate

% if usrpSymbolRate is higher than otfsSymbolRate, we need to do upsample
% and pulse shaping, currently only support integer upSampleRate
% upSampleRate = 4;
upSampleRate = round(usrpSymbolRate/otfsSymbolRate); 
Ts = 1/usrpSymbolRate;

% OTFS specific parameters, one OTFS block now contain 1 preamble and
% nFramePerBlock OTFS frames and CPs
nBlocks = 5e2;                                 % number of OTFS blocks to be generated at the same time for higher speed communication
nFramePerBlock = round(2/paraScaleRate);       % number of OTFS frames in one block
M = round(200*paraScaleRate);                  % number of delay bins, set to be 200, not sure if this works on USRP
N = 4;                                         % number of Doppler bins in DD grid, also the FFT size for OTFS
L = ceil(6*paraScaleRate);                     % maximum supported channel delay in bins
preambleLength = 160;                          % preamble length, fixed for all different bandwidth

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
randDoppler(1) = 600;       % set the main path doppler to our desired value 
channelParas = struct();
channelParas.numPath = numPath;
channelParas.pathGains = chGain;
channelParas.pathDelays = 363*[0, 0, 0.035, 0.612, 1.363, 1.405, 1.804, 2.596, 1.775,...
        4.042, 7.937, 9.424, 9.708, 12.525]./1e9; % channel delay in s                   
channelParas.pathDopplers = randDoppler;        % channel Doppler in times of subcarrier spacing

%% Generate OTFS Tx symbols and packet data
rng(0);
% generate bit stream
% codeBits = randi([0,1],nFrame*2*(M*N - pilotPara.numGuardBins),1);
nBitsPerBlock = 2*numel(pilotPara.dataIndices)*nFramePerBlock;
% currently message length should be limited to put all messageBits in one
% OTFS block, can be shorter ( numel(nBitsPerBlock) <= nBitsPerBlock-8 ) 
message = 'Orthogonal Time Frequency Space (OTFS) is an communication technique that enhances data transmission in high-mobility environments,ensuring improved reliability and efficiency';
numAddEndSpaces = floor(nBitsPerBlock/8) - 1 - numel(message);
if numAddEndSpaces < 0
    error('message is too long!')
end
messageBinary = reshape(dec2bin(message, 8)', 1, []);
messageBinary = [messageBinary, ones(1,8), repmat([0 0 1 0 0 0 0 0],1,numAddEndSpaces)];
codeBits = messageBinary == '1';
% codeBits = message2codeBits(message, nBitsPerBlock);
% messageBinary = reshape(dec2bin(message, 8)', 1, []);
% messageData = messageBinary == '1';
% codeBits = messageData.'; %[messageData zeros(6,4)]';
% codeBits = repmat(codeBits,nBlocks,1);


% mapping to QPSK symbols
txSyms = lteSymbolModulate(codeBits,'QPSK');
txSyms = reshape(txSyms,[],nFramePerBlock);
txSyms = repmat(txSyms,1,1,nBlocks);

% DD grid formation

txDDMtx = zeros(M*N,nFramePerBlock,nBlocks);
txDDMtx(pilotPara.dataIndices,:,:) = txSyms;
txDDMtx(pilotPara.pilotIndex,:,:) = db2mag(pilotPara.pilotPow);
txDDMtx = reshape(txDDMtx, M, N, nFramePerBlock, nBlocks);

% OTFS modulation, note that here time symbol for one block is a column
% vector, not a row vector
TimeSymbols = reshape(ifft(txDDMtx,N,2).*sqrt(N), [], nFramePerBlock, nBlocks);

% Add CP
TimeSymbols_CP = cat(1, TimeSymbols(end-CP+1:end,:,:), TimeSymbols);
TimeSymbols_CP = reshape(TimeSymbols_CP, [], nBlocks);

% Add preamble, preamble is fixed for different frames
preamble = zadoffChuSeq(1,preambleLength+1);
preamble = preamble(1:end-1);
TimeSymbols_L = [repmat(preamble(:),1,nBlocks); TimeSymbols_CP];
TimeSymbols_L = reshape(TimeSymbols_L,1,[]);

% upsampling and apply srrc pulse
if upSampleRate ~= 1
    rolloffFactor = 0.25;
    Q = 8;
    srrcFilt = comm.RaisedCosineTransmitFilter(...
      'Shape',                  'Square root', ...
      'RolloffFactor',          rolloffFactor, ...
      'FilterSpanInSymbols',    2*Q, ...
      'OutputSamplesPerSymbol', upSampleRate);
    srrcPulse = srrcFilt.impz;

    TimeSymbols_L = upsample(TimeSymbols_L, upSampleRate);
    TimeSymbols_L = conv(TimeSymbols_L, srrcPulse);
    cutStart = ceil(numel(srrcPulse)/2);
    cutEnd = cutStart - numel(srrcPulse);
    TimeSymbols_L = TimeSymbols_L(cutStart:end+cutEnd);
end

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

%% Save data
save(rxSaveName,...
    "packet_data","preamble","codeBits",...
    "M","N","L","CP","pilotPara","doppler_flag");
fid = fopen(txSaveName,'wb');
new_save = reshape([real(packet_data);imag(packet_data)],1,[]);
F = fwrite(fid, new_save, 'float');
fclose all;

% profile off
% profile viewer

end