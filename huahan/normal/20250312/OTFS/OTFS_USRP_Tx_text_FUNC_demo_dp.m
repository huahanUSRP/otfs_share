function packet_data = OTFS_USRP_Tx_text_FUNC_demo_dp(otfsSymbolRate)
% OTFS USPR CODE for Transmitter, new preamble with 160 length, use TDL-D 
% parameters for multipath and Doppler simulation 

% clc;
% close all;
% clear
% profile on
%% Key parameters

% control parameters
doppler_flag = true;        % flag to add artifical Doppler in static testing
rxSaveName = 'rx_otfs_USRP_Tx_text_dp.mat';
switch round(otfsSymbolRate)
    case 0.25e6 % 250 kHz BW
        txSaveName = 'tx_otfs_USRP_Tx_text_250k_d.sc';
    case 0.5e6 % 500 kHz BW
        txSaveName = 'tx_otfs_USRP_Tx_text_500k_d.sc';
    case 1e6 % 1 MHz BW
        txSaveName = 'tx_otfs_USRP_Tx_text_1m_d.sc';
    case 2e6
        txSaveName = 'tx_otfs_USRP_Tx_text_2m_d.sc';
    otherwise
        error('Bandwidth not supported! Please use 250 kHz, 500 kHz or 1 MHz.')
end


% USRP configure parameters
nPointsFFT = 64;            % number of total subcarriers used in OFDM codes
CP = nPointsFFT/4;          % Cyclic Prefix Length
carrierFreq = 5.9e9;        % Carrier Frequency, default as 5.9 GHz
%otfsSymbolRate = 1e6;       % otfs symbol rate, 1 MHz as default value
usrpSymbolRate = otfsSymbolRate;       % usrp symbol rate, must be integer times of the otfsSymbolRate
%usrpSymbolRate =4e6;
% if usrpSymbolRate is higher than otfsSymbolRate, we need to do upsample
% and pulse shaping, currently only support integer upSampleRate
upSampleRate = round(usrpSymbolRate/otfsSymbolRate) 
%upSampleRate = 4;
Ts = 1/usrpSymbolRate;
%Ts = 1/4e6;
rolloffFactor = 0.25;
Q = 8;
srrcFilt = comm.RaisedCosineTransmitFilter(...
  'Shape',                  'Square root', ...
  'RolloffFactor',          rolloffFactor, ...
  'FilterSpanInSymbols',    2*Q, ...
  'OutputSamplesPerSymbol', upSampleRate);
srrcPulse = srrcFilt.impz;

% OTFS specific parameters
nFrame = 1;               % number of OTFS frames to be generated at the same time for higher speed communication
M = 200;                    % number of delay bins, set to be 200, not sure if this works on USRP
N = 4;                      % number of Doppler bins in DD grid, also the FFT size for OTFS
L = 6;                      % maximum supported channel delay in bins

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
% codeBits = randi([0,1],nFrame*2*(M*N - pilotPara.numGuardBins),1);
message = 'Orthogonal Time Frequency Space (OTFS) is an communication technique that enhances data transmission in high-mobility environments,ensuring improved reliability and efficiency';
messageBinary = reshape(dec2bin(message, 8)', 1, []);
messageData = messageBinary == '1';
codeBits = messageData.'; %[messageData zeros(6,4)]';
codeBits = repmat(codeBits,nFrame,1);


% mapping to QPSK symbols
txSyms = lteSymbolModulate(codeBits,'QPSK');

% DD grid formation

txDDMtx = zeros(M*N,nFrame);
txDDMtx(pilotPara.dataIndices,1:nFrame) = reshape(txSyms,[],nFrame);
txDDMtx(pilotPara.pilotIndex,1:nFrame) = db2mag(pilotPara.pilotPow);
txDDMtx = reshape(txDDMtx, M, N, nFrame);

% OTFS modulation, note that here time symbol for one block is a column
% vector, not a row vector
TimeSymbols = reshape(ifft(txDDMtx,N,2).*sqrt(N), [], nFrame);

% Add CP
TimeSymbols_CP = [TimeSymbols(end-CP+1:end,:); TimeSymbols];

% Add preamble, preamble is fixed for different frames
preamble = zadoffChuSeq(1,161);
preamble = preamble(1:end-1);
TimeSymbols_L = [repmat(preamble(:),1,nFrame); TimeSymbols_CP];
TimeSymbols_L = reshape(TimeSymbols_L,1,[]);

% upsampling and apply srrc pulse
if upSampleRate ~= 1
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
    "M","N","L","nPointsFFT","CP","pilotPara","doppler_flag");
fid = fopen(txSaveName,'wb');
new_save = reshape([real(packet_data);imag(packet_data)],1,[]);
F = fwrite(fid, new_save, 'float');
fclose all;

% profile off
% profile viewer

end