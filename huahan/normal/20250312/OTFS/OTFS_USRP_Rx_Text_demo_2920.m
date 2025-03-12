% function packet_data = OTFS_USRP_Rx_Text_FixUSRPRate(otfsSymbolRate)
% OTFS USRP CODE for Receiver
% Use the same structure as 'OTFS_USRP_Rx_AllTxRxBand_text.m', Rx USRP 
% sampling rate is fixed at 4 MHz, sampling ratio changed according to the
% estimated bandwidth (250 kHz, 500 kHz and 1 MHz).
clear;

%% flag setting

simulate = false;
oneusrp = false;
streaming = false;
uhd = true;
profile_flag = false;
switch_cnt = false;


RxSaveName_default = 'rx_otfs_USRP_Tx_text.mat';
if oneusrp
    load Rx
else
    load(RxSaveName_default);               % read data
end

frame_cnt = 0;
snrdba1 = 0;
preamble_size = numel(preamble);
data_size = M*N;
cp_size = 16;
frame_size = preamble_size + data_size;
mainDopplerEst1 = [];

% Doppler estimation parameters
indxDoppler = 0;
numDoppEstFrame = 30;
rxMainPath_f = zeros(N,numDoppEstFrame);

%% Tx and Rx sampling rate setting
samplingRateRx = 4e6; % Rx USRP sampling rate, must be integer times of Tx rate
% samplingRatio = samplingRateRx / samplingRateTx; % This ratio must be an integer


%% Matched filter
matched_filter = conj(fliplr(preamble));
norm_matched_filter = matched_filter / norm(matched_filter);


%% Synchronization MF
offfset = 0;
vars = whos;

for i = 1:numel(vars)
    varName = vars(i).name;
    if strcmp(vars(i).class, 'double')
        % Convert to single
        assignin('base', varName, single(eval(varName)));
    end
end

Rx_gain=10;


if ~uhd
    if streaming
        fid = fopen('./rx_fifo','rb');         % USRP interface
    else
        fid = fopen('rx_fifo_file_1M_norrc','rb'); 
    end
end

%% Main Loop for processing
while (1)
    try
        %% uhd read case
        if uhd
            

            system('sudo /home/otfs3/git/uhd-master/host/build/examples/rx_samples_to_file --file out1.dat --type float --nsamps 1200000 --rate 4e6 --freq 2.0e9 --gain 20  --bw 2e5 --ref internal --ant="TX/RX" --channel=0');

            file_name = 'out1.dat';
            fid=fopen(file_name,'rb');          % Recorded data with Tx and Rx gain
            [rx_temp,COUNT]=fread(fid, 12e5, 'float');
            fprintf('COUNT: %d\n',COUNT);       % Modified for better view
            % rx_temp = single(rx_temp);
            status = fclose(fid);               % Add a semicolon here
            fprintf('fclose: %d\n',status);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        else 
            [rx_temp,COUNT] = fread(fid, 1e6, 'single');
            % rx_temp = single(rx_temp);
        end

        tic;
        
        rx_mat = reshape(rx_temp, 2, []);       % reshape will not change the type of variable
        packet_data = complex(rx_mat(1, :), rx_mat(2, :));

        % estimate the bandwidth of the packet_data
        [pxx,fSample] = pwelch(packet_data,800,[],[],samplingRateRx);
        pxx_dB = pow2db(pxx);
        fSample = fSample - mean(fSample);
        pxx_dB = fftshift(pxx_dB);
        [peak_power, peak_idx] = max(pxx_dB);
        reference_power = mean([peak_power - 15, mean(pxx_dB)]);
        lower_idx = find(pxx_dB(1:peak_idx) >= reference_power, 1, 'first');
        upper_idx = peak_idx + find(pxx_dB(peak_idx:end) >= reference_power, 1, 'last') - 1;
        flo = fSample(lower_idx);
        fhi = fSample(upper_idx);
        bw_15dB = fhi-flo;
        if bw_15dB < 0.375e6
            bw_15dB = 0.25e6;
        elseif bw_15dB < 0.75e6
            bw_15dB = 0.5e6;
        elseif bw_15dB < 1.5e6
            bw_15dB = 1e6;
        else
            bw_15dB = 2e6;
        end
        samplingRatio = round(samplingRateRx./bw_15dB);
        matched_filter_upsample = upsample(norm_matched_filter,samplingRatio);

        cfo_temp = -1j*2*pi*(offfset)*(0:numel(packet_data)-1)/samplingRateRx; % will not change the value of the detected starts
        rx_CFO_fine = packet_data.*exp(cfo_temp);
        packet_data = rx_CFO_fine;

        [detected_starts,sync_op] = Sync2_Modify(packet_data,matched_filter_upsample,samplingRatio*frame_size-1);

        clf
        subplot(3,1,1)
        plot(1:length(sync_op),abs(sync_op)); grid on; title('Matched Filter output')

        % plot(real(packet_data1));hold on;
        % xlabel('Samples');ylabel('Amplitude')
        % stem(detected_starts,max(real(packet_data1))*ones(1,length(detected_starts)),'r^','linewidth',1.5);
        % %ylim([-max(real(packet_data1))-.02 max(real(packet_data1))+.2])
        % title('Receive signal and Timing reference in I channel');

        packet_data = packet_data(detected_starts(1):samplingRatio:end);
        detected_starts_diff = diff(detected_starts);
        detected_starts(1) = 1;
        detected_starts(2:end) = 1 + cumsum(round(detected_starts_diff/samplingRatio));
        data_sym_len = round(detected_starts_diff(1)/samplingRatio);

        initial_idx_temp = detected_starts+1;
        Preamble_initial_idx = initial_idx_temp - length(preamble);

        Preamble_initial_idx = Preamble_initial_idx(1:end-1);
        if(Preamble_initial_idx(1)<=0)
            Preamble_initial_idx = Preamble_initial_idx(2:end);
        end

        dis = diff(Preamble_initial_idx);
        if dis(1) < frame_size
            Preamble_initial_idx = Preamble_initial_idx(2:end);
        end
        initial_idx = Preamble_initial_idx + numel(preamble);

        % No zeros added
        if Rx_gain == 10
            np= 1.3e-08;
        else
            np= 4.972e-08;
        end

        sp = rms(packet_data(Preamble_initial_idx(2):end)).^2;
        snr = (sp-np)/np;
        if snr < 0
            snr = 1;
        end
        snrdb = pow2db(snr);

        if snr < 1
            Sigma = single(1e-4);
        else
            Sigma = single(4e-5*snrdb - 5e-4);
            if Sigma < 6e-5
                Sigma = single(6e-5);
            end
        end

        %%

        frame_number = length(Preamble_initial_idx);

        %% CFO Estimation and Compensation
        % cfo_fine = cfo_fine_estimation(packet_data,Preamble_initial_idx);
        % cfo_temp = -1j*2*pi*(mean(cfo_fine(:)))*(0:length(packet_data)-1)/1e7;
        % rx_CFO_fine = packet_data.*exp(cfo_temp);
        % cfo_mean = mean(rx_CFO_fine);
        % packet_data_cfo_com = rx_CFO_fine - mean(rx_CFO_fine);
        packet_data_cfo_com = packet_data;

        if profile_flag
            profile on
        end
        ber_f_cur_fr = zeros(1,frame_number);
        snr_f_cur_fr = zeros(1,frame_number);

        dt_inter = data_sym_len / bw_15dB;
        dt_intra = 200 / bw_15dB;
        
        for tt = 1:frame_number
            frame_cnt = frame_cnt+1;

            % Data Extraction, minus 1 by testing
            syncOffset = -1;
            packet_data_cfo_com_ex= packet_data_cfo_com(initial_idx(tt)+cp_size+syncOffset: initial_idx(tt)+syncOffset+data_sym_len-1-preamble_size);
            rxTimePilotPow = sum(abs(reshape(packet_data_cfo_com_ex(pilotPara.rxPilotIndices),[],N)),2);
            [~,mainPilotPowInd] = max(rxTimePilotPow);
            if mainPilotPowInd ~= ceil(numel(rxTimePilotPow)/2)
                syncOffset = syncOffset + mainPilotPowInd - ceil(numel(rxTimePilotPow)/2);
                packet_data_cfo_com_ex= packet_data_cfo_com(initial_idx(tt)+cp_size+syncOffset: initial_idx(tt)+syncOffset+data_sym_len-1-preamble_size);
            end
            sample_data = reshape(packet_data_cfo_com_ex, [], N);

            % OTFS Demodulation
            rxDDSymbols = fft(sample_data,N,2)./sqrt(N);
            rxDataSymbols = rxDDSymbols(pilotPara.rxDataIndices);
            rxPilotSymbols = rxDDSymbols(pilotPara.rxPilotIndices);

            % DD grid channel estimation
            [H_DATA_CE,Pow_N] = CE_Basic3_fast(rxPilotSymbols,M,N,pilotPara);
            rxMainPath = rxDDSymbols(100,:);
            pilotSymsTime = [0.6347716 + 0.000000i,0.000000 + 0.6347716i,-0.6347716 + 0.000000i,0.000000 - 0.6347716i];
            rxMainPath = rxMainPath./pilotSymsTime;
            pow_n_all(frame_cnt) = Pow_N;
            pow_s_all(frame_cnt) = mean(abs(rxDDSymbols(pilotPara.dataIndices)).^2,'all') - Pow_N;
            snr_base(frame_cnt) = pow2db(pow_s_all(frame_cnt)/Pow_N);
            snr_f_cur_fr(tt) = snr_base(frame_cnt);

            % Estimate Doppler value
            frameDoppEst = mod(frame_cnt, numDoppEstFrame);
            if frameDoppEst ~= 0
                rxMainPath_f(:,frameDoppEst) = rxMainPath(:);
            else
                indxDoppler = indxDoppler + 1;
                rxMainPath_f(:,end) = rxMainPath(:);
                % mainDopplerEst(indxDoppler) = directDoppEst3(rxMainPath_f,dt_intra,dt_inter);
                mainDopplerEst(indxDoppler) = directDoppEst4(rxMainPath_f,offfset,dt_intra,dt_inter);
                 % doppTemp = espritDoppEst(rxMainPath_f,dt_inter);
                 % mainDopplerEst(indxDoppler) = doppTemp(1);
            end

            % Equalization and decode
            x_hat = Detector_LMMSE_sparse(reshape(double(rxDataSymbols),[],1), H_DATA_CE, double(Pow_N));
            codeBits_hat = lteSymbolDemodulate(x_hat,'QPSK','Hard');

            decodedBinary = num2str(codeBits_hat);
            decodedBinary(decodedBinary == ' ') = '';

            decodedBinary = reshape(decodedBinary, 8, []);
            decodedBinary = bin2dec(decodedBinary')';
            decodedMessage = char(decodedBinary);
            decodedMessage2 = [decodedMessage, '  '];
            decodedMessage = reshape(decodedMessage2, [59, 3])';

            % BER calculation
            ber_f_cur_fr(tt) = sum(codeBits(1:numel(codeBits_hat)) ~= codeBits_hat)/length(codeBits);
            ber_f(frame_cnt) = ber_f_cur_fr(tt);
        end

        snr_base_avg = mean(snr_base);
        pow_s_avg = mean(pow_s_all);
        pow_n_avg = mean(pow_n_all);

        if profile_flag
            profile viewer
        end

        % plot figures
        av_ber=mean(ber_f);
        subplot(3,1,2)
        b=bar(ber_f,'r');
        snrdbav=snrdba1/(frame_cnt/frame_number);
        title({['Average SNR=',num2str(snr_base_avg),' dB','    Current SNR=',num2str(mean(snr_f_cur_fr)),' dB'] ...
            ['Average BER=',num2str(av_ber), '     Current BER=',num2str(mean(ber_f_cur_fr))]})
        ylim([0 1])
        ylabel('BER')
        xlabel('Frame Numbers')
        grid on

        subplot(3,1,3);
        if switch_cnt == 0
            text(0.5, 1, decodedMessage, 'FontSize', 16, 'FontWeight', 'normal','Interpreter', 'none', 'VerticalAlignment', 'baseline', 'HorizontalAlignment', 'center', 'Color', "#D95319",'FontAngle','normal','Position',[0.5 0.8 0]);
            switch_cnt = 1;
        else
            text(0.5, 1, decodedMessage, 'FontSize', 16, 'FontWeight', 'normal', 'Interpreter', 'none', 'VerticalAlignment', 'baseline', 'HorizontalAlignment', 'center', 'Color', 'b','FontAngle','normal','Position',[0.5 0.8 0]);
            switch_cnt = 0;
        end
        set(gca,'xtick',[],'ytick',[])
        title(['Doppler = ',num2str(mainDopplerEst(end)),'Hz, Bw = ',num2str(bw_15dB),'Hz' ])

        % subplot(4,1,4);
        % mainDopplerEstAdd = repmat(mainDopplerEst,numDoppEstFrame,1);
        % mainDopplerEst1 = [mainDopplerEst1,reshape(mainDopplerEstAdd,1,[])];
        % plot(mainDopplerEst1,'k','LineWidth',3)
        % % ylim([-1000,1000]);
        % ylabel('Doppler (Hz)')
        % xlabel('Frame Numbers')
        % title(['Doppler = ',num2str(mainDopplerEst(end)),'Hz'])
        drawnow;
        time1 = toc;
        dataBitRate = frame_number * 1400 / time1
        timeSymbolRate = frame_number * 976 / time1
    catch ME
        %%error
        disp(ME.message);
        disp('Stack trace:');
        for i = 1:length(ME.stack)
            disp(['Function: ', ME.stack(i).name]);
            disp(['Line: ', num2str(ME.stack(i).line)]);
            disp(['File: ', ME.stack(i).file]);
        end
        disp(Preamble_initial_idx);
    end
end

% end