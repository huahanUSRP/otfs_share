function OTFS_Tx_RealTimeBWInput
    fig = figure('Name', 'RealTimeBandwidth', 'NumberTitle', 'off');
    fig.Position = [300,300,1200,600];
    usrpSymbolRate_kHz = 4e3;
    
    uicontrol('Style', 'text',...
             'Position', [20 100 120 20],...
             'String', 'Bandwidth (MHz):');
    hEdit = uicontrol('Style', 'edit',...
                     'Position', [20 70 120 30],...
                     'String', '1',...
                     'Callback', @updateTx1);
    hAxes1 = axes('Position', [0.6 0.3 0.35 0.6]);
    hAxes2 = axes('Position', [0.15 0.3 0.35 0.6]);
    
    % initialize parameters
    defaultBW = 1;
    [t, y, l, y2] = Tx1(defaultBW);
    hPlot = plot(hAxes1, t, y);
    set(hAxes1, 'XGrid', 'on', 'YGrid', 'on');
    xlabel(hAxes1,'Frequency/MHz'); ylabel(hAxes1,'Magnitude');
    title(hAxes1,['Frequency domain response, ', num2str(defaultBW), ' MHz'])

    t2 = 0:1/usrpSymbolRate_kHz:(l-1)/usrpSymbolRate_kHz;
    hPlot2 = plot(hAxes2, t2, y2);
    title(hAxes2,'Time domain symbols')
    xlabel(hAxes2,'Time/ms'); ylabel(hAxes2,'Magnitude');

    % Callback function
    function updateTx1(~, ~)
        try
            newBW = str2double(get(hEdit, 'String'));
            if isnan(newBW) || newBW <= 0
                error('Invalid input!');
            end
            [newT, newY, newL, newY2] = Tx1(newBW);
            newt2 = 0:1/usrpSymbolRate_kHz:(newL-1)/usrpSymbolRate_kHz;
            
            set(hPlot, 'XData', newT, 'YData', newY);
            set(hPlot2, 'XData', newt2, 'YData', newY2);
            title(hAxes1,['Frequency domain response, ', num2str(newBW), ' MHz']);
            
        catch
            errordlg('Please input a positive number！', 'input error');
            set(hEdit, 'String', '1');
            [t, y, l, y2] = Tx1(defaultBW); 
            t2 = 0:1/usrpSymbolRate_kHz:(l-1)/usrpSymbolRate_kHz;
            set(hPlot, 'XData', t, 'YData', y);
            set(hPlot2, 'XData', t2, 'YData', y2);
        end
        drawnow;
    end
end

function [f, h, l, h2] = Tx1(BW)
    %packet_data = OTFS_USRP_Tx_text_FUNC(1e6*BW);
    packet_data = OTFS_USRP_Tx_text_FUNC_demo_dp(1e6*BW);

    [pxx, f] = pwelch(packet_data,4e2,[],[],4e6);
    f = f - mean(f);
    pxx = fftshift(pxx);
    h = pow2db(pxx);
    l = min(numel(packet_data),4e4);
    h2 = abs(packet_data(1:l));
end