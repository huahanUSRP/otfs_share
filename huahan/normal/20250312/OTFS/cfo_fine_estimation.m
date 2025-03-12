function [cfo] = cfo_fine_estimation(rx,pointer);

cfo = zeros(1,length(pointer));

for i = 1 :length(pointer)
    yt = rx(pointer(i)+32:pointer(i)+32+127);  %%??
    ytDelayBuffer = zeros(1,64);
    op = zeros(size(yt));
    for ii = 1:length(yt)
        op(ii) = conj(yt(ii))*ytDelayBuffer(end);
        
        % shifting samples in the delay buffer
        ytDelayBuffer(2:end) = ytDelayBuffer(1:end-1);
        ytDelayBuffer(1) = yt(ii);
    end
    fdeltaEstimatekHz = -1*angle(op)/(2*pi*6.4*1e-6);    
    cfo(i) = mean(fdeltaEstimatekHz(65:end));
    0;
end