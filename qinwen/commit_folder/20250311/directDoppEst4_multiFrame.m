function doppEstInter = directDoppEst4_multiFrame(h_est,offfset,dt_intra,dt_frame,dt_block)

[N,nFramePerBlock,nBlock] = size(h_est);

% precompensate the cfo values
if offfset ~= 0
    phase_cfo_1d = exp(-1j*2*pi*offfset*dt_intra*(0:N-1));
    phase_cfo_2d = exp(-1j*2*pi*offfset*dt_frame*(0:nFramePerBlock-1));
    phase_cfo_3d = exp(-1j*2*pi*offfset*dt_block*(0:nBlock-1));
    phase_cfo_1d = phase_cfo_1d(:);
    phase_cfo_2d = phase_cfo_2d(:);
    phase_cfo_3d = phase_cfo_3d(:);
    phase_cfo = phase_cfo_1d.*reshape(phase_cfo_2d*phase_cfo_3d.', 1, nFramePerBlock, nBlock);
    h_est = h_est.*phase_cfo;
end

% direct estimate the Doppler of the main path
phase_est1 = angle(h_est);
phase_est1 = unwrap(phase_est1,[],1);
dopp_est1 = diff(phase_est1,1,1)./(2*pi*dt_intra);
doppEstIntra = dopp_est1(:);
doppEstIntra = mean(doppEstIntra(~isoutlier(doppEstIntra)));

% precompensate the phase using the estimated doppler to solve the
% estimation range issue
if nFramePerBlock > 1
    % if contains multiple frame, use frame-wise phase change to estimate
    % Doppler
    phase_comp = exp(-1j*2*pi*doppEstIntra*dt_frame*(0:nFramePerBlock-1));
    h_comp = h_est.*reshape(phase_comp,1,[],1);
    phase_est2 = angle(h_comp);
    phase_est2 = unwrap(phase_est2,[],2);
    dopp_est2 = diff(phase_est2,1,2)./(2*pi*dt_frame);
    doppEstInter = reshape(dopp_est2 + doppEstIntra,[],1);
    doppEstInter = mean(doppEstInter(~isoutlier(doppEstInter)));
else
    % if one block only contain one frame, then use block-wise phase change
    % for estimation
    phase_comp = exp(-1j*2*pi*doppEstIntra*dt_block*(0:nBlock-1));
    h_comp = h_est.*reshape(phase_comp,1,1,[]);
    phase_est3 = angle(h_comp);
    phase_est3 = unwrap(phase_est3,[],3);
    dopp_est3 = diff(phase_est3,1,3)./(2*pi*dt_block);
    doppEstInter = reshape(dopp_est3 + doppEstIntra,[],1);
    doppEstInter = mean(doppEstInter(~isoutlier(doppEstInter)));
end