function doppEstInter = directDoppEst4(h_est,offfset,dt_intra,dt_inter)
if offfset ~= 0
    phase_comp_offset_col = exp(-1j*2*pi*offfset*dt_intra*(0:size(h_est,1)-1));
    phase_comp_offset_row = exp(-1j*2*pi*offfset*dt_inter*(0:size(h_est,2)-1));
    phase_comp_offset = reshape(phase_comp_offset_col,[],1)*reshape(phase_comp_offset_row,1,[]);
    h_est = h_est .* phase_comp_offset;
end
% direct estimate the Doppler of the main path
phase_est1 = angle(h_est);
phase_est2 = unwrap(phase_est1,[],1);
dopp_est1 = diff(phase_est2,1,1)./(2*pi*dt_intra);
doppEstIntra = dopp_est1(:);
doppEstIntra = mean(doppEstIntra(~isoutlier(doppEstIntra)));

phase_comp = exp(-1j*2*pi*doppEstIntra*dt_inter*(0:size(h_est,2)-1));
h_comp = h_est.*reshape(phase_comp,1,[]);
phase_est3 = angle(h_comp);
phase_est3 = unwrap(phase_est3,[],2);
dopp_est2 = diff(phase_est3,1,2)./(2*pi*dt_inter);
doppEstInter = reshape(dopp_est2 + doppEstIntra,[],1);
doppEstInter = mean(doppEstInter(~isoutlier(doppEstInter)));
if abs(abs(doppEstInter) - 1/dt_inter) < 0.1/dt_inter
    if doppEstInter < 0
        doppEstInter = doppEstInter + 1/dt_inter;
    else
        doppEstInter = doppEstInter - 1/dt_inter;
    end
end
end