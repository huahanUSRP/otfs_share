function doppEstInter = espritDoppEst(h_est,dt_inter)
[~,n1] = size(h_est);
try
    gain_est2 = ESPRIT(h_est, min(floor(n1/2),3), true, true);
catch
    gain_est2 = ESPRIT(h_est, min(floor(n1/2),3), false, true);
end
doppEstInter = angle(gain_est2)/2/pi/dt_inter;
end