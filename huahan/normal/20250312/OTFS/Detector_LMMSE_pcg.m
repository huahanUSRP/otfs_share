function x_est = Detector_LMMSE_pcg(rxSym, H, sigma2)

sigLength = size(H,2);
H = sparse(H);
H1 = H'*H + sigma2*eye(sigLength);
b = H'*rxSym;
x_est = pcg(H1,b,1e-6,100);

end