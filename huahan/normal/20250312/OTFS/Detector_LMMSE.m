function x_est = Detector_LMMSE(rxSym, H, sigma2)

sigLength = size(H,2);
H1 = H'*H + sigma2*eye(sigLength);
x_est = H1\(H'*rxSym);

end