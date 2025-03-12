function x_est = Detector_LMMSE_sparse(rxSym, H, sigma2)

sigLength = size(H,2);
H = sparse(H);
H1 = H'*H + sigma2*speye(sigLength);
b = H'*rxSym;
x_est = H1\b;

end