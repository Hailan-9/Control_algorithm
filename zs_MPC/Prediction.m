function u_k=Prediction(x_k,E,H,C1,Q1,N,p)
    U_K = zeros(N*p,1);
    [U_K,fval] =quadprog(H,E*x_k-C1'*Q1);
    u_k = U_K(1:p,1); % 取第一个结果
end