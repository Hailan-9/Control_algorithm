function [E,H,C1,Q1] = MPC_Matrices(A,B,C,Q,R1,R,F,N)
    n=size(A,1);%A��n*n���� �õ�nҲ��������
    p=size(B,2);%B��n*p���󣬵õ�pҲ��������
    M=[eye(n);zeros(N*n,n)]; %��ʼ��M����M��(N+1)n*n
    C1=zeros((N+1)*n,N*p);%��ʼ��C1����(N+1)n*Np
    tmp=eye(n);
    
    %����M��C1
    for i=1:N %ѭ����i��1��N
        rows = i*n+(1:n);
        C1(rows,:)=[tmp*B,C1(rows-n,1:end-p)];%��C��������
        tmp = A*tmp;%ÿһ�ν�tmp���һ��A
        M(rows,:)=tmp;
    end
    
    Q_bar=kron(eye(N),C'*Q*C);
    Q_bar=blkdiag(Q_bar,C'*F*C);
    R1_bar = kron(eye(N),R1);
    
    Q1=zeros((N+1)*n,1);
    for i=1:N
        rows=(i-1)*n+(1:n);
        Q1(rows,:)=C'*Q*R;
    end
    rows=N*n+(1:n);
    Q1(rows,:)=C'*F*R;
    
    
    
    
    
    %����H
    H=C1'*Q_bar*C1+R1_bar;
    E=C1'*Q_bar'*M;
    
    