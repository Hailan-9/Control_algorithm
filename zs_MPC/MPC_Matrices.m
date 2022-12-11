function [E,H,C1,Q1] = MPC_Matrices(A,B,C,Q,R1,R,F,N)
    n=size(A,1);%A是n*n矩阵 得到n也就是行数
    p=size(B,2);%B是n*p矩阵，得到p也就是列数
    M=[eye(n);zeros(N*n,n)]; %初始化M矩阵，M是(N+1)n*n
    C1=zeros((N+1)*n,N*p);%初始化C1矩阵，(N+1)n*Np
    tmp=eye(n);
    
    %更新M和C1
    for i=1:N %循环，i从1到N
        rows = i*n+(1:n);
        C1(rows,:)=[tmp*B,C1(rows-n,1:end-p)];%将C矩阵填满
        tmp = A*tmp;%每一次将tmp左乘一次A
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
    
    
    
    
    
    %计算H
    H=C1'*Q_bar*C1+R1_bar;
    E=C1'*Q_bar'*M;
    
    