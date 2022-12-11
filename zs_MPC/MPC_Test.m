%����
clear;
close all;
clc;

%��һ�� ����״̬�ռ����
%����״̬����A n*n����
A=[1 0.1;-1 2];
n=size(A,1);%��ȡ����
B=[0.2 1;0.5 2];
p=size(B,2);
C=eye(2);


%���ھ��� ��������
%����Q���� n*n����
Q=[80 0;0 100];
%����F���� n*n����
F=[1 0;0 100];
%����R1���� p*p����
R1=[2 0;0 0.1];


%����R���� ϵͳ����
R=[10;20];

%����step����
k_steps=100;

%�������X_k n*k����
X_k = zeros(n,k_steps);
%��ʼ״̬����ֵ n*1����
X_k(:,1)=[20;-20];

%�����������U_k p*k����
U_k = zeros(p,k_steps);

%����Ԥ��ռ� N=5
N = 5;

%Call MPC_Matrices(A,B,Q,R1,F,N);
[E,H,C1,Q1] = MPC_Matrices(A,B,C,Q,R1,R,F,N);
%����ÿһ����״̬������ֵ
for k = 1 : k_steps
    %���U_k��:,k��
    U_k(:,k)=Prediction(X_k(:,k),E,H,C1,Q1,N,p);
 
    %�����k+1��ʱ��״̬������ֵ
    X_k(:,k+1) = (A*X_k(:,k)+B*U_k(:,k));
end

subplot(2,1,1);
hold;
for i=1:size(X_k,1)
    plot(X_k(i,:),"linewidth",1);
    
end
legend("x1","x2");

hold off;
grid on;
subplot(2,1,2);
hold;
for i=1:size(U_k,1)
    plot(U_k(i,:),"linewidth",1);
end
legend("u1","u2");
grid on;











