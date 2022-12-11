%清屏
clear;
close all;
clc;

%第一步 定义状态空间矩阵
%定义状态矩阵，A n*n矩阵
A=[1 0.1;-1 2];
n=size(A,1);%获取行数
B=[0.2 1;0.5 2];
p=size(B,2);
C=eye(2);


%调节矩阵 三个矩阵
%定义Q矩阵 n*n矩阵
Q=[80 0;0 100];
%定义F矩阵 n*n矩阵
F=[1 0;0 100];
%定义R1矩阵 p*p矩阵
R1=[2 0;0 0.1];


%定义R矩阵 系统给定
R=[10;20];

%定义step数量
k_steps=100;

%定义矩阵X_k n*k矩阵
X_k = zeros(n,k_steps);
%初始状态变量值 n*1向量
X_k(:,1)=[20;-20];

%定义输入矩阵U_k p*k矩阵
U_k = zeros(p,k_steps);

%定义预测空间 N=5
N = 5;

%Call MPC_Matrices(A,B,Q,R1,F,N);
[E,H,C1,Q1] = MPC_Matrices(A,B,C,Q,R1,R,F,N);
%计算每一步的状态变量的值
for k = 1 : k_steps
    %求得U_k（:,k）
    U_k(:,k)=Prediction(X_k(:,k),E,H,C1,Q1,N,p);
 
    %计算第k+1步时的状态变量的值
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











