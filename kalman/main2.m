rng(100); % For reproducibility
T = 150;
sigma1 = 0.1;
sigma2 = 0.2;
phi = 0.6;

u1 = randn(T,1)*sigma1;
x1 = cumsum(u1);
Mdl2 = arima('AR',phi,'Variance',sigma2^2,'Constant',0);
x2 = simulate(Mdl2,T,'Y0',0);
y = x1 + x2;

figure;
plot([x1 x2 y])
legend('x_1','x_2','y','Location','Best');
ylabel('Net exports');
xlabel('Period');