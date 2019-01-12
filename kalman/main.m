
T = 1.5;
Ts = 0.01;
t = (0:T/Ts) * Ts;

v0 = 10;
vt = 7;
alpha = pi * (3/8);
g = 9.82;

x = (v0*vt*cos(alpha) / g) * (1 - exp(-g*t/vt));
z = (vt / g) * (v0*sin(alpha) + vt) * (1- exp(-g*t/vt)) - vt*t;

xm = [];
zm = [];
r = 0.05;

A = [1  Ts  0  0  ;
     0  NaN 0  0  ;
     0  0   1  Ts ;
     0  0   0  NaN];
B = [0; 0; 0; -9.82*Ts];
C = [1 0 0 0;
     0 0 1 0];
D = r * ones(2,1);
Mean0 = [0; 0; 0; 0];
Cov0 = eye(4);
StateType = [0; 0; 0; 0];
mdl = ssm(A,B,C,D,'Mean0',Mean0,'Cov0',Cov0,'StateType',StateType);

disp(mdl);

%%

Y = horzcat(x', z');
Y = Y(1:80,:);
params0 = 0.9 * ones(2, 1);
lb = 0.8 * ones(2, 1);
ub = ones(2, 1);
est = estimate(mdl, Y, params0, 'lb', lb, 'ub', ub);

%%
F = 500;
Yf = forecast(est, F, Y);
hold off;
plot(Y(:,1), Y(:,2));
hold on;
plot(Yf(:,1), Yf(:,2));

xlim([0 5]);
ylim([0 5]);

%%

for i = 1:length(t)    
    xm = [xm, x(i) + r * rand(1)];
    zm = [zm, z(i) + r * rand(1)];
    
    hold off;
    plot(x, z);
    xlim([0 3]);
    ylim([0 3]);

    hold on;
    scatter(xm, zm);
    
    pause(0.1);
end