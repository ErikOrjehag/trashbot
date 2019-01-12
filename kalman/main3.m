
T = 1.1;
Ts = 0.01;
t = (0:T/Ts) * Ts;

v0 = 6;
vt = 1;
alpha = pi * (3/10) + (rand()-0.5) * pi/10;
g = 9.82;
d = -g/vt;
x0 = 0;
z0 = 1.5;

s0 = [x0; v0*cos(alpha); z0; v0*sin(alpha)];

x = x0 + (v0*vt*cos(alpha) / g) * (1 - exp(-g*t/vt));
z = z0 + (vt / g) * (v0*sin(alpha) + vt) * (1- exp(-g*t/vt)) - vt*t;

r = 0.1;
d = -1.4;
A = [0  1  0  0  ;
     0  d  0  0 ;
     0  0   0  1;
     0  0   0  d];
B = [0; 0; 0; -g];
C = [1 0 0 0;
     0 0 1 0];
D = zeros(size(C,1), size(B,2));

Vd = diag([2 6 2 6]);
Vn = r*eye(2);

CF = [zeros(2,4) Vn];
BF = [B Vd 0*B];

sysC = ss(A,BF,C,CF);
sysFullOutput = ss(A,BF,eye(4),zeros(4,size(BF,2)));

Kf = lqe(A,Vd,C,Vd,Vn);
Akf = A-Kf*C;
Bkf = [B Kf];
sysKf = ss(Akf, Bkf, eye(length(A)), 0*Bkf);

uDIST = diag(Vd).*randn(4,size(t,2));
uNOISE = Vn(1,1)*randn(size(t));
u = ones(size(t));
uAUG = [u; 0*uDIST; uNOISE];

Ym = lsim(sysC, uAUG, t, s0);
Yt = lsim(sysFullOutput, uAUG, t, s0);
l = 5;
mul = 1.03;
se = [Ym(1,1); 
    mul*(Ym(l,1)-Ym(1,1))/((l-1)*Ts); 
    Ym(1,2); 
    mul*(Ym(l,2)-Ym(1,2))/((l-1)*Ts)
    ];

for i = l:length(t)-1
    
    u1 = [u; Ym(:,1)'; Ym(:,2)'];
    u1 = u1(:,1:i);
    t1 = t(:,1:i);
    
    Ykf = lsim(sysKf, u1, t1, se);
    
    sn = Ykf(end,:)';
    un = uAUG(:,i:end);
    tn = t(:,1:end-i+1);
    Yp = lsim(sysFullOutput, un, tn, sn);
    
    hold off;
    %plot(x, z);
    plot(Yt(:,1),Yt(:,3));
    xlim([0 3]);
    ylim([0 3]);
    hold on;
    plot(u1(2,:),u1(3,:));
    plot(Ykf(:,1),Ykf(:,3));
    plot(Yp(:,1),Yp(:,3));
    pause(0.01);
end