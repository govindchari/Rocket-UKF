nsig = 2;
R = R_gps;
nt = length(t);
ffun = 'predict';
n = 10;

Z = gps;

r0 = [0;0;0];
v0 = [0;0;0];
q0 = [1;0;0;0];
xhatu = [r0;v0;q0];
P0=diag([0.1 0.1 0.1 0.01 0.01 0.01 0.001 0.001 0.001 0.001]);
Pu(1:n,1:n,1)=P0;

Q = diag([0.1 0.2 0.5 0.01 0.01 0.01 0 0 0 0]);

%Q = 10*diag([0.01 0.01 0.01 0.0001 0.0001 0.0001 0 0 0 0]); this works
%well



for k=1:(nt-1) 
    zkp1=Z(k+1,:)';
    w = gyro(k,:);
    a = accel(k,:);
    [xhatu(:,k+1),Pu(1:n,1:n,k+1)] = ukf(xhatu(:,k),Pu(1:n,1:n,k),Q,ffun,zkp1,R,dt,nsig,w,a,k*dt);
end
function dz = eom_ukf(z,w,a)
    v = z(4:6);
    q = z(7:10)';

    vd_quat = quatmultiply(quatmultiply((q),[0 a]),quatconj(q));

    rd = v;
    vd = vd_quat(2:4);
    qd = 0.5*quatmultiply(q,[0 w]);

    dz=[rd;vd';qd'];
end
function Xkp1=predict(Xk,w,a,dt)
[~,nsp]=size(Xk);
Xkp1 = zeros(size(Xk));

    %Predict for all sigma points
    opt = odeset('RelTol',1e-13,'AbsTol',1e-13);
    for i=1:nsp
        xinit=Xk(:,i);
        [~,x]=ode45(@(t,x) eom_ukf(x,w,a),[0 dt],xinit,opt);
        Xkp1(:,i)=x(end,:)';
    end

end
function [xEst,PxEst]=ukf(xEst,PxEst,Q,ffun,z,R,dt,nsig,w,a,t)

%Initialization
nx = length(xEst);  %number of states
nsp=2*nx+1;        %number of sigma points
ensp=ones(1,nsp); %vector of all ones 

%Generate weights
Wi=0.5/nsig^2;
W0M=(nsig^2-nx)/nsig^2;
W0C=(nsig^2-nx)/nsig^2+3-nsig^2/nx;

%vector form
WM=[W0M;ones(2*nx,1)*Wi];
WC=[W0C;ones(2*nx,1)*Wi];

% Generate Sigma Points
Psqrtm = nsig*chol(PxEst)';
xSigmaPts=[zeros(nx,1) -Psqrtm Psqrtm];
xSigmaPts = xSigmaPts + xEst*ensp;

% Propagate sigma points through nonlinear dynamics
xPredSigmaPts = feval(ffun,xSigmaPts,w,a,dt);

% Calculate Predicted State 
xPred = xPredSigmaPts*WM; 

% Calculate Predicted Covariances
exSigmaPts = xPredSigmaPts - xPred*ensp;
PxxPred = exSigmaPts*[diag(WC)]*exSigmaPts' + Q;

%%Compute Kalman Gain
H=[1 0 0 0 0 0 0 0 0 0;
   0 1 0 0 0 0 0 0 0 0
   0 0 1 0 0 0 0 0 0 0];
K = PxxPred*H'*(H*PxxPred*H'+R)^-1;

% Update Step
PxEst = [PxxPred - K*(PxxPred*H')'];
% xEst = xPred + K*(z - H*xPred);

% Only update if you have a GPS reading. Otherwise, only predict
if t*10 == round(t*10)
    xEst = xPred + K*(z - H*xPred);
else
    xEst = xPred;
end

end