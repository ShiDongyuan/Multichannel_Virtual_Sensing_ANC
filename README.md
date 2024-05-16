# Multichannel Virtual Sensing Active Noise Control
## Introduction
The multichannel virtual sensing active noise control (MVANC) methodology is an advanced approach that may provide a wide area of silence at specific virtual positions that are distant from the physical error microphones. Currently, there is a scarcity of open-source programs available for the MVANC algorithm. This work presents a MATLAB code for the MVANC approach, utilizing the multichannel filtered-x least mean square (MCFxLMS) algorithm. The code is designed to be applicable to systems with any number of channels.

## Code Explanation
### Key MATLAB Files
- [`CreatReferenceSignal.m`](#function-creatreferencesignal): This code is utilized to generates the filtered reference signals and disturbances.
- [`MultichannelFxLMS.m`](#function-multichannelfxlms): The code of the multichannel filterd reference least mean sqaure (McFxLMS) algorithm.
- [`AuxiliaryLMS.m`](#function-auxiliarylms): The code is used to obtain the auxiliary filters.
- [`ContrFxLMS.m`]: The code of the control stage of the virtual sensing ANC. 
- [`VirtualSensing_test.m`]: The main testing program of the vritual ANC codes. 

## Function: CreatReferenceSignal
The MVANC technique utilizes the FxLMS algorithm to adaptively update the control filter coefficients. Therefore, <font color=blue>`CreatReferenceSignal.m`</font> generates the reference signals filtered by the physical secondary path and the virtual secondary path respectively. In addition, it produces disturbances at the location of the physical microphone and virtual microphone respectively. These signals will later be used in other functions.

```matlab
    function [Dv,Dp,Fx_v,Fx_p] = CreatReferenceSignal(Pv,Pp,Sv,Sp,PriNoise,N,L,K,M,J)
%% -----------------------------------------------------------
% Inputs:
% Pv is the virtual primary path. 
% Pp is the physical primary path. 
% Sv is the virtual secondary path.
% Sp is the physical secondary path.
% PriNoise is the primary noise. 
% N is the number of simulation cycles.
% L is the length of control filter.
% K is the number of secondary sources.
% M is the number of virtual microphones.
% J is the number of physical microphones.
% Outputs:
% Dv is the disturbance at virtual microphone.
% Dp is the disturbance at physical microphone.
% Fx_v is the reference signal filtered by virtual secondary path.
% Fx_p is the reference signal filtered by physical secondary path.
%% -----------------------------------------------------------
Dp = zeros(N,J);
for i=1:J
    Dp(:,i) = filter(Pp(:,i),1,PriNoise);
end
Dv = zeros(N,M);
for i=1:M
    Dv(:,i) = filter(Pv(:,i),1,PriNoise);
end
Fx_p = zeros(N+L-1,J,K);
for i = 1:K 
    for j = 1:J
        Fx_p(:,j,i) = [zeros(L-1,1);filter(Sp(:,j,i),1,PriNoise)];
    end
end
Fx_v = zeros(N+L-1,M,K); 
for i = 1:K 
    for j = 1:M
        Fx_v(:,j,i) = [zeros(L-1,1);filter(Sv(:,j,i),1,PriNoise)]; 
    end
end
end
```

In this code snippet, $Dv$ is used to store the disturbances at virtual microphones and has a dimension of $N$ by $M$, while $Dp$ is used to store the disturbances at physical microphones and has a dimension of $N$ by $J$. In addition, $Fx\\_v$ is used to store the reference signals filtered by virtual secondary paths and has a dimension of $N$ by $M$ by $K$, while $Fx\\_p$ is used to store the reference signals filtered by virtual secondary paths and has a dimension of $N$ by $J$ by $K$. $N$, $M$, $J$ and $K$ denote the simulation cycle number, the number of virtual error microphones, the number of physical error microphones and the number of secondary sources respectively.

## Function: MultichannelFxLMS
In the tuning stage of the MVANC technique, the optimal control filters are first trained by `MultichannelFxLMS.m` using the FxLMS algorithm.

```matlab
function [W,Er]= MultichannelFxLMS(L,K,M,N,Fx_v,Dv,StepSize)
%% -----------------------------------------------------------
% Inputs:
% L is the length of control filter.
% K is the number of secondary sources.
% M is the number of virtual microphones. 
% N is the number of simulation cycles.
% Fx_v is the reference signal filtered by virtual secondary path. 
% Dv is the disturbance at virtual microphone.
% StepSize is the stepsize of FxLMS algorithm.
% Outputs:
% W is the control filter matrix.
% Er is the error signal at virtual microphone.
%% -----------------------------------------------------------
W  = zeros(K*L,1); 
FX = zeros(M,K*L); 
Er = zeros(M,N); 
for i = 1:N
    for j = 1:M
        for kk= 1:K
            FX(j,(kk-1)*L+1:kk*L) = Fx_v(i+L-1:-1:i,j,kk)';
        end
    end
    Ev = Dv(i,:)'- FX*W;
    SX = StepSize*Ev'*FX;
    W  = W + SX';
    Er(:,i) = Ev;
end
end
```
In this code snippet, $W$ is used to store the optimal control filters trained using the FxLMS algorithm and has a dimension of $K$ by $L$, where $L$ denotes the length of the control filter. $Er$ is used to store the error signals at virtual microphones and has a dimension of $M$ by $N$.

## Function: AuxiliaryLMS
After the control filter converges, the optimal control filters are then used by `AuxiliaryLMS.m` to train the auxiliary filters using the LMS algorithm.

```matlab
function [H,Er]=AuxiliaryLMS(L,K,J,N,Fx_p,Dp,PriNoise,W,StepSize)
%% -----------------------------------------------------------
% Inputs:
% L is the length of control filter.
% K is the number of secondary source.
% J is the number of physical microphones. 
% N is the number of simulation cycle.
% Fx_p is the reference signal filtered by physical secondary path.
% Dp is the disturbance at physical microphone.
% W is the optimal control filter matrix.
% StepSize is the stepsize of LMS algorithm.
% Outputs:
% H is the auxiliary filter matrix.
% Er is the error signal of LMS algorithm.
%% -----------------------------------------------------------
H  = zeros(J*L,1); 
SH = zeros(J*L,1); 
FX = zeros(J,K*L); 
YH = zeros(J,1); % The output signal of the auxiliary fitler.
Er = zeros(J,N)  ;
X  = [zeros(L-1,1); PriNoise];
for i = 1:N
    for j = 1:J
        for kk = 1:K 
            FX(j,(kk-1)*L+1:kk*L) = Fx_p(i+L-1:-1:i,j,kk)';
        end
        YH(j) =  X(i+L-1:-1:i)'*H((j-1)*L+1:j*L);
    end
    Ep = Dp(i,:)'-FX*W; % The error signal at physical microphone.
    Eh = Ep - YH;
    for j = 1:J
        SH((j-1)*L+1:j*L) = StepSize*Eh(j)*X(i+L-1:-1:i);
    end
    H = H + SH;
    Er(:,i) = Eh;
end
end
```

In this code snippet, $H$ is used to store the optimal auxiliary filters trained using the LMS algorithm and has a dimension of $J$ by $L$. $Er$ is used to store the error signals and has a dimension of $J$ by $N$.

## Function: ContrFxLMS

In the control stage of the MVANC technique, the optimal auxiliary filers are used by `ContrFxLMS.m` to train the new control filters using the FxLMS algorithm.

```matlab
function [WC,ErPhysic,ErVirt] = ContrFxLMS(L,K,M,J,N,Fx_p,Fx_v,Dp,Dv,PriNoise,H,StepSize) 
%% ------------------------------------------------------------------------
% Inputs:
% L is the length of control filter.
% K is the number of secondary source.
% M is the number of virtual microphones. 
% J is the number of physical microphones. 
% N is the number of simulation cycle.
% Fx_p is the reference signal filtered by physical secondary path.
% Fx_v is the reference signal filtered by virtual secondary path. 
% Dp is the disturbance at physical microphone.
% Dv is the disturbance at virtual microphone.
% H is the optimal auxiliary filter matrix.
% StepSize is the stepsize of FxLMS algorithm.
% Outputs:
% WC is the new control filter matrix.
% ErPhysic is the error signal at physical microphone.
% ErVirt is the error signal at virtual microphone.
%% ------------------------------------------------------------------------
FX = zeros(J,K*L);
FV = zeros(M,K*L);
WC = zeros(K*L,1); 
ErPhysic = zeros(J,N); 
ErVirt   = zeros(M,N); 
XH       = zeros(N,J);
for i = 1:J 
    XH(:,i) = filter(H((i-1)*L+1:i*L),1,PriNoise);
end
for i=1:N
    for j = 1:J
        for kk = 1:K 
            FX(j,(kk-1)*L+1:kk*L) = Fx_p(i+L-1:-1:i,j,kk)';
        end
    end
    for j = 1:M
        for kk = 1:K 
            FV(j,(kk-1)*L+1:kk*L) = Fx_v(i+L-1:-1:i,j,kk)';
        end
    end
    Ep      = Dp(i,:)' - FX*WC; 
    ErVirt(:,i) = Dv(i,:)' - FV*WC; 
    Eh      = Ep-XH(i,:)';
    WC      = WC + (StepSize*Eh'*FX)';
    ErPhysic(:,i) = Ep;
end
end
```

In this code snippet, $WC$ is used to store the new control filters trained using the FxLMS algorithm and has a dimension of $K$ by $L$. $ErPhysic$ is used to store the error signals at physical microphones which have a dimension of $J$ by $N$, while $ErVirt$ is used to store the error signals at virtual microphones which have a dimension of $M$ by $N$.