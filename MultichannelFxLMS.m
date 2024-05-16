function [W,Er]= MultichannelFxLMS(LenFilter,NumSource,NumVM,N,Fx_v,DisturVirt,StepSize)
%% -----------------------------------------------------------
% LenFilter  = 512         ; % The length of the every control filter.
% NumSource is the number of the secondary source.
% N is the number of the signal.
% NumVM is the number of the microphones. 
% Fx_v:(N,NumVM,NumSource) is the filterd reference signal. 
% DisturVirt(N,NumVM) is the disturbances. 
%% -----------------------------------------------------------
% Ev = zeros(NumVM,1)  ; % Error signal on the virtual microphones.
W  = zeros(NumSource*LenFilter,1); % The control filters. 
FX = zeros(NumVM,NumSource*LenFilter); % Filtered reference matrix.
% SX = zeros(1,NumSource*LenFilter);

% StepSize  = 0.000001    ; % Step size.
Er = zeros(4,N)  ;
for i = 1:N
    for j = 1:NumVM
%         FX(j,:) = [Fx_v(i+LenFilter-1:-1:i,j,1)' Fx_v(i+LenFilter-1:-1:i,j,2)'...
%                    Fx_v(i+LenFilter-1:-1:i,j,3)' Fx_v(i+LenFilter-1:-1:i,j,4)'];
        for kk= 1:NumSource
            FX(j,(kk-1)*LenFilter+1:kk*LenFilter) = Fx_v(i+LenFilter-1:-1:i,j,kk)';
        end
    end
    Ev = DisturVirt(i,:)'- FX*W;
    SX = StepSize*Ev'*FX      ;
    W  = W + SX'       ;
    Er(:,i) = Ev       ;
end

end