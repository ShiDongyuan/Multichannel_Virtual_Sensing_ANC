function [H,Er]=AuxiliaryLMS(LenFilter,NumSource,NumPM,N,Fx_p,DisturPhysic,PriNoise,W,StepSize)
%% ------------------------------------------------------------------------
% LenFilter is the length of the control filter.
% NumPM is the number of the physical microphones.
% Fx_P: (N,NumPM,NumSource) is the filtered reference.
%% ------------------------------------------------------------------------
H  = zeros(NumPM*LenFilter,1); % Auxilary filters.
SH = zeros(NumPM*LenFilter,1); 
FX = zeros(NumPM,NumSource*LenFilter); % Filtered reference matrix.
YH = zeros(NumPM,1)  ; % The output signal of the auxiliary fitler.
Er = zeros(NumPM,N)  ;
X  = [zeros(LenFilter-1,1); PriNoise];
for i = 1:N
    for j = 1:NumPM
        for kk = 1:NumSource 
            FX(j,(kk-1)*LenFilter+1:kk*LenFilter) = Fx_p(i+LenFilter-1:-1:i,j,kk)';
        end
        YH(j) =  X(i+LenFilter-1:-1:i)'*H((j-1)*LenFilter+1:j*LenFilter);
    end
    Ep = DisturPhysic(i,:)'-FX*W ;
    Eh = Ep - YH ;
    for j = 1:NumPM
        SH((j-1)*LenFilter+1:j*LenFilter) = StepSize*Eh(j)*X(i+LenFilter-1:-1:i);
    end
    H       = H + SH;
    Er(:,i) = Eh    ;
end
end