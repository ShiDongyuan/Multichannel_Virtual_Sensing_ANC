function [WC,ErPhysic,ErVirt] = ContrFxLMS(LenFilter,NumSource,NumVM,NumPM,N,Fx_p,Fx_v,DisturPhysic,DisturVirt,PriNoise,H,StepSize) 
%% ------------------------------------------------------------------------
% NumSource is the number of the secondary sources.
% NumPM is the number of the physical mirophones.
% NumVM is the number of the virtual microphones.
% Fx_v is the filtered reference by the virtual microphones. 
% Fx_p is the filtered reference by the physical microphones. 
% DisturVirt is the disturbance of the virtual microphones. 
% DisturPhysic is the disturbance of the physical microphones.
% LenFilter is the taps of the filter. 
%% ------------------------------------------------------------------------
FX = zeros(NumPM,NumSource*LenFilter); % Clear memory of FX.
FV = zeros(NumVM,NumSource*LenFilter);
WC = zeros(NumSource*LenFilter,1)    ; % Control filter at the control stage.
ErPhysic = zeros(NumPM,N)  ;
ErVirt   = zeros(NumVM,N)  ;
XH       = zeros(N,NumPM)  ;
for i = 1:NumPM 
    XH(:,i) = filter(H((i-1)*LenFilter+1:i*LenFilter),1,PriNoise);
end
for i=1:N
    for j = 1:NumPM
%         FX(j,:) = [Fx_p(i+LenFilter-1:-1:i,j,1)' Fx_p(i+LenFilter-1:-1:i,j,2)'...
%                    Fx_p(i+LenFilter-1:-1:i,j,3)' Fx_p(i+LenFilter-1:-1:i,j,4)'];
        for kk = 1:NumSource 
            FX(j,(kk-1)*LenFilter+1:kk*LenFilter) = Fx_p(i+LenFilter-1:-1:i,j,kk)';
        end
    end
    for j = 1:NumVM
        for kk = 1:NumSource 
            FV(j,(kk-1)*LenFilter+1:kk*LenFilter) = Fx_v(i+LenFilter-1:-1:i,j,kk)';
        end
    end
    Ep      = DisturPhysic(i,:)' - FX*WC ;
    ErVirt(:,i) = DisturVirt(i,:)' - FV*WC ;
    Eh      = Ep-XH(i,:)'      ;
    WC      = WC + (StepSize*Eh'*FX)';
    ErPhysic(:,i) = Ep               ;
end

end