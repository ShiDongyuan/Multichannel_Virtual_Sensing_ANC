function [Dv,Dp,Fx_v,Fx_p] = CreatReferenceSignal(Pv,Pp,Sv,Sp,PriNoise,N,L,K,M,J)
%% ------------------------------------------------------------------------
% Pv is the virtual primary path. 
% Pp is the physical priamry path. 
% Sv is the secondary path of the virtual microphone.
% Pp is the secondary path of the physical microphone.
% PriNoise is the primary noise. 
%% ------------------------------------------------------------------------
Dv = zeros(N,M)             ;
for i=1:M
    Dv(:,i) = filter(Pv(:,i),1,PriNoise);
end
%
Dp = zeros(N,J)             ;
for i=1:J
    Dp(:,i) = filter(Pp(:,i),1,PriNoise);
end
%
Fx_v = zeros(N+L-1,M,K)     ;
for i = 1:K 
    for j = 1:M
        Fx_v(:,j,i) = [zeros(L-1,1);filter(Sv(:,j,i),1,PriNoise)];
    end
end
%
Fx_p = zeros(N+L-1,J,K)     ;
for i = 1:K 
    for j = 1:J
        Fx_p(:,j,i) = [zeros(L-1,1);filter(Sp(:,j,i),1,PriNoise)];
    end
end
% XH = zeros(N,J)     ;
% for i = 1:J 
%     XH(:,i) = filter(H((i-1)*L+1:i*L),1,PriNoise);
% end
end