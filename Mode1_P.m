function [X_k_plus,Theta_k_plus,S_LF,S_LB]=Mode1_P(X_k_minus, Theta_k_minus, P, D, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag)
X_LF = mod(X_k_minus + HalfBodyDiag*cos(beta_rad - Theta_k_minus), P+D);
X_LB = mod(X_k_minus - HalfBodyDiag*cos(beta_rad + Theta_k_minus), P+D);
X_LF_absolute = X_k_minus + HalfBodyDiag*cos(beta_rad - Theta_k_minus);
X_LB_absolute = X_k_minus - HalfBodyDiag*cos(beta_rad + Theta_k_minus);

% if(X_LF_absolute >= 0)
    if X_LF >= 0 && X_LF < D/2.0
        S_LF = -X_LF;
    elseif X_LF > D/2.0 && X_LF <= D
        S_LF = D - X_LF;
    else
        S_LF = 0;
    end
% else
%     S_LF = 0; 
% end
%     
% if(X_LB_absolute >= 0)
    if X_LB >= 0 && X_LB < D/2.0
        S_LB = -X_LB;
    elseif X_LB > D/2.0 && X_LB <= D
        S_LB = D - X_LB;
    else
        S_LB = 0;
    end
% else
%     S_LB = 0;
% end

Delta_S = S_LF - S_LB;

if (abs(X_LF_absolute-X_LB_absolute+Delta_S) > 2*HalfBodyLength)  % Geometry solution
    if (abs(S_LF) >= abs(S_LB) || S_LF == 0)
        X_LB_absolute = X_LB_absolute + S_LB;
        X_LF_absolute = X_LB_absolute + 2*HalfBodyLength;
    elseif (abs(S_LF) < abs(S_LB) || S_LB == 0)
        X_LF_absolute = X_LF_absolute + S_LF;
        X_LB_absolute = X_LF_absolute - 2*HalfBodyLength;
    end
    Theta_k_plus = 0;
    X_k_plus = 0.5*(X_LF_absolute + X_LB_absolute);
else
    Theta_k_plus = sign(sin(Theta_k_minus))*acos(cos(Theta_k_minus) + Delta_S/(2*HalfBodyLength));
    X_med = 0.5*(X_LF_absolute + S_LF + X_LB_absolute + S_LB);
    X_k_plus = X_med - HalfBodyWidth * sin(Theta_k_plus);
end
end
