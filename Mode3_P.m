function [X_k_plus,Theta_k_plus,S_RF,S_RB]=Mode3_P(X_k_minus, Theta_k_minus, P, D, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag)
X_RF = mod(X_k_minus + HalfBodyDiag*cos(beta_rad + Theta_k_minus), P+D);
X_RB = mod(X_k_minus - HalfBodyDiag*cos(beta_rad - Theta_k_minus), P+D);
X_RF_absolute = X_k_minus + HalfBodyDiag*cos(beta_rad + Theta_k_minus);
X_RB_absolute = X_k_minus - HalfBodyDiag*cos(beta_rad - Theta_k_minus);

% if(X_RF_absolute >= 0)
    if X_RF >= 0 && X_RF < D/2.0
        S_RF = -X_RF;
    elseif X_RF > D/2.0 && X_RF <= D
        S_RF = D - X_RF;
    else
        S_RF = 0;
    end
% else
%     S_RF = 0; 
% end
%     
% if(X_RB_absolute >= 0)
    if X_RB >= 0 && X_RB < D/2.0
        S_RB = -X_RB;
    elseif X_RB > D/2.0 && X_RB <= D
        S_RB = D - X_RB;
    else
        S_RB = 0;
    end
% else
%     S_RB = 0;
% end

Delta_S = S_RF - S_RB;

if (abs(X_RF_absolute-X_RB_absolute+Delta_S) > 2*HalfBodyLength)  % Geometry solution
    if (abs(S_RF) >= abs(S_RB) || S_RF == 0)
        X_RB_absolute = X_RB_absolute + S_RB;
        X_RF_absolute = X_RB_absolute + 2*HalfBodyLength;
    elseif (abs(S_RF) < abs(S_RB) || S_RB == 0)
        X_RF_absolute = X_RF_absolute + S_RF;
        X_RB_absolute = X_RF_absolute - 2*HalfBodyLength;
    end
    Theta_k_plus = 0;
    X_k_plus = 0.5*(X_RF_absolute + X_RB_absolute);
else
%     Theta_k_plus = pi/2 - asin(sin(pi/2 - Theta_k_minus) + Delta_S/(2*HalfBodyLength));
    Theta_k_plus = sign(sin(Theta_k_minus))*acos(cos(Theta_k_minus) + Delta_S/(2*HalfBodyLength));
    X_med = 0.5*(X_RF_absolute + S_RF + X_RB_absolute + S_RB);
    X_k_plus = X_med + HalfBodyWidth * sin(Theta_k_plus);
end
end
