function [X_k_plus,Theta_k_plus,S_LF,S_RF]=Mode1_B(X_k_minus, Theta_k_minus, P, D, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag)
X_LF = mod(real(X_k_minus + HalfBodyDiag*cos(beta_rad - Theta_k_minus)), P+D);
X_RF = mod(real(X_k_minus + HalfBodyDiag*cos(beta_rad + Theta_k_minus)), P+D);
X_LF_absolute = X_k_minus + HalfBodyDiag*cos(beta_rad - Theta_k_minus);
X_RF_absolute = X_k_minus + HalfBodyDiag*cos(beta_rad + Theta_k_minus);

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

Delta_S = S_LF - S_RF;

if (abs(X_LF_absolute-X_RF_absolute+Delta_S) >= 2*HalfBodyWidth)  % Geometry solution
    if ((abs(S_LF) > abs(S_RF) || S_LF == 0) && S_RF ~= 0)
        X_RF_absolute = X_RF_absolute + S_RF;
        if(Theta_k_minus > 0)
            X_LF_absolute = X_RF_absolute + 2*HalfBodyWidth;
        else
            X_LF_absolute = X_RF_absolute - 2*HalfBodyWidth;
        end
    elseif ((abs(S_LF) < abs(S_RF) || S_RF == 0) && S_LF ~= 0)
        X_LF_absolute = X_LF_absolute + S_LF;
        if(Theta_k_minus < 0)
            X_RF_absolute = X_LF_absolute + 2*HalfBodyWidth;
        else
            X_RF_absolute = X_LF_absolute - 2*HalfBodyWidth;
        end
    end
    Theta_k_plus = pi/2*sign(Theta_k_minus);
    X_med = 0.5*(X_LF_absolute + X_RF_absolute);
    X_k_plus = X_med - HalfBodyLength * cos(Theta_k_plus);
else
    Theta_k_plus = asin(sin(Theta_k_minus) + Delta_S/(2*HalfBodyWidth));
    X_med = 0.5*(X_LF_absolute + S_LF + X_RF_absolute + S_RF);
    X_k_plus = X_med - HalfBodyLength * cos(Theta_k_plus);
end
end
