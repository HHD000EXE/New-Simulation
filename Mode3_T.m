function [X_k_plus,Theta_k_plus,S_RF,S_LB]=Mode3_T(X_k_minus, Theta_k_minus, P, D, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag)
X_RF = mod(X_k_minus + HalfBodyDiag*cos(beta_rad + Theta_k_minus), P+D);
X_LB = mod(X_k_minus - HalfBodyDiag*cos(beta_rad + Theta_k_minus), P+D);
X_RF_absolute = X_k_minus + HalfBodyDiag*cos(beta_rad + Theta_k_minus);
X_LB_absolute = X_k_minus - HalfBodyDiag*cos(beta_rad + Theta_k_minus);
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

Delta_S = S_RF - S_LB;

if (abs(X_RF_absolute-X_LB_absolute+Delta_S) > 2*HalfBodyDiag)  % Geometry solution
    if (abs(S_RF) >= abs(S_LB) || S_RF == 0)
        X_LB_absolute = X_LB_absolute + S_LB;
        if(Theta_k_minus < pi/2 - beta_rad && Theta_k_minus > -pi/2 - beta_rad)
            X_RF_absolute = X_LB_absolute + 2*HalfBodyDiag;
        else
            X_RF_absolute = X_LB_absolute - 2*HalfBodyDiag;
        end
    elseif (abs(S_RF) < abs(S_LB) || S_LB == 0)
        X_RF_absolute = X_RF_absolute + S_RF;
        if(Theta_k_minus < pi/2 - beta_rad && Theta_k_minus > -pi/2 - beta_rad)
            X_LB_absolute = X_RF_absolute - 2*HalfBodyDiag;
        else
            X_LB_absolute = X_RF_absolute + 2*HalfBodyDiag;
        end
    end
    if(Theta_k_minus < pi/2 - beta_rad && Theta_k_minus > -pi/2 - beta_rad)
        Theta_k_plus = -beta_rad;
    else
        Theta_k_plus = pi*3/2 - beta_rad;
    end
    X_k_plus = 0.5*(X_RF_absolute + X_LB_absolute);
else
    Theta_1 = pi/2-beta_rad-asin(sin(pi/2-beta_rad-Theta_k_minus) + Delta_S/(2*HalfBodyDiag));
    Theta_2 = -beta_rad-pi/2+asin(sin(pi/2-beta_rad-Theta_k_minus) + Delta_S/(2*HalfBodyDiag));
    if abs(Theta_1-Theta_k_minus) > abs(Theta_2-Theta_k_minus)
        Theta_k_plus = Theta_2;
    else
        Theta_k_plus = Theta_1;
    end
    X_k_plus = 0.5*(X_RF_absolute + S_RF + X_LB_absolute + S_LB);
end
end
