function [X_k_plus,Theta_k_plus,S_LF,S_RB]=Mode1_T(X_k_minus, Theta_k_minus, P, D, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag)
X_LF = mod(X_k_minus + HalfBodyDiag*cos(beta_rad - Theta_k_minus), P+D);
X_RB = mod(X_k_minus - HalfBodyDiag*cos(beta_rad - Theta_k_minus), P+D);
X_LF_absolute = X_k_minus + HalfBodyDiag*cos(beta_rad - Theta_k_minus);
X_RB_absolute = X_k_minus - HalfBodyDiag*cos(beta_rad - Theta_k_minus);
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


Delta_S = S_LF - S_RB;

if (abs(X_LF_absolute-X_RB_absolute+Delta_S) > 2*HalfBodyDiag)  % Geometry solution
    if (abs(S_LF) >= abs(S_RB) || S_LF == 0)
        X_RB_absolute = X_RB_absolute + S_RB;
        if(Theta_k_minus < pi/2 + beta_rad && Theta_k_minus > -pi/2 + beta_rad)
            X_LF_absolute = X_RB_absolute + 2*HalfBodyDiag;
        else
            X_LF_absolute = X_RB_absolute - 2*HalfBodyDiag;
        end
    elseif (abs(S_LF) < abs(S_RB) || S_RB == 0)
        X_LF_absolute = X_LF_absolute + S_LF;
        if(Theta_k_minus < pi/2 + beta_rad && Theta_k_minus > -pi/2 + beta_rad)
            X_RB_absolute = X_LF_absolute - 2*HalfBodyDiag;
        else
            X_RB_absolute = X_LF_absolute + 2*HalfBodyDiag;
        end
    end
    if(Theta_k_minus < pi/2 + beta_rad && Theta_k_minus > -pi/2 + beta_rad)
        Theta_k_plus = beta_rad;
    else
        Theta_k_plus = pi*3/2 - beta_rad;
    end
    X_k_plus = 0.5*(X_LF_absolute + X_RB_absolute);
else
    Theta_1 = pi/2+beta_rad-asin(sin(pi/2+beta_rad-Theta_k_minus) + Delta_S/(2*HalfBodyDiag));
    Theta_2 = beta_rad-pi/2+asin(sin(pi/2+beta_rad-Theta_k_minus) + Delta_S/(2*HalfBodyDiag));
    if abs(Theta_1-Theta_k_minus) > abs(Theta_2-Theta_k_minus)
        Theta_k_plus = Theta_2;
    else
        Theta_k_plus = Theta_1;
    end
    X_k_plus = 0.5*(X_LF_absolute + S_LF + X_RB_absolute + S_RB);
end
end
