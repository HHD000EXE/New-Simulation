function [X_k_plus,Theta_k_plus,S_LB,S_RB]=Mode3_B(X_k_minus, Theta_k_minus, P, D, HalfBodyWidth, HalfBodyLength, beta_rad, HalfBodyDiag)
X_LB = mod(real(X_k_minus - HalfBodyDiag*cos(beta_rad + Theta_k_minus)), P+D);
X_RB = mod(real(X_k_minus - HalfBodyDiag*cos(beta_rad - Theta_k_minus)), P+D);
X_LB_absolute = X_k_minus - HalfBodyDiag*cos(beta_rad + Theta_k_minus);
X_RB_absolute = X_k_minus - HalfBodyDiag*cos(beta_rad - Theta_k_minus);

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

Delta_S = S_LB - S_RB;

if (abs(X_LB_absolute-X_RB_absolute+Delta_S) >= 2*HalfBodyWidth)  % Geometry solution
    if ((abs(S_LB) > abs(S_RB) || S_LB == 0) && S_RB ~= 0)
        X_RB_absolute = X_RB_absolute + S_RB;
        if(Theta_k_minus > 0)
            X_LB_absolute = X_RB_absolute + 2*HalfBodyWidth;
        else
            X_LB_absolute = X_RB_absolute - 2*HalfBodyWidth;
        end
    elseif ((abs(S_LB) < abs(S_RB) || S_RB == 0) && S_LB ~= 0) 
        X_LB_absolute = X_LB_absolute + S_LB;
        if(Theta_k_minus < 0)
            X_RB_absolute = X_LB_absolute + 2*HalfBodyWidth;
        else
            X_RB_absolute = X_LB_absolute - 2*HalfBodyWidth;
        end
    end
    Theta_k_plus = pi/2*sign(Theta_k_minus);
    X_med = 0.5*(X_LB_absolute + X_RB_absolute);
    X_k_plus = X_med + HalfBodyLength * cos(Theta_k_plus);
else
    Theta_k_plus = asin(sin(Theta_k_minus) + Delta_S/(2*HalfBodyWidth));
    X_med = 0.5*(X_LB_absolute + S_LB + X_RB_absolute + S_RB);
    X_k_plus = X_med + HalfBodyLength * cos(Theta_k_plus);
end
end