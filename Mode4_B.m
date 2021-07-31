function [X_k_minus,Theta_k_minus,X_lateral_out]=Mode4_B(X_k_plus, Theta_k_plus,X_lateral_in, StepLengthlist)
Theta_k_minus = Theta_k_plus;
X_k_minus = X_k_plus + StepLengthlist*cos(Theta_k_plus);
X_lateral_out = X_lateral_in + StepLengthlist*sin(Theta_k_plus);
end