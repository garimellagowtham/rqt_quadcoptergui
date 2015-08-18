function [ MLEval ] = Transferfunction( theta,omega,Gi,select)
% Evaluates MLE function on Candidate Transfer functions
%   The type of transfer function is selected using the select value.
% The transfer function will be evaluated at omega frequencies:
switch select
    case 1
        Geval = G1(theta,omega);
    case 2
        Geval = G2(theta,omega);
    case 3
        Geval = G3(theta,omega);
    case 4
        Geval = G4(theta,omega);
    case 5
        Geval = G5(theta,omega);
    case 6
        Geval = G6(theta,omega);
    case 7
        Geval = G7(theta,omega);
    case 8
        Geval = G8(theta,omega);
    case 9
        Geval = G9(theta,omega);
end
ph1 = atan2(imag(Geval),real(Geval));
ph2 = atan2(imag(Gi),real(Gi));
 MLEval = sum((log(abs(Gi)) - log(abs(Geval))).^2 + (ph1-ph2).^2);
end
function G = G1(theta,omega)
    omega1 = 1i*omega;
    G = theta(1)*(omega1)./(omega1+theta(2));
end
function G = G2(theta,omega)
    omega1 = 1i*omega;
    G = (exp(-omega1.*theta(3)).*theta(1)).*((omega1)./(omega1+theta(2)));
end
function G = G3(theta,omega)
    omega1 = 1i*omega;
    G = (exp(-omega1.*theta(4)).*theta(1)).*((omega1+theta(2))./(omega1+theta(3)));
end
function G = G4(theta,omega)
    omega1 = 1i*omega;
    G = (exp(-omega1.*theta(5)).*theta(1)).*((omega1.^2+theta(2).*omega1)./(omega1.^2+theta(3).*omega1+theta(4)));
end
function G = G5(theta,omega)
    omega1 = 1i*omega;
    G = (exp(-omega1.*theta(6)).*theta(1)).*((omega1.^2+theta(2).*omega1+theta(3))./(omega1.^2+theta(4).*omega1+theta(5)));
end
function G = G6(theta,omega)
    omega1 = 1i*omega;
    G = (exp(-omega1.*theta(7)).*theta(1)).*((omega1.^3+theta(2).*omega1.^2+theta(3).*omega1)./(omega1.^3+theta(4).*omega1.^2+theta(5).*omega1+theta(6)));
end
function G = G7(theta,omega)
    omega1 = 1i*omega;
    G = (exp(-omega1.*theta(8)).*theta(1)).*((omega1.^4+theta(2).*omega1.^3+theta(3).*omega1.^2)./(omega1.^4+theta(4).*omega1.^3+theta(5).*omega1.^2+theta(6).*omega1+theta(7)));
end
function G = G8(theta,omega)
    omega1 = 1i*omega;
    G = (exp(-omega1.*theta(4)).*theta(1)).*((omega1+theta(2))./(omega1.^2+theta(3)));
end
function G = G9(theta,omega)
    omega1 = 1i*omega;
    G = (exp(-omega1.*theta(3)).*theta(1))./(omega1.^2+theta(2));
end