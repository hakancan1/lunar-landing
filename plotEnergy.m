function plotEnergy(ix, m, mdot, V, Isp, g, gamma, dt)
g0= 9.807;

Mc= [m(ix(1))-m(ix(2));
    m(ix(2))-m(ix(3));
    m(ix(3))-m(ix(4));
    m(ix(4))-m(ix(5))];

dVc= Isp*g0*[log(m(ix(1))/m(ix(2)));
            log(m(ix(2))/m(ix(3)));
            log(m(ix(3))/m(ix(4)));
            log(m(ix(4))/m(ix(5)))];

gLc= dt*Isp*g0* [sum(log(m(ix(1):ix(2))/m(ix(2))).*(g(ix(1):ix(2)).*sind(gamma(ix(1):ix(2)))));
                sum(log(m(ix(2):ix(3))/m(ix(3))).*(g(ix(2):ix(3)).*sind(gamma(ix(2):ix(3)))));
                sum(log(m(ix(3):ix(4))/m(ix(4))).*(g(ix(3):ix(4)).*sind(gamma(ix(3):ix(4)))));
                sum(log(m(ix(4):ix(5))/m(ix(5))).*(g(ix(4):ix(5)).*sind(gamma(ix(4):ix(5)))))];

gLc= sqrt(-2*gLc); % G= .5*V^2

X= categorical(["Entry", "Cruise", "Brake", "Landing"]);
X = reordercats(X,string(X));
Y= [Mc, dVc, gLc];

figure
bar(X, Y), title("Energy Requirements for Lunar Landing")
grid, legend(["Consumed Propellant", "ΔV_{del}", "Gravity Loss"])
ylabel("Propellant Mass (kg)    |   ΔV (m/s)    |   Gravity Loss (m/s)")