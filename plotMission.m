function plotMission(t, i_stage, x, y, V, a, T, m, mr, alpha, gamma, dGdt, iS)

a_max= -9.807*3; % 3*g constaint
Tmax= 45e3; % Maximum thrust

Ei= i_stage(iS):i_stage(end); % index range of the desired stages
tE= t(Ei); % time domain of the desired stages

figure, plot(x(Ei)*1e-3, y(Ei)*1e-3, LineWidth= 2), title("Trajectory")
hold on, plot(x(i_stage)*1e-3, y(i_stage)*1e-3, "kv")
xlabel("x (km)"), ylabel("y (km)"), grid, axis([0 500 -25 500])
yline(0, "LineWidth", 2)

figure, plot(tE, y(Ei)*1e-3, LineWidth= 2), title("Altitude")
xlabel("time (s)"), ylabel("y (m)"), grid
xline(t(i_stage(iS:end)), "LineStyle", "--")

figure, plot(tE, V(Ei).*sind(gamma(Ei)), "b-", tE, V(Ei).*cosd(gamma(Ei)), "r-", tE, V(Ei), "k:", LineWidth= 2)
xlabel("time (s)"), ylabel("V (m/s)"), title("Velocity"), grid
legend("Vy", "Vx", "V"), xline(t(i_stage(iS:end)), "LineStyle", ":", "HandleVisibility", "off")

figure, plot(tE, a(Ei), LineWidth= 2), title("Acceleration")
xlabel("time (s)"), ylabel("a (m/s^2)"), grid
xline(t(i_stage(iS:end)), "LineStyle", "--")
yline(a_max, "LineStyle", "-.", "LineWidth", 2)

figure, plot(tE, T(Ei)*1e-3, LineWidth= 2), title("Thrust")
xlabel("time (s)"), ylabel("T (kN)"), grid
yline(Tmax*[1 .6 .1]*1e-3, "LineStyle", "-.")
xline(t(i_stage(iS:end)), "LineStyle", "--")

figure, hold on, plot(tE, m(Ei), LineWidth= 2), title("Mass")
xlabel("time (s)"), ylabel("mass (kg)"), grid
xline(t(i_stage(iS:end)), "LineStyle", "--")
yline(mr, "LineStyle", "-.", "LineWidth", 2)

figure, plot(tE, alpha(Ei), LineWidth= 2), title("TVC angle")
xlabel("time (s)"), ylabel("alpha (degrees)"), grid
xline(t(i_stage(iS:end)), "LineStyle", "--")

figure, plot(tE, gamma(Ei), LineWidth= 2), title("Trajectory Angle")
xlabel("time (s)"), ylabel("gamma (degrees)"), grid
xline(t(i_stage(iS:end)), "LineStyle", "--")

figure, plot(tE, dGdt(Ei), LineWidth= 2), title("dGdt")
xlabel("time (s)"), ylabel("dGdt (degree/s)"), grid
xline(t(i_stage(iS:end)), "LineStyle", "--")
