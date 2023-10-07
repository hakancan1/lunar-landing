close all; clear, clc

% Moon constants
g0= 9.807; % Earth gravity (m/s^2)
mu= 4.905e12; % mu value of the Moon (m^3/s^2)
rho= 0; % atmospheric density of the Moon (kg/m^3)
r0= 1737e3; % radius of moon (m)

mpl= 200+200; % mass of the payload + structure (kg)
mpr= 600; % mass of the propellant (kg)
mr= mpr*.1+mpl; % mass of the reserve (kg)
Isp= 305; % specific impulse of the propulsion system (s)

mdot_max= 45e3/(Isp*g0); % maximum mass flow rate (kg/s)

% Setting up the simulation time domain
t_end= 60*30; % end time (s)
dt= 1e-2; % time step (s)
t= 0:dt:t_end; % time domain

% Initialize the state variables
x= zeros(size(t)); % horizontal position (m)
y= zeros(size(t)); % vertical position (m)
V= zeros(size(t)); %  velocity (m/s)
gamma= zeros(size(t)); %  trajectory angle (rad)
alpha= 180*ones(size(t)); %  TVC angle (rad)
r= zeros(size(t)); % radial position (m)
mdot= zeros(size(t)); % mass flow rate (kg/s)
a= zeros(size(t)); % acceleration (m/s^2)
m= zeros(size(t)); % mass of the vehicle (kg)
T= zeros(size(t)); % thrust of the propulsion system (N)
dGdt= zeros(size(t)); % derivative of trajectory angle (rad/s)
g= zeros(size(t));

% Set the initial conditions
y(1)= 500e3; % initial altitude (m)
r(1)= (y(1)+r0); % initial radius (m)
V(1)= sqrt(mu/r(1)); % circular orbit trajectory velocity (m/s)
m(1)= mpl+mpr; % initial mass (kg)

stage= "E"; % Start with entry stage
i_stage(1)= 1; % stage index
fuelFlag= 1; % boolean for fuel depletion

% Run the simulation
for i= 1:length(t)-1
    r(i)= (r0+y(i)); % instantanerous radius
    g(i)= mu/r(i)^2; % instantanerous gravity

    switch stage
        case "E"
            mdot(i)= 0.4*mdot_max;
            alpha(i)= 180;

            if V(i) < V(1)*.5
                a(1)= a(2);
                i_stage(end+1)= i;
                stage= "C";
                fprintf("Entry Phase Sucessful at t= %.2f s \n", t(i))
            end

        case "C"
            if y(i) < 50e3
                i_stage(end+1)= i;
                V_ref= V(i);
                stage= "B";
                fprintf("Cruise Phase Sucessful at t= %.2f s \n", t(i))
            end

        case "B"
            mdot(i)= .4*mdot_max;
            alpha(i)= 180+4;

            if y(i) < 30e3
                i_stage(end+1)= i;
                y_ref= y(i);
                V_ref= V(i);
                gamma_ref= gamma(i);
                stage= "L";
                fprintf("Brake Phase Sucessful at t= %.2f s \n", t(i))
            end

        case "L"
            Vpower= 2.0834; % Found by
            mdot(i)= (V(i)/V_ref)^Vpower*(0.4*mdot_max);
            mdot(i)= min(0.6*mdot_max, mdot(i)); % Min throttle
            mdot(i)= max(0.1*mdot_max, mdot(i)); % Max throttle

            alpha(i)= 180;

            if y(i) < 0
                stage= "TD";
                fprintf("Landing Sucessful at t= %.2f s \n", t(i))
            end

        case "TD"
            fprintf("TOUCH DOWN at t= %.2f s \n", t(i))
            i_stage(end+1)= i-1;
            t= t(1:i);
            mdot= mdot(1:i);
            x= x(1:i);
            y= y(1:i);
            r= r(1:i);
            V= V(1:i);
            a= a(1:i);
            m= m(1:i);
            T= T(1:i);
            gamma= gamma(1:i);
            dGdt= dGdt(1:i);
            alpha= alpha(1:i);
            break;

        case "NF" % No fuel stage
            mdot(i)= 0;
            alpha(i)= 180;

            if y(i) < 0
                stage= "TD";
            end

    end

    T(i)= mdot(i)*g0*Isp;

    % Time Marching
    a(i+1)= T(i)*cosd(alpha(i))/m(i) - g(i)*sind(gamma(i));
    V(i+1)= V(i) + a(i)*dt;

    dGdt(i+1)= V(i)*cosd(gamma(i))/r(i) + T(i)*sind(alpha(i))/(V(i)*m(i)) - g(i)*cosd(gamma(i))/V(i);
    gamma(i+1)= gamma(i) + 180*dGdt(i)*dt; % radians to deegree

    x(i+1)= x(i) + V(i)*cosd(gamma(i))*dt; % x position of the rocket
    y(i+1)= y(i) + V(i)*sind(gamma(i))*dt; % y position of the rocket

    m(i+1)= m(i) - mdot(i)*dt; % Update the mass of the vehicle

    % Mass control
    if m(i+1)-mr < 0 && fuelFlag== 1 % Check if the propellant mass is valid
        stage= "NF"; % Set the stage to No Fuel
        fprintf("No fuel at t= %.2f s \n", t(i));
        fuelFlag= 0;
    end
end

deltaV= Isp*g0*log(m(1)/m(end));
fprintf("\n ********************* Touchdown State ********************* \n")
fprintf("Velocity: V= %.2f m/s, Trajectory angle: gamma= %.0f° \n", V(end), gamma(end))
fprintf("Landing mass: m= %.1f kg, Reserve fuel rate: %.2f \n", m(end), (m(end)-mpl)/mpr)
fprintf("Mission delta-V: %.2f m/s \n", deltaV)


plotMission(t, i_stage, x, y, V, a, T, m, mr, alpha, gamma, dGdt, 1)
plotEnergy(i_stage, m, mdot, V, Isp, g, gamma, dt)

save("TrajectoryZ.mat", "t", "x", "y", "i_stage")
