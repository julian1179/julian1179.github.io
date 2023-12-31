clear *
clc

% Useful values for n:
    n488 = 1.4986;
    n532 = 1.494;
    n656 = 1.491; % approx
    n760 = 1.4877044217687;
    n780 = 1.48742346938775;
    n795 = 1.48695102040816;
    n1550 = 1.481;

% ----------------Grating Parameters---------------
lam_wr = 656.1 .*1e-9;
n_wr = n656;

th1_0_wr = [10, 17.77, 19]; % MUX'd Input- Static arm, 'left'
th2_0_wr = th1_0_wr;

% Substrate thickness
d = 2.265; % mm
d = d .* 1e-3; % m



% -----------------Read Parameters-----------------
lambda0_rd = [760, 780, 795] .*1e-9; % Write beam center wavelength
n_rd = [n780, n780, n780];

for k=1:length(lambda0_rd) % Find the center (aka '0') angle at the read wavelengths
    a1p = asind((sind(th1_0_wr))./n_wr);
    a2p = asind((sind(th2_0_wr))./n_wr);

    th1_0_rd = (asind((n_wr./n_rd).*(lambda0_rd./lam_wr).*sind((a1p + a2p)./2))) + ((a1p - a2p)./2);
    th2_0_rd = (asind((n_wr./n_rd).*(lambda0_rd./lam_wr).*sind((a1p + a2p)./2))) - ((a1p - a2p)./2);

    % Convert the angles to outside the material with Snell's law
    th1_0_rd_air = asind(n_rd.*sind(th1_0_rd));
    th2_0_rd_air = asind(n_rd.*sind(th2_0_rd));
end
clear a1p a2p

% % Uncomment to overwrite the calculated values
% th1_0_rd_air = 36.1627;
% th1_0_rd = asind((sind(th1_0_rd_air))./n_rd);

dn_eq = 1.8e-4; % Just a temporary variable to give all gratings equal strength
dn = [dn_eq, dn_eq, dn_eq]; % at read wavelengths
alpha = 0; % Assume absorption is equal at all wavelengths, for simplicity

normalize = 0;

lambda_rd = lambda0_rd; % Assume Bragg match at read wavelength
max_dth = 0.2;
pts = 1000;
theta_rd_air = linspace(min(th1_0_rd_air)-max_dth,max(th1_0_rd_air)+max_dth,pts); % read angle- equivalent to th1g
                      % th1g is defined as the write angle on the same side
                      % of the hologram that will be read


%

% ------------------Grating Maths------------------


% Snell's law
th1_0_wr = asind(sind(th1_0_wr)./n_wr);
th2_0_wr = asind(sind(th2_0_wr)./n_wr);

th1w_rad = th1_0_wr.*pi./180; % Radians
th2w_rad = th2_0_wr.*pi./180;

% Using complex notation to denote a 2D vector
k1 = exp(1i*th1w_rad).*2.*pi.*n_wr./(lam_wr);
k2 = exp(-1i*th2w_rad).*2.*pi.*n_wr./(lam_wr);

% From Kogelnik. Grating vector
K = k1-k2; % Grating k-vector
phir = angle(K); % radians
aK = abs(K);
LAM = 2*pi./aK;% Grating wavelength

%%
clc

fig = figure;
for i=1:3

    set(fig,'color', 'w');
    quiver(-real(k1(i)),-imag(k1(i)), real(k1(i)),imag(k1(i)),'AutoScaleFactor',1, 'Color', '#77AC30','Linewidth',2); hold on;
    quiver(-real(k2(i)),-imag(k2(i)), real(k2(i)),imag(k2(i)),'AutoScaleFactor',1, 'Color', '#77AC30','Linewidth',2);
    quiver(zeros(size(K(i),1),1),zeros(size(K(i),1),1),real(K(i)),imag(K(i)),'AutoScaleFactor',1, 'Color', '#0072BD','Linewidth',2);
    quiver(-real(k1(i)),-imag(k1(i)),real(K(i)),imag(K(i)),'AutoScaleFactor',1, 'Color', '#0072BD','Linewidth',2);

k1_rd = exp(1i*th1_0_rd.*pi./180).*2.*pi.*n_rd./(lambda_rd);
k2_rd = exp(-1i*th2_0_rd.*pi./180).*2.*pi.*n_rd./(lambda_rd);
    quiver(-real(k1_rd(i)),-imag(k1_rd(i)), real(k1_rd(i)),imag(k1_rd(i)),'AutoScaleFactor',1, 'Color', '#EDB120','Linewidth',2);
    quiver(-real(k2_rd(i)),-imag(k2_rd(i)), real(k2_rd(i)),imag(k2_rd(i)),'AutoScaleFactor',1, 'Color', '#EDB120','Linewidth',2);
    quiver(-real(k1_rd(i)),-imag(k1_rd(i)),real(K(i)),imag(K(i)),'AutoScaleFactor',1, 'Color', '#0072BD','Linewidth',2);
    grid on;
end
%% Bragg matched case
% clc
i = 3;
lambda = lambda_rd(i); %lam_wr;
theta_in = th1_0_rd(i);
theta_out = th2_0_rd(i);
n = n_rd(i);%n_wr;

d = 2.265e-3;
dn = 6.62e-5;
% eta = 0.3748;
% eta = 0.3312;
% eta = 0.2322;


rho = exp(1i*theta_in.*pi./180).*2.*pi.*n./(lambda);
sig = exp(1i*theta_out.*pi./180).*2.*pi.*n./(lambda);

cr = cos(angle(sig));
cs = cos(angle(rho));

eta =(sin(pi.*dn.*d./((lambda).*sqrt(cr.*cs)))).^2;
disp(sprintf('total %c = %.4f%%', 951, eta.*100));

% dn = asin(sqrt(eta)).* ((lambda).*sqrt(cr.*cs)) ./ (pi.*d)

%% General case
clc
i = 3;
lambda = lambda_rd(i); %lam_wr;
n = n_rd(i);%n_wr;

steps = 10;
d = 2.265e-3 ./steps; % 2.265e-3
dn = 6.62e-5;

theta_R = th1_0_rd(i).*pi./180;
theta_S = th2_0_rd(i).*pi./180;
theta = [theta_R, theta_S];

Kay = aK(i);
phi = phir(i);

% Initial values of the power of the beams.
A = [1, 0]; % [R, S]
I = abs(A).^2;

IO_indices = [  [1,2]; %Input-Output indices
                [2,1]];
% Find the diffraction efficiency and amplitude of each grating interaction.
for j=1 : size(IO_indices,1)
    i_in = IO_indices(j,1);
    i_out = IO_indices(j,2);
    
    % R and S vectors
    rho = exp(1i*theta(i_in)).*2.*pi.*n./(lambda);  % Input beam
    sig = exp(1i*theta(i_out)).*2.*pi.*n./(lambda); % Diffracted beam

    % Obliquity factors
    cr = cos(angle(sig));
    cs = cos(angle(rho));

    % Dephasing, eqn 17
    v = Kay.*cos(phi-theta(i_in)) - ((Kay.^2 ./ (4.*pi.*n)) .* lambda);

    % eqn 42
    nu = pi.*dn.*d./(lambda.*sqrt(cr.*cs));
    xi = v.*d./(2.*cs);
    %   Amplitude of the diffracted beam
    S = -1i.*sqrt(cr./cs) .* exp(-1i.*xi) .* sin(sqrt(nu.^2 + xi.^2)) ./ sqrt(1+ (xi./nu).^2);

    eta_S = (abs(cs)./cr) .* S.*conj(S);
    
    Diffracted{j} = S;
    eta_diffracted{j} = eta_S;
end

% Find the finite-difference power flow

for j=1:steps
    disp('-----------');
    disp(sprintf('j = %d, d=%.3f mm\n',j,j.*d.*1000));
    A_gain = zeros(length(A),length(A));
    
    for x=1:size(A,2) % Input beam index
        out = Diffracted{IO_indices(x,2)};
        
        for y=1:size(A,2) % Output beam index
            if y == x
                A_gain(x,y) = - I(x) .*out; % Ir= 1 - Is
            else
                A_gain(x,y) = I(x).*out;
            end
        end
    end
    
    for x=1:size(A,2)
        A(x) = A(x) + sum(A_gain(:,x));
    end
    I = abs(A).^2;
    
    disp(sprintf('     R= %.2e + %.2ei, Ir=%.3f\n     S= %.2e + %.2ei, Is=%.3f\n', real(A(1)), imag(A(1)), I(1), real(A(2)), imag(A(2)), I(2)));
end


diff_eff = I(2)./(1);
disp(sprintf('total %c = %.4f%%', 951, diff_eff.*100));





