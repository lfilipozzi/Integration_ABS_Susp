%% First method: invert PSD
% Road PSD parameters
C = 1e-6;   % Roughness parameter
n = 2;

% Define nb of sample required for PSD from nb of sample of road and length
xmax = 100;     % Lenght of the road to generate (m)
N = 2^12;       % Number of road sample desired (must be a power of two)
M = N/2-1;      % Number of sample in the PSD (in frequency domain)
fmax = (N-1)/(2*xmax);  % Maximum frequency of the PSD
Df = fmax/M;    % Spacing in frequency domain

% Define wavelength
f = 0:Df:fmax;
w = 1./f;

% PSD of the road (ISO 8608)
PSD = C*w.^(-n);   % n: wavelength

A = sqrt(2*PSD);            % Convert PSD to amplitude
Phi = 2*pi*rand(size(A));   % Create a random phase
Phi(1) = 0;                 % First entry must be real
Z = A.*exp(j*Phi);          % Create complex nb from amplitude and phase
Zfft = [Z(1) Z(2:end) flip(conj(Z(2:end)))];
z = ifft(Zfft);

x = linspace(0,xmax,length(z));

figure(1)
plot(x,z)

%% Second method: approximation using sine
% Road PSD parameters
G = 4.6e-7;         % Roughness parameter
p = 2.57;           % Waviness

N = 100;            % Number of frequencies
Lmin = 1/15;        % Smallest wavelength considered
Lmax = 60;          % Biggest wavelength considered
nmin = 2*pi/Lmax;   % Smallest wavenumber
nmax = 2*pi/Lmin;   % Biggest wavenumber
Dn = (nmax - nmin)/N;
n = nmin:Dn:nmax;   % Wavenumber

S = G * n.^(-p);    % PSD
P = 2 * S.*(n>=0);  % One side PSD
A = sqrt(2*P*Dn);   % Amplitude
Phase = 2*pi*rand(size(A));  % Phase

xmax = 100;
x = linspace(0,xmax,10000);
z = zeros(size(x));
for i = 1:length(x)
    z(i) = sum(A.*sin(n*x(i) - Phase)) + sum(A.*sin(Phase));
end

figure(10)
plot(x,z)













