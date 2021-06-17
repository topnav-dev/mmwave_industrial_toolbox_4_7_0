theta_desired = 33.75;

m_ind = [0 1 2];   % antenna distance in unit of lambda  

phaseInc = 5.625; % step size for the phase shifter

    

phaseTX_rad = 2*pi*(m_ind*sind(theta_desired));

phaseTX_deg = (phaseTX_rad)*180/pi;

phaseTX_deg_wrap = wrapTo360(phaseTX_deg);

phaseShifter = round(phaseTX_deg_wrap/phaseInc) * [ 0, 2^8, 2^16].' * 4