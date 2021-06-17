import math as m
thetas = [-33.75, -11.25, 11.25, 33.75]
#thetas = [-20, 0, 20]
for theta_desired in thetas:

	m_ind = [0, 1, 2]   # antenna distance in unit of lambda  

	phaseInc = 5.625 # step size for the phase shifter

	phaseTx_rad = [0, 0, 0]
	phaseTX_deg = [0, 0, 0]
	phaseTX_deg_wrap = [0, 0, 0]
	for i in range(3):
		phaseTx_rad[i] = 2*m.pi*(m_ind[i]*m.sin(m.radians(theta_desired)))
		phaseTX_deg[i] = m.degrees(phaseTx_rad[i])
		phaseTX_deg_wrap[i] = round((phaseTX_deg[i]%360)/phaseInc)
	#get sdk value
	phaseShifter = 0
	offset = [ m.pow(2,0), m.pow(2,8), m.pow(2,16)]
	for i in range(3):
		phaseShifter += phaseTX_deg_wrap[i]*offset[i]
	print(phaseShifter*4)