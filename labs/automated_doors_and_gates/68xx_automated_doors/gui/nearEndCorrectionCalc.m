%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %
 %      (C) Copyright 2016 Texas Instruments, Inc.
 %
 %  Redistribution and use in source and binary forms, with or without
 %  modification, are permitted provided that the following conditions
 %  are met:
 %
 %    Redistributions of source code must retain the above copyright
 %    notice, this list of conditions and the following disclaimer.
 %
 %    Redistributions in binary form must reproduce the above copyright
 %    notice, this list of conditions and the following disclaimer in the
 %    documentation and/or other materials provided with the
 %    distribution.
 %
 %    Neither the name of Texas Instruments Incorporated nor the names of
 %    its contributors may be used to endorse or promote products derived
 %    from this software without specific prior written permission.
 %
 %  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 %  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 %  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 %  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 %  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 %  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 %  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 %  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 %  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 %  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 %  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 %
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
function [nearEndCorr] = nearEndCorrectionCalc(P)
global NUM_ANGLE_BINS
    numAzimuthBins = P.dataPath.numAzimuthBins;
    numRangeBins = P.dataPath.numRangeBins; 
    rangeIdxToMeters = P.dataPath.rangeIdxToMeters;
    lam = 3e8/( P.profileCfg.startFreq * 1e9 )*1000; % mm

    nearEndCorr = zeros(numAzimuthBins,numRangeBins);
    % horizontal distance between tx1 and rx4, measured on the 16xx EVM to be 8.7 mm approx
    h_tx1_rx4 = 8.7;
    A = 0;
    B = lam;%(2+3/4)*lam + h_tx1_rx4; %2.5*lam;
    C = 2 * lam;
    D = C + h_tx1_rx4;
    E = D + 3* lam/2;
    rangeBiasMiliMeters = 65;    
    for rangeIdx = 0 : (numRangeBins-1)
        Ran = rangeIdxToMeters * rangeIdx *1000 - rangeBiasMiliMeters;
        
        Ran = max(Ran, 0);
        for azimIdx = 0 : (numAzimuthBins-1)
            if azimIdx < numAzimuthBins/2
                azimIdxS = azimIdx;
            else
                azimIdxS = azimIdx - numAzimuthBins;
            end
            th = 2*azimIdxS/numAzimuthBins;
            tha(azimIdx+1) = th;
            tx1 = sqrt(Ran^2 + (C-B)^2 - 2*Ran*(C-B)*th);
            rx4_tx1 = sqrt(Ran^2 + (D-B)^2 - 2*Ran*(D-B)*th);

            tx2 = sqrt(Ran^2 + (A-B)^2 - 2*Ran*(A-B)*th);
            rx1_tx2 = sqrt(Ran^2 + (E-B)^2 - 2*Ran*(E-B)*th);

            if Ran > 0
                nearEndCorr(azimIdx+1, rangeIdx+1) = exp(-1j * (2*pi/lam*((tx2 + rx1_tx2) - (rx4_tx1 + tx1)) - pi*th));            
            else
                nearEndCorr(azimIdx+1, rangeIdx+1) = exp(-1j*0);
            end
        end
    end
return