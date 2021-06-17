

clear all; close all;
param = setSystemParam(); 

%% two option provided
%  param.capturedDataFlag = 1: use pre-captured data
%  param.capturedDataFlag = 0: use generated data, no thermal/adc noise is added.
if (param.capturedDataFlag == 1)
    %%%%%%%%% load a raw data capture: Corner reflector around bin #38
    %%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %   Align these set of parameter with your data capture
	NUM_ADC_SAMPLES = param.capturedData.numAdcSamples; 
	NUM_CHIRPS =  param.capturedData.numChirps;
    NUM_VIRTUAL_ANTENNAS = param.capturedData.numVirtualAnt;

    %using hanning window
    win_range_fft=repmat(hanning(NUM_ADC_SAMPLES)',NUM_VIRTUAL_ANTENNAS,1,1,NUM_CHIRPS);
    hanning1(1,1,1,:)=hanning(NUM_CHIRPS)';
    win_doppler_fft=repmat(hanning1,NUM_VIRTUAL_ANTENNAS,NUM_ADC_SAMPLES,1,1);

    % loading and process only one frame of captured data.
    samples = NUM_ADC_SAMPLES * NUM_CHIRPS * NUM_VIRTUAL_ANTENNAS; 
    fid = fopen(param.capturedData.fileName,'r');
    dataChunk  = fread(fid,samples*2,'uint16','l');
    fclose(fid);
    
    %*********************************************************************
    %% Radar signal processing
    %*********************************************************************    
    dataChunk = dataChunk - (dataChunk >= 2^15) * 2^16;

    % through iwr6843 + DCA1000,  radar_data has data in the following format 
    % Rx0I0, Rx0I1, Rx0Q0, Rx0Q1, Rx0I2, Rx0I3, Rx0Q2, Rx0Q3, ...
    % The following script reshapes it into a 4-dimensional array.    
    len = length(dataChunk) / 2;
    adcOut(1:2:len) = dataChunk(1:4:end) + 1j*dataChunk(3:4:end);
    adcOut(2:2:len) = dataChunk(2:4:end) + 1j*dataChunk(4:4:end);
    %clear dataChunk
    adcOut = reshape(adcOut, NUM_ADC_SAMPLES, NUM_VIRTUAL_ANTENNAS, NUM_CHIRPS);
    adcOut = permute(adcOut, [2, 1, 3]);
    radarCube_raw = zeros(NUM_VIRTUAL_ANTENNAS, NUM_ADC_SAMPLES, 1, NUM_CHIRPS);
    for (nChirp = 1:NUM_CHIRPS)
        radarCube_raw(:,:,1,nChirp) = adcOut(:,:,nChirp);
    end
    
    % keep the range FFT output still within 16 bits integer.
    radarCube_rangeFFT= round((fft(radarCube_raw.*win_range_fft/sqrt(NUM_ADC_SAMPLES),[],2)));
    rangeFFTPower = sum(abs(radarCube_rangeFFT).^2, 1);
    dopplerFFT=fftshift(fft(radarCube_rangeFFT.*win_doppler_fft,[],4), 4);
    dopplerFFT=squeeze(dopplerFFT);    
    dopplerFFTPower = sum(abs(dopplerFFT).^2, 1);
    
else
%%%%%%%%%%%%%%%%%FIRST WE GENERATE A SAMPLE RADAR CUBE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	NUM_ADC_SAMPLES = param.simulatedData.numAdcSamples; 
	NUM_CHIRPS =  param.simulatedData.numChirps;
    NUM_VIRTUAL_ANTENNAS = param.simulatedData.numVirtualAnt;
    range_bin =  param.simulatedData.targetLoc.range_bin;
    doppler_bin =  param.simulatedData.targetLoc.doppler_bin;
    angle_bin =  param.simulatedData.targetLoc.angle_bin;

    %these are the frequencies after sampling
    w_adc = range_bin*2*pi/NUM_ADC_SAMPLES;
    w_chirp = doppler_bin*2*pi/NUM_CHIRPS;
    w_antenna= angle_bin*2*pi/NUM_VIRTUAL_ANTENNAS;

    %using hanning window
    win_range_fft=repmat(hanning(NUM_ADC_SAMPLES)',NUM_VIRTUAL_ANTENNAS,1,1,NUM_CHIRPS);
    hanning1(1,1,1,:)=hanning(NUM_CHIRPS)';
    win_doppler_fft=repmat(hanning1,NUM_VIRTUAL_ANTENNAS,NUM_ADC_SAMPLES,1,1);

    %generate radar cube : ADC samples and multiple chirps for one antenna 
    radarCube_raw_temp=exp(1j*w_adc*[0:NUM_ADC_SAMPLES-1]')*exp(1j*w_chirp*[0:NUM_CHIRPS-1]);

    %replicate the radar cube with appropriate phase shift across multiple virtual antennas  
    for(k=1:NUM_VIRTUAL_ANTENNAS)
        radarCube_raw(k,:,1,:)=radarCube_raw_temp*exp(1j*w_antenna*(k-1));	
    end

    %quantize to 16 bits
    radarCube_rangeFFT= round((2^10)*(fft(radarCube_raw.*win_range_fft,[],2)));
    dopplerFFT=fftshift(fft(radarCube_rangeFFT.*win_doppler_fft,[],4), 4);
    dopplerFFT=squeeze(dopplerFFT);
    dopplerFFTPower = sum(abs(dopplerFFT).^2, 1);
    
end

%%%%%%%% NEXT  WE APPLY COMPRESSION TO radarCube_rangeFFT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The incoming data is assumed to be of the form 
%  [Channels, RangeBins, TxOrder, Chirps]

% use the string to provide the compression options
[cmpOptions dcmpOptions]=generate_compression_params(param.cmpOptions,[]);
[cmpOut dimsOrig]=compress_data(radarCube_rangeFFT,cmpOptions);
dcmpOut=decompress_data(cmpOut,dcmpOptions,dimsOrig);
dcmpOut_dopplerFFT=fftshift(fft(dcmpOut.*win_doppler_fft,[],4), 4);
dcmpOut_dopplerFFT=squeeze(dcmpOut_dopplerFFT);

%%%%%%%%%%%%%LOOK AT THE NOISE FLOOR ACROSS THE DOPPLER-FFT (for the range gate corresponding to the target)
figure(1);
imagesc(0: NUM_ADC_SAMPLES-1, -NUM_CHIRPS/2 : NUM_CHIRPS/2-1, squeeze(((dopplerFFTPower))).')
xlabel('Range index')
ylabel('Doppler index')
title('Range-Doppler heatmap')
% subplot(2, 1, 1);
% plot(0: NUM_ADC_SAMPLES-1, squeeze(10*log10(dopplerFFTPower)))
% title('DopplerFFTOut (accumulated power over all antennas)')
% xlabel('Range index')
% ylabel('Amplitude (dB)')
% grid
% subplot(2, 1, 2)
% plot(-NUM_CHIRPS/2: NUM_CHIRPS/2-1, squeeze((10*log10(dopplerFFTPower))).')
% title('DopplerFFTOut (accumulated power over all antennas)')
% xlabel('Doppler index')
% ylabel('Amplitude (dB)')
% grid

figure(2);
range_bin = param.resultDemo.rangeBin;
plot(-NUM_CHIRPS/2:NUM_CHIRPS/2-1, 20*log10(squeeze(abs((dcmpOut_dopplerFFT(1,range_bin+1,:))))),'b');hold on
plot(-NUM_CHIRPS/2:NUM_CHIRPS/2-1, 20*log10(squeeze(abs((dopplerFFT(1,range_bin+1,:))))),'r')
legend(strcat('with compression ratio of ', num2str(param.cmpOptions.cmpRatio)),'without compression')
xlabel('Doppler index')
ylabel('Amplitude (dB)')
title(strcat('DopplerOut on TX1-RX1 for rangeBin = ', num2str(range_bin)))
grid


