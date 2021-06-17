function display_and_save_results(adc_filename, frameIdx, testResult, list_of_tests, testIndices, nCmpResult)
% (fname, testResult, list_of_tests, testIndices, nCmpResult)
global currTime
disp(adc_filename);
if frameIdx == 1
    currTime = now;
end
    
CSV_SQNRFname = [adc_filename(1:end-4) 'SQNR_results' num2str(currTime) '.csv'];
CSV_DetFname = [adc_filename(1:end-4) 'detection_results' num2str(currTime) '.csv'];

fid  = fopen(CSV_SQNRFname,'a');

header_str = 'Fname, frameIdx, Test , Median, 65%, 95%, Max';
disp(header_str)
fprintf(CSV_SQNRFname,'%%s\n',header_str);

for testIdx = testIndices
    line_str = [fname ', '...
        num2str(frameIdx) ', ' ...
        testResult{testIdx}.cmpOptions.testName ', ' ...
        num2str(testResult{testIdx}.SQNR_Stats.median) ', ' ...
        num2str(testResult{testIdx}.SQNR_Stats.orderStat65percent) ', ' ...
        num2str(testResult{testIdx}.SQNR_Stats.orderStat95percent) ', ' ...
        num2str(testResult{testIdx}.SQNR_Stats.max) ', ' ...
        ];
    disp(line_str);
    
    fprintf(CSV_SQNRFname,'%s\n',line_str);
end
fclose (fid);

fid  = fopen(CSV_DetFname,'a');

header_str = 'Fname, frameIdx, Test, Total Expected Detections, Missed Detections, False Detections' ;
disp(header_str);
fprintf(CSV_DetFname,'%%s\n',header_str);


true_det_objs_ind = numel(nCmpResult.detected_objs.SNRdB);

for testIdx  = testIndices
    missed_detections = numel(testResult{testIdx}.missed_detection_stats.SNRdB);
    false_detections = numel(testResult{testIdx}.false_alarm_stats.SNRdB);
    line_str = [fname ', '...
        num2str(frameIdx) ', ' ...
        testResult{testIdx}.cmpOptions.testName ', ' ...
        num2str(numel(true_det_objs_ind)) ', ' ...
        num2str(missed_detections) ', ' ...
        num2str(false_detections)];
    
    disp(line_str);
    fprintf(CSV_DetFname,'%s\n',line_str);
end
fclose (fid);
end