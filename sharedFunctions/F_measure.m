% F-measure evaluation.

function FMeasure = F_measure(imBinary, imGT)
    
    TP = sum(sum(imGT==255&imBinary==255));		% True Positive 
   % TN = sum(sum(imGT<=50&imBinary==0));		% True Negative
    FP = sum(sum((imGT<=50)&imBinary==255));	% False Positive
    FN = sum(sum(imGT==255&imBinary==0));		% False Negative
    
    FMeasure = 2.0*TP/(2.0*TP + FP + FN);
    % confMat = [TP FP FN TN];
end
