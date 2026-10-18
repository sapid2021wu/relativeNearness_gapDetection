% Evaluating the scale-invariant mean squared error (in log space).

function  SILog = Silog(groundTruth, predicted)
% groundTruth: ground truth for the relative nearness map.
% predicted: relative nearness map predicted by the model.

    log_y = log(predicted);  
    log_y_star = log(groundTruth);
    alpha = mean(log_y_star(:) - log_y(:));

    di = log_y - log_y_star + alpha;
    SILog = mean(di(:).^2);
end