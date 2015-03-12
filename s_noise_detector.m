function [Noise_level, mean_vec] = s_noise_detector(botSim)
% input: botsim with location
%output: noise level and avereged first reading
trials = 200
for i=1:trials
    botScan(:,i) = botSim.ultraScan()  ;
end




mean_vec= mean(botScan,2);
errors = botScan - repmat(mean_vec,1,size(botScan,2));
S = std(errors');
Noise_level = max(S);





end