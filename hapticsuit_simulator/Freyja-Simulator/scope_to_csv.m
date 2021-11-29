T = table(ScopeData{1}.Values.Time, ScopeData{1}.Values.Data, ScopeData{2}.Values.data, ScopeData{3}.Values.data);
T.Properties.VariableNames{1} = 'time';
T.Properties.VariableNames{2} = 'fx';
T.Properties.VariableNames{3} = 'fy';
T.Properties.VariableNames{4} = 'fz';
writetable(T,"c9.csv")