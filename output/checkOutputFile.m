function checkOutputFile(ourFilePath)
% Compare our file with sample submission provided by Google
if nargin < 1
    ourFilePath = 'data/results/test/all/result_20210602_173112.csv';% 160144
end

refTable = readtable('data/sample_submission.csv');
ourTable = readtable(ourFilePath);

if size(refTable, 1) == size(ourTable, 1)
    fprintf('Both tables have the same size\n')
else
    fprintf(2, 'Our table has %d entries while the reference has %d\n', size(ourTable, 1), size(refTable, 1));
end

refTableCampaigns = unique(refTable.phone);
refTableCamp = cell(1, length(refTableCampaigns));
ourTableCamp = cell(1, length(refTableCampaigns));
timeDiff = cell(1, length(refTableCampaigns));
for iCamp = 1:length(refTableCampaigns)
    if any(strcmp(ourTable.phone, refTableCampaigns{iCamp}))
        nRefEntries = sum(strcmp(refTable.phone, refTableCampaigns{iCamp}));
        nOurEntries = sum(strcmp(ourTable.phone, refTableCampaigns{iCamp}));
        refTableCamp{iCamp} = refTable(strcmp(refTable.phone, refTableCampaigns{iCamp}), :);
        ourTableCamp{iCamp} = ourTable(strcmp(ourTable.phone, refTableCampaigns{iCamp}), :);
        if nRefEntries == nOurEntries
            fprintf('  = %s : Both tables have the same size\n', refTableCampaigns{iCamp});
            timeDiff{iCamp} = refTableCamp{iCamp}.millisSinceGpsEpoch - ourTableCamp{iCamp}.millisSinceGpsEpoch;
        elseif nOurEntries < nRefEntries 
            fprintf(2, '  < %s : Our table has %d entries while the reference has %d\n', refTableCampaigns{iCamp}, nOurEntries, nRefEntries);
        else
            fprintf(2, '  > %s : Our table has %d entries while the reference has %d\n', refTableCampaigns{iCamp}, nOurEntries, nRefEntries);
        end
    else
        fprintf(2, 'Our result does not contain the campaign ''%s''\n', refTableCampaigns{iCamp})
    end
end
end
% figure; plot([refTableCamp{3}.millisSinceGpsEpoch; refTableCamp{3}.millisSinceGpsEpoch(end)*[1;1;1]] - ourTableCamp{3}.millisSinceGpsEpoch)
% dtRef = diff(refTableCamp{4}.millisSinceGpsEpoch);
% dtOur = diff(ourTableCamp{4}.millisSinceGpsEpoch);
% figure; plot(dtRef)
% figure; plot(dtOur)