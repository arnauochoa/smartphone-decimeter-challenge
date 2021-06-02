% Compare our file with sample submission provided by Google

ourFilePath = 'data/results/test/all/result_20210602_160144.csv';

refTable = readtable('data/sample_submission.csv');
ourTable = readtable(ourFilePath);

if size(refTable, 1) == size(ourTable, 1)
    fprintf('Both tables have the same size\n')
else
    fprintf(2, 'Our table has %d entries while the reference has %d\n', size(ourTable, 1), size(refTable, 1));
end

refTableCampaigns = unique(refTable.phone);
timeDiff = {};
for iCamp = 1:length(refTableCampaigns)
    if any(strcmp(ourTable.phone, refTableCampaigns{iCamp}))
        nRefEntries = sum(strcmp(refTable.phone, refTableCampaigns{iCamp}));
        nOurEntries = sum(strcmp(ourTable.phone, refTableCampaigns{iCamp}));
        if nRefEntries == nOurEntries
            fprintf('  > %s : Both tables have the same size\n', refTableCampaigns{iCamp});
            refTableCamp = refTable(strcmp(refTable.phone, refTableCampaigns{iCamp}), :);
            ourTableCamp = ourTable(strcmp(ourTable.phone, refTableCampaigns{iCamp}), :);
            timeDiff{iCamp} = refTableCamp.millisSinceGpsEpoch - ourTableCamp.millisSinceGpsEpoch;
        else
            fprintf(2, '   > %s : Our table has %d entries while the reference has %d\n', refTableCampaigns{iCamp}, nRefEntries, nOurEntries);
        end
    else
        fprintf(2, 'Our result does not contain the campaign ''%s''\n', refTableCampaigns{iCamp})
    end
end