function campaign = getGeometryForTest(testCampaignName)
switch testCampaignName
    case '2020-05-15-US-MTV-1'
        campaign = '2020-05-21-US-MTV-2';
    case '2020-05-28-US-MTV-1'
        campaign = '2020-05-21-US-MTV-2';
    case '2020-05-28-US-MTV-2'
        campaign = '2020-05-29-US-MTV-1';
    case '2020-06-04-US-MTV-2'
        campaign = '2020-06-04-US-MTV-1';
    case '2020-06-10-US-MTV-1'
        campaign = '2020-06-05-US-MTV-1';
    case '2020-06-10-US-MTV-2'
        campaign = '2020-06-05-US-MTV-1';
    case '2020-08-13-US-MTV-1'
        campaign = '2020-08-03-US-MTV-1';
    case '2020-08-03-US-MTV-2'
        campaign = '2020-08-06-US-MTV-2';
    case '2021-03-16-US-RWC-2'
        campaign = '2021-04-15-US-MTV-1';
    case '2021-03-25-US-PAO-1'
        campaign = '';
    case '2021-04-02-US-SJC-1'
        campaign = '';
    case '2021-04-08-US-MTV-1'
        campaign = '';
    case '2021-03-16-US-MTV-2'
        campaign = '';
    case '2021-04-21-US-MTV-1'
        campaign = '2021-04-15-US-MTV-1';
    case '2021-04-22-US-SJC-2'
        campaign = '';
    case '2021-04-26-US-SVL-2'
        campaign = '';
    case '2021-04-28-US-MTV-2'
        campaign = '2021-04-28-US-MTV-1';
    case '2021-04-29-US-MTV-2'
        campaign = '2021-04-29-US-MTV-1';
    case '2021-04-29-US-SJC-3'
        campaign = '2021-04-29-US-SJC-2';
end
end