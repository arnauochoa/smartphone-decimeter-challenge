clear; close all;
config = Config.getInstance;
delete(config); % Delete previous instance of Config
config = Config.getInstance;

assert(strcmp(config.DATASET_TYPE, 'train'), 'Train set must be used to compute the geometry');

% Folder where all geometries are saved
folderpath = [projectPath filesep 'geometry' filesep 'baselines' filesep];
if ~exist(folderpath, 'dir'), mkdir(folderpath); end

% Get all campaign names
campaignNames = getValidDir(config.obsDataPath);

for iCampaign = 1:length(campaignNames)
    config.campaignName     = campaignNames{iCampaign};
    config.phoneNames       = getPhoneNamesInCampaign(config);
    
    %% Input
    [phones, nav, ~, osrRnx] = loadData();
    nPhones = length(config.phoneNames);
    ref = phones(1).ref;
    nEpochs = length(ref.utcSeconds);
    
    %% Groundtruth velocity for master phone
    refPosEcef = zeros(nEpochs, 3, nPhones);
    
    dtRef = diff(ref.utcSeconds);
    [xRef, yRef, zRef] = geodetic2ecef(wgs84Ellipsoid, ref.posLla(:, 1), ref.posLla(:, 2), ref.posLla(:, 3));
    refPosEcef(:, :, 1) = [xRef, yRef, zRef];
%     refVelEcef = diff(refPosEcef(:, :, 1)) ./ dtRef;
%     refVelTime = ref.utcSeconds(1:end-1) + dtRef/2;
%     refVelEcef = interp1(refVelTime, refVelEcef, ref.utcSeconds);
    
    %% Computation of geometry
    baselineEcef = nan(nEpochs, 3, nPhones);
    baselineBody = nan(nEpochs, 3, nPhones);
    
%     baselineEcef(:, :, 1) = zeros(nEpochs, 3);
    baselineNed(:, :, 1) = zeros(nEpochs, 3);
    baselineBody(:, :, 1) = zeros(nEpochs, 3);
    
    idxPhones = 1:nPhones;
    idxPhones(idxPhones == 1) = [];
    % Baseline between smartphones
    for iPhone = idxPhones
%         refPosEcef(:, :, iPhone) = geodetic2ecefVector(phones(iPhone).ref.posLla);
%         baselineEcef(:, :, iPhone) = refPosEcef(:, :, iPhone) - refPosEcef(:, :, 1);
        
        baselineNed(:, :, iPhone) = Lla2Ned(phones(iPhone).ref.posLla, phones(1).ref.posLla);
    end
    
    % Rotation of baseline
    for iEpoch = 1:nEpochs
        % Using velocity
        %     vEcef = refVelEcef(iEpoch, :)';
        %     vEcef = movmean(vEcef, 5, 1);
        %     vEcef = vEcef ./ norm(vEcef);
        %     vBody = [0 1 0]';
        %     if all(isnan(vEcef)) || norm(vEcef) == 0, continue; end
        %     Re2b = vrrotvec2mat(vrrotvec(vEcef, vBody));
        % %     Re2b = findRotMat(vEcef, vBody);
        
        yaw = phones(1).ref.trackDeg(iEpoch);
        Rned2b = [-sind(yaw) cosd(yaw) 0; cosd(yaw) sind(yaw) 0; 0 0 1];
        
        for iPhone = idxPhones
            baselineBody(iEpoch, :, iPhone) = (Rned2b * baselineNed(iEpoch, :, iPhone)')';
        end
    end
    
    
    %% Plots
    timeVec = ref.utcSeconds - ref.utcSeconds(1);
    % MAP plot
    f = figure;
    for iPhone = 1:nPhones
        geoplot(phones(iPhone).ref.posLla(:,1), phones(iPhone).ref.posLla(:,2));
        hold on;
    end
    geobasemap('none');
    legend(config.phoneNames);
    figureWindowTitle(f, 'Groundtruth map');
    % Coordinates plot
    dimensions = {'x_b', 'y_b', 'z_b'};
    for iPhone = 1:nPhones
        f = figure;
        for iDim = 1:3
            subplot(3, 1, iDim);
            plot(timeVec, baselineBody(:, iDim, iPhone)); hold on;
            plot(timeVec, nanmean(baselineBody(:, iDim, iPhone))*ones(size(timeVec)));
            %         ylim([-1 1]);
            xlabel('Time since start (s)'); ylabel([dimensions{iDim} ' (m)']);
        end
        figureWindowTitle(f, ['Position in body frame - ' config.phoneNames{iPhone}]);
    end
    
    % Compute geometry as mean of all epochs
    fprintf('>>>>> %s', config.campaignName(1:end-2));
    phoneNames      = config.phoneNames
    phonePosBody    = permute(mean(baselineBody, 1, 'omitnan'), [2 3 1])
    
%     fprintf('Press any key to save and continue')
%     pause;
    
    % Save geometry
    filename        = config.campaignName(1:end-2);
    save([folderpath filename], 'phoneNames', 'phonePosBody');
    
    clear baselineBody baselineNed
    close all
end

