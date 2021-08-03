config = Config.getInstance;
delete(config); % Delete previous instance of Config
config = Config.getInstance;

tic

config.DATASET_TYPE = 'train';
main;
folderName = [workspacePath 'figs/good/train/'];
saveFigs(folderName);

config.DATASET_TYPE = 'test';
main;
folderName = [workspacePath 'figs/good/test/'];
saveFigs(folderName);

fprintf('\n')
toc