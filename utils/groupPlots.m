function navi = groupPlots(figures, navi)
desktop = com.mathworks.mde.desk.MLDesktop.getInstance;
navi.nav_report_group = desktop.addGroup('Navigation report');
desktop.setGroupDocked('Navigation report', 0);
myDim   = java.awt.Dimension(length(figures), 1);   % columns, rows
desktop.setDocumentArrangement('Navigation report', 1, myDim)
bakWarn = warning('off','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');
warning('off')
for k=1:length(figures)
    figures(k).WindowStyle = 'docked';
    drawnow;
    pause(0.02);  % Magic, reduces rendering errors
    set(get(handle(figures(k)), 'javaframe'), 'GroupName', 'Navigation report');
end
warning('on')
warning(bakWarn);
end