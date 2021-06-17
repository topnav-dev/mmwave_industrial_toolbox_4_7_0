classdef setup_as_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        SetupUIFigure                 matlab.ui.Figure
%         GridLayout                    matlab.ui.container.GridLayout
%         LeftPanel                     matlab.ui.container.Panel
        VisualizerModeButtonGroup     matlab.ui.container.ButtonGroup
        RealTimeButton                matlab.ui.control.ToggleButton
        PlayBackButton                matlab.ui.control.ToggleButton
        COMPanel                      matlab.ui.container.Panel
        CFG_PORTLabel                 matlab.ui.control.Label
        CfgPort                       matlab.ui.control.NumericEditField
        DATA_PORTLabel                matlab.ui.control.Label
        DatPort                       matlab.ui.control.NumericEditField
        TestConnectionButton          matlab.ui.control.Button
        DatPanel                      matlab.ui.container.Panel
        DatButton                     matlab.ui.control.Button
        DataFileLabel                 matlab.ui.control.Label
        DatPath                       matlab.ui.control.EditField
        SelectCFGFilePanel            matlab.ui.container.Panel
        CFGFileLabel                  matlab.ui.control.Label
        CfgPath                       matlab.ui.control.EditField
        CfgButton                     matlab.ui.control.Button
        RecordDataPanel               matlab.ui.container.Panel
        SaveDirectoryLabel            matlab.ui.control.Label
        SavePath                      matlab.ui.control.EditField
        LogButton                     matlab.ui.control.Button
        EnableRecord                  matlab.ui.control.CheckBox
        FileNameLabel                 matlab.ui.control.Label
        SaveName                      matlab.ui.control.EditField
        AppendtimedateCheckBox        matlab.ui.control.CheckBox
        SensorInformationButtonGroup  matlab.ui.container.ButtonGroup
        SensorMountingHeightmEditFieldLabel  matlab.ui.control.Label
        MountingHeight                matlab.ui.control.NumericEditField
        SensorElevationTiltdegEditFieldLabel  matlab.ui.control.Label
        ElevationTilt                 matlab.ui.control.NumericEditField
        SetupCompleteButton           matlab.ui.control.Button
        VisualizerOptionsButtonGroup  matlab.ui.container.ButtonGroup
        RangeCriticalZoneStartmLabel  matlab.ui.control.Label
        CriticalStart                 matlab.ui.control.NumericEditField
        RangeCriticalZoneEndmLabel    matlab.ui.control.Label
        CriticalEnd                   matlab.ui.control.NumericEditField
        EnableZones                   matlab.ui.control.CheckBox
        RangeWarningZoneStartmLabel   matlab.ui.control.Label
        WarnStart                     matlab.ui.control.NumericEditField
        RangeWarningZoneEndmLabel     matlab.ui.control.Label
        WarnEnd                       matlab.ui.control.NumericEditField
        ProjectionTimesecLabel        matlab.ui.control.Label
        ProjTime                      matlab.ui.control.NumericEditField
%         RightPanel                    matlab.ui.container.Panel
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
    end

    
    
    properties (Access = public)
        mode = 1;
        datFile = struct('name', [], 'path', []);
        cfgFile = struct('name', [], 'path', []);
        logFile = struct('name', [], 'path', []);
        offset = struct('height', [], 'az', [], 'rot', []);
        comPort = struct('cfg',1,'data',1,'status',0);
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            
            try
               
                if exist('state.mat')
                    load('state.mat')
                    app.RealTimeButton.Value = REAL_TIME_MODE;
                    VisualizerModeButtonGroupSelectionChanged(app, []);
                    app.EnableRecord.Value = ENABLE_RECORD;
                    app.DatPath.Value = [datFile.path datFile.name];
                    app.CfgPath.Value = [cfgFile.path cfgFile.name];
                    
                    app.SavePath.Value = logFile.path;
                    app.SaveName.Value = logFile.name;
                    app.MountingHeight.Value = offset.height;
                    app.ElevationTilt.Value = offset.el;
                    
                    app.CfgPort.Value = comPort.cfg;
                    app.DatPort.Value = comPort.data;
                    
                    app.EnableZones.Value = zones.enable;
                    app.CriticalStart.Value = zones.criticalStart;
                    app.CriticalEnd.Value = zones.criticalEnd;
                    app.WarnStart.Value = zones.warnStart;
                    app.WarnEnd.Value = zones.warnEnd;
                    app.ProjeTime.Value = zone.projTime;
                end
            catch ME
                
            end
            uiwait(app.SetupUIFigure)
        end

        % Changes arrangement of the app based on UIFigure width
        function updateAppLayout(app, event)
            currentFigureWidth = app.SetupUIFigure.Position(3);
%             if(currentFigureWidth <= app.onePanelWidth)
%                 % Change to a 2x1 grid
%                 app.GridLayout.RowHeight = {644, 644};
%                 app.GridLayout.ColumnWidth = {'1x'};
%                 app.RightPanel.Layout.Row = 2;
%                 app.RightPanel.Layout.Column = 1;
%             else
%                 % Change to a 1x2 grid
%                 app.GridLayout.RowHeight = {'1x'};
%                 app.GridLayout.ColumnWidth = {552, '1x'};
%                 app.RightPanel.Layout.Row = 1;
%                 app.RightPanel.Layout.Column = 2;
%             end
        end

        % Selection changed function: VisualizerModeButtonGroup
        function VisualizerModeButtonGroupSelectionChanged(app, event)
            if(app.RealTimeButton.Value)
                app.mode = 1;
                app.DatPanel.Visible = 'off';
                app.COMPanel.Visible = 'on';
                app.RecordDataPanel.Visible = 'on';
            else
                app.mode = 2;
                app.DatPanel.Visible = 'on';
                app.COMPanel.Visible = 'off';
                app.RecordDataPanel.Visible = 'off';
                
            end
            
        end

        % Button pushed function: DatButton
        function DatButtonPushed(app, event)
            [app.datFile.name app.datFile.path] = uigetfile('*.txt','Select Log File');
            if(ischar(app.datFile.name))
                app.DatPath.Value = [app.datFile.path app.datFile.name] ;
            end
            figure(app.SetupUIFigure);
        end

        % Button pushed function: CfgButton
        function CfgButtonPushed(app, event)
            [app.cfgFile.name app.cfgFile.path] = uigetfile('*.cfg','Select CFG File');
            if(ischar(app.cfgFile.name))
                app.CfgPath.Value =  [app.cfgFile.path app.cfgFile.name] ;
            end
            figure(app.SetupUIFigure);
            %movegui(app.SetupUIFigure,'center')
        end

        % Button pushed function: LogButton
        function LogButtonPushed(app, event)
            app.logFile.path = uigetdir('Select Folder to Save');
            if(ischar(app.logFile.path))
                app.SavePath.Value = [app.logFile.path];
            end
            figure(app.SetupUIFigure);
            %movegui(app.SetupUIFigure,'center')
        end

        % Value changed function: CfgPath
        function CfgPathValueChanged(app, event)
            [app.cfgFile.path,name,ext] = fileparts(app.CfgPath.Value);
            app.cfgFile.name = [name ext];
        end

        % Value changed function: DatPath
        function DatPathValueChanged(app, event)
            [app.datFile.path,name,ext] = fileparts(app.DatPath.Value);
            app.datFile.name = [name ext];
        end

        % Value changed function: CfgPort
        function CfgPortValueChanged(app, event)
            app.comPort.cfg = app.CfgPort.Value;
        end

        % Value changed function: DatPort
        function DatPortValueChanged(app, event)
            app.comPort.data = app.DatPort.Value;
        end

        % Button pushed function: TestConnectionButton
        function TestConnectionButtonPushed(app, event)
            hDataPort = initDataPort(app.DatPort.Value);
            hCfgPort = initCfgPort(app.CfgPort.Value);
            
            % send version command
            if(hCfgPort ~= -1 && hDataPort ~=-1)
                if(hDataPort.BytesAvailable)
                    %TODO: remove warning when config reload w/o NRST is enabled
                    uialert(app.SetupUIFigure, 'Device appears to already be running. Will not be able to load a new configuration. To load a new config, press NRST on the EVM and try again.','Need to assert NRST');    
                    app.comPort.status = -1;
                    return;
                else       
                    fprintf(hCfgPort, 'version');
                    pause(0.5); % adding some delay to make sure bytes are received
                    response = '';
                    if(hCfgPort.BytesAvailable)
                        for i=1:10 % version command reports back 10 lines TODO: change if SDK changes response
                            rstr = fgets(hCfgPort);
                            response = append(response, rstr);
                        end
                        uialert(app.SetupUIFigure, response,'Test successful: CFG Port Opened & Data Received','icon','success');
                        app.comPort.status = 1;
                    else
                        uialert(app.SetupUIFigure,'Port opened but no response received. Check port # and SOP mode on EVM','Issue with CFG Port');
                        app.comPort.status = -2;
                        fclose(hDataPort);
                        fclose(hCfgPort);
                    end
                end
            else
                app.comPort.status = -2;
                uialert(app.SetupUIFigure, 'Could not open ports. Check port # and that EVM is powered with correct SOP mode.','Ports not valid');    
            end
        end

        % Button pushed function: SetupCompleteButton
        function SetupCompleteButtonPushed(app, event)

           REAL_TIME_MODE = app.RealTimeButton.Value;
           ENABLE_RECORD = app.EnableRecord.Value;
           [dPath,dName,dExt] = fileparts(app.DatPath.Value);
           datFile.path = [dPath '\'];
           datFile.name = [dName dExt];
           [cPath,cName,cExt] = fileparts(app.CfgPath.Value);
           cfgFile.path = [cPath '\'];
           cfgFile.name = [cName cExt];
           logFile.path = app.SavePath.Value;
           logFile.name = app.SaveName.Value;
           offset.height = app.MountingHeight.Value;
           offset.el = app.ElevationTilt.Value;
           comPort.cfg = app.CfgPort.Value;
           comPort.data = app.DatPort.Value;
           comPort.status = app.comPort.status;
           zones = struct('enable', app.EnableZones.Value,...
                'criticalStart', app.CriticalStart.Value, ...
                'criticalEnd', app.CriticalEnd.Value, ...
                'warnStart', app.WarnStart.Value, ...
                'warnEnd', app.WarnEnd.Value,...
                'projTime', app.ProjTime.Value);

            assignin('base', 'REAL_TIME_MODE', REAL_TIME_MODE);
            assignin('base', 'ENABLE_RECORD', ENABLE_RECORD);
            assignin('base', 'datFile', datFile);
            assignin('base', 'cfgFile', cfgFile);
            assignin('base', 'logFile', logFile);
            assignin('base', 'comPort', comPort);
            assignin('base', 'offset', offset);
            assignin('base', 'zones', zones);
            save('state.mat','REAL_TIME_MODE', 'ENABLE_RECORD','datFile','cfgFile','logFile','offset','comPort','zones');
            closereq
        end

        % Value changed function: EnableZones
        function EnableZonesValueChanged(app, event)
            value = app.EnableZones.Value;
            if(value)
                app.CriticalStart.Enable = 'on';
                app.CriticalEnd.Enable = 'on';
                app.WarnStart.Enable = 'on';
                app.WarnEnd.Enable = 'on';
                app.ProjTime.Enable  = 'on';
            else
                app.CriticalStart.Enable = 'off';
                app.CriticalEnd.Enable = 'off';
                app.WarnStart.Enable = 'off';
                app.WarnEnd.Enable = 'off';
                app.ProjTime.Enable  = 'off';
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create SetupUIFigure and hide until all components are created
            app.SetupUIFigure = uifigure('Visible', 'off');
            app.SetupUIFigure.AutoResizeChildren = 'on';
            app.SetupUIFigure.Position = [100 100 564 644];
            %movegui(app.SetupUIFigure,'center');
            app.SetupUIFigure.Name = 'Setup ';
            %app.SetupUIFigure.SizeChangedFcn = createCallbackFcn(app, @updateAppLayout, true);
%             app.SetupUIFigure.Scrollable = 'on';

            % Create GridLayout
%             app.GridLayout = uigridlayout(app.SetupUIFigure);
%             app.GridLayout.ColumnWidth = {552, '1x'};
%             app.GridLayout.RowHeight = {'1x'};
%             app.GridLayout.ColumnSpacing = 0;
%             app.GridLayout.RowSpacing = 0;
%             app.GridLayout.Padding = [0 0 0 0];
%             app.GridLayout.Scrollable = 'on';

            % Create LeftPanel
%             app.LeftPanel = uipanel(app.SetupUIFigure);
%             app.LeftPanel.Layout.Row = 1;
%             app.LeftPanel.Layout.Column = 1;
%             app.LeftPanel.Scrollable = 'on';

            % Create VisualizerModeButtonGroup
            app.VisualizerModeButtonGroup = uibuttongroup(app.SetupUIFigure);
            app.VisualizerModeButtonGroup.SelectionChangedFcn = createCallbackFcn(app, @VisualizerModeButtonGroupSelectionChanged, true);
            app.VisualizerModeButtonGroup.Title = 'Visualizer Mode';
            app.VisualizerModeButtonGroup.FontWeight = 'bold';
            app.VisualizerModeButtonGroup.FontSize = 14;
            app.VisualizerModeButtonGroup.Position = [8 447 535 173];

            % Create RealTimeButton
            app.RealTimeButton = uitogglebutton(app.VisualizerModeButtonGroup);
            app.RealTimeButton.Text = 'Real Time';
            app.RealTimeButton.Position = [11 118 240 22];
            app.RealTimeButton.Value = true;

            % Create PlayBackButton
            app.PlayBackButton = uitogglebutton(app.VisualizerModeButtonGroup);
            app.PlayBackButton.Text = 'Play Back';
            app.PlayBackButton.Position = [278 118 240 22];

            % Create COMPanel
            app.COMPanel = uipanel(app.VisualizerModeButtonGroup);
            app.COMPanel.Title = 'Set COM port for real time';
            app.COMPanel.Position = [11 8 240 97];

            % Create CFG_PORTLabel
            app.CFG_PORTLabel = uilabel(app.COMPanel);
            app.CFG_PORTLabel.Position = [7 48 71 22];
            app.CFG_PORTLabel.Text = 'CFG_PORT';

            % Create CfgPort
            app.CfgPort = uieditfield(app.COMPanel, 'numeric');
            app.CfgPort.Limits = [1 99];
            app.CfgPort.ValueChangedFcn = createCallbackFcn(app, @CfgPortValueChanged, true);
            app.CfgPort.HorizontalAlignment = 'center';
%            app.CfgPort.Tooltip = {'Appears as User UART or Standard Port in Device Manager'};
            app.CfgPort.Position = [93 48 33 22];
            app.CfgPort.Value = 1;

            % Create DATA_PORTLabel
            app.DATA_PORTLabel = uilabel(app.COMPanel);
            app.DATA_PORTLabel.Position = [7 18 76 22];
            app.DATA_PORTLabel.Text = 'DATA_PORT';

            % Create DatPort
            app.DatPort = uieditfield(app.COMPanel, 'numeric');
            app.DatPort.Limits = [1 99];
            app.DatPort.ValueChangedFcn = createCallbackFcn(app, @DatPortValueChanged, true);
            app.DatPort.HorizontalAlignment = 'center';
%            app.DatPort.Tooltip = {'Appears as Data or Enhanced Port in Device Manager'};
            app.DatPort.Position = [93 18 33 22];
            app.DatPort.Value = 1;

            % Create TestConnectionButton
            app.TestConnectionButton = uibutton(app.COMPanel, 'push');
            app.TestConnectionButton.ButtonPushedFcn = createCallbackFcn(app, @TestConnectionButtonPushed, true);
            app.TestConnectionButton.Position = [132 48 102 22];
            app.TestConnectionButton.Text = 'Test Connection';

            % Create DatPanel
            app.DatPanel = uipanel(app.VisualizerModeButtonGroup);
            app.DatPanel.Title = 'Select data log file for play back';
            app.DatPanel.Position = [278 8 240 97];

            % Create DatButton
            app.DatButton = uibutton(app.DatPanel, 'push');
            app.DatButton.ButtonPushedFcn = createCallbackFcn(app, @DatButtonPushed, true);
            app.DatButton.Icon = 'folder_file_icon.png';
            app.DatButton.Position = [78 10 154 22];
            app.DatButton.Text = 'Browse';

            % Create DataFileLabel
            app.DataFileLabel = uilabel(app.DatPanel);
            app.DataFileLabel.HorizontalAlignment = 'right';
            app.DataFileLabel.Position = [9 48 54 22];
            app.DataFileLabel.Text = 'Data File';

            % Create DatPath
            app.DatPath = uieditfield(app.DatPanel, 'text');
            app.DatPath.ValueChangedFcn = createCallbackFcn(app, @DatPathValueChanged, true);
            app.DatPath.FontSize = 10;
%            app.DatPath.Tooltip = {'Enter full path to file or use Browse button.'};
            app.DatPath.Position = [78 39 154 31];

            % Create SelectCFGFilePanel
            app.SelectCFGFilePanel = uipanel(app.SetupUIFigure);
            app.SelectCFGFilePanel.Title = 'Select CFG File';
            app.SelectCFGFilePanel.FontWeight = 'bold';
            app.SelectCFGFilePanel.FontSize = 14;
            app.SelectCFGFilePanel.Position = [7 374 535 59];

            % Create CFGFileLabel
            app.CFGFileLabel = uilabel(app.SelectCFGFilePanel);
            app.CFGFileLabel.FontWeight = 'bold';
            app.CFGFileLabel.Position = [19 7 55 22];
            app.CFGFileLabel.Text = 'CFG File';

            % Create CfgPath
            app.CfgPath = uieditfield(app.SelectCFGFilePanel, 'text');
            app.CfgPath.ValueChangedFcn = createCallbackFcn(app, @CfgPathValueChanged, true);
            app.CfgPath.FontSize = 10;
%            app.CfgPath.Tooltip = {'Enter full path to file or use Browse button.'};
            app.CfgPath.Position = [82 8 333 21];

            % Create CfgButton
            app.CfgButton = uibutton(app.SelectCFGFilePanel, 'push');
            app.CfgButton.ButtonPushedFcn = createCallbackFcn(app, @CfgButtonPushed, true);
            app.CfgButton.Icon = 'folder_file_icon.png';
            app.CfgButton.Position = [420 7 100 22];
            app.CfgButton.Text = 'Browse';

            % Create RecordDataPanel
            app.RecordDataPanel = uipanel(app.SetupUIFigure);
            app.RecordDataPanel.Title = 'Record Data';
            app.RecordDataPanel.FontWeight = 'bold';
            app.RecordDataPanel.FontSize = 14;
            app.RecordDataPanel.Position = [7 255 535 103];

            % Create SaveDirectoryLabel
            app.SaveDirectoryLabel = uilabel(app.RecordDataPanel);
            app.SaveDirectoryLabel.FontWeight = 'bold';
            app.SaveDirectoryLabel.Position = [18 32 90 22];
            app.SaveDirectoryLabel.Text = 'Save Directory';

            % Create SavePath
            app.SavePath = uieditfield(app.RecordDataPanel, 'text');
            app.SavePath.Editable = 'off';
            app.SavePath.FontSize = 10;
%            app.SavePath.Tooltip = {'Browse or leave blank for current directory.'};
            app.SavePath.Position = [113 32 302 22];

            % Create LogButton
            app.LogButton = uibutton(app.RecordDataPanel, 'push');
            app.LogButton.ButtonPushedFcn = createCallbackFcn(app, @LogButtonPushed, true);
            app.LogButton.Icon = 'foldericon.png';
            app.LogButton.Position = [420 32 98 22];
            app.LogButton.Text = 'Browse';

            % Create EnableRecord
            app.EnableRecord = uicheckbox(app.RecordDataPanel);
            app.EnableRecord.Text = 'Enable recording. UART stream will be saved to file.';
            app.EnableRecord.Position = [15 53 303 22];
            app.EnableRecord.Value = true;

            % Create FileNameLabel
            app.FileNameLabel = uilabel(app.RecordDataPanel);
            app.FileNameLabel.FontWeight = 'bold';
            app.FileNameLabel.Position = [19 5 62 22];
            app.FileNameLabel.Text = 'File Name';

            % Create SaveName
            app.SaveName = uieditfield(app.RecordDataPanel, 'text');
            app.SaveName.FontSize = 10;
%            app.SaveName.Tooltip = {'Enter full path to file or use Browse button.'};
            app.SaveName.Position = [114 5 270 22];
            app.SaveName.Value = 'as_demo_uart_stream.txt';

            % Create AppendtimedateCheckBox
            app.AppendtimedateCheckBox = uicheckbox(app.RecordDataPanel);
            app.AppendtimedateCheckBox.Enable = 'off';
            app.AppendtimedateCheckBox.Visible = 'off';
            app.AppendtimedateCheckBox.Text = 'Append time & date';
            app.AppendtimedateCheckBox.Position = [391 5 127 22];
            app.AppendtimedateCheckBox.Value = true;

            % Create SensorInformationButtonGroup
            app.SensorInformationButtonGroup = uibuttongroup(app.SetupUIFigure);
            app.SensorInformationButtonGroup.Title = 'Sensor Information';
            app.SensorInformationButtonGroup.FontWeight = 'bold';
            app.SensorInformationButtonGroup.FontSize = 14;
            app.SensorInformationButtonGroup.Position = [7 147 265 94];

            % Create SensorMountingHeightmEditFieldLabel
            app.SensorMountingHeightmEditFieldLabel = uilabel(app.SensorInformationButtonGroup);
            app.SensorMountingHeightmEditFieldLabel.HorizontalAlignment = 'right';
            app.SensorMountingHeightmEditFieldLabel.Position = [11 38 158 22];
            app.SensorMountingHeightmEditFieldLabel.Text = 'Sensor Mounting Height [m]:';

            % Create MountingHeight
            app.MountingHeight = uieditfield(app.SensorInformationButtonGroup, 'numeric');
            app.MountingHeight.LowerLimitInclusive = 'off';
            app.MountingHeight.Limits = [0 Inf];
            app.MountingHeight.HorizontalAlignment = 'center';
%            app.MountingHeight.Tooltip = {'Height of the sensor from the ground'};
            app.MountingHeight.Position = [180 38 41 22];
            app.MountingHeight.Value = 2;

            % Create SensorElevationTiltdegEditFieldLabel
            app.SensorElevationTiltdegEditFieldLabel = uilabel(app.SensorInformationButtonGroup);
            app.SensorElevationTiltdegEditFieldLabel.Position = [18 9 148 22];
            app.SensorElevationTiltdegEditFieldLabel.Text = 'Sensor Elevation Tilt [deg]:';

            % Create ElevationTilt
            app.ElevationTilt = uieditfield(app.SensorInformationButtonGroup, 'numeric');
            app.ElevationTilt.Limits = [-90 90];
            app.ElevationTilt.HorizontalAlignment = 'center';
%            app.ElevationTilt.Tooltip = {'Rotation about X-axis. + is upwards. - is down tilt. Valid values: -90 to 90'};
            app.ElevationTilt.Position = [181 9 41 22];

            % Create SetupCompleteButton
            app.SetupCompleteButton = uibutton(app.SetupUIFigure, 'push');
            app.SetupCompleteButton.ButtonPushedFcn = createCallbackFcn(app, @SetupCompleteButtonPushed, true);
            app.SetupCompleteButton.Position = [7 11 105 41];
            app.SetupCompleteButton.Text = 'Setup Complete';

            % Create VisualizerOptionsButtonGroup
            app.VisualizerOptionsButtonGroup = uibuttongroup(app.SetupUIFigure);
            app.VisualizerOptionsButtonGroup.Title = 'Visualizer Options';
            app.VisualizerOptionsButtonGroup.FontWeight = 'bold';
            app.VisualizerOptionsButtonGroup.FontSize = 14;
            app.VisualizerOptionsButtonGroup.Position = [285 62 256 179];

            % Create RangeCriticalZoneStartmLabel
            app.RangeCriticalZoneStartmLabel = uilabel(app.VisualizerOptionsButtonGroup);
            app.RangeCriticalZoneStartmLabel.Position = [24 109 164 22];
            app.RangeCriticalZoneStartmLabel.Text = 'Range Critical Zone Start [m]:';

            % Create CriticalStart
            app.CriticalStart = uieditfield(app.VisualizerOptionsButtonGroup, 'numeric');
            app.CriticalStart.UpperLimitInclusive = 'off';
            app.CriticalStart.Limits = [0 Inf];
            app.CriticalStart.RoundFractionalValues = 'on';
            app.CriticalStart.HorizontalAlignment = 'center';
%            app.CriticalStart.Tooltip = {'Radial distance from the sensor for beginning of critical (red) zone monitoring.'};
            app.CriticalStart.Position = [198 109 34 22];

            % Create RangeCriticalZoneEndmLabel
            app.RangeCriticalZoneEndmLabel = uilabel(app.VisualizerOptionsButtonGroup);
            app.RangeCriticalZoneEndmLabel.Position = [24 85 160 22];
            app.RangeCriticalZoneEndmLabel.Text = 'Range Critical Zone End [m]:';

            % Create CriticalEnd
            app.CriticalEnd = uieditfield(app.VisualizerOptionsButtonGroup, 'numeric');
            app.CriticalEnd.LowerLimitInclusive = 'off';
            app.CriticalEnd.Limits = [0 Inf];
            app.CriticalEnd.HorizontalAlignment = 'center';
%            app.CriticalEnd.Tooltip = {'Radial distance from the sensor for end of critical (red) zone monitoring.'};
            app.CriticalEnd.Position = [198 85 34 22];
            app.CriticalEnd.Value = 1;

            % Create EnableZones
            app.EnableZones = uicheckbox(app.VisualizerOptionsButtonGroup);
            app.EnableZones.ValueChangedFcn = createCallbackFcn(app, @EnableZonesValueChanged, true);
            app.EnableZones.Text = 'Show zone occupancy';
            app.EnableZones.Position = [16 130 142 22];
            app.EnableZones.Value = true;

            % Create RangeWarningZoneStartmLabel
            app.RangeWarningZoneStartmLabel = uilabel(app.VisualizerOptionsButtonGroup);
            app.RangeWarningZoneStartmLabel.Position = [24 61 171 22];
            app.RangeWarningZoneStartmLabel.Text = 'Range Warning Zone Start [m]:';

            % Create WarnStart
            app.WarnStart = uieditfield(app.VisualizerOptionsButtonGroup, 'numeric');
            app.WarnStart.UpperLimitInclusive = 'off';
            app.WarnStart.Limits = [0 Inf];
            app.WarnStart.RoundFractionalValues = 'on';
            app.WarnStart.HorizontalAlignment = 'center';
%            app.WarnStart.Tooltip = {'Radial distance from the sensor for beginning of warning (yellow) zone monitoring. Commonly equal to end of critical zone range.'};
            app.WarnStart.Position = [198 61 34 22];
            app.WarnStart.Value = 1;

            % Create RangeWarningZoneEndmLabel
            app.RangeWarningZoneEndmLabel = uilabel(app.VisualizerOptionsButtonGroup);
            app.RangeWarningZoneEndmLabel.Position = [24 37 167 22];
            app.RangeWarningZoneEndmLabel.Text = 'Range Warning Zone End [m]:';

            % Create WarnEnd
            app.WarnEnd = uieditfield(app.VisualizerOptionsButtonGroup, 'numeric');
            app.WarnEnd.LowerLimitInclusive = 'off';
            app.WarnEnd.Limits = [0 Inf];
            app.WarnEnd.HorizontalAlignment = 'center';
%            app.WarnEnd.Tooltip = {'Radial distance from the sensor for end of warning (yellow) zone monitoring. '};
            app.WarnEnd.Position = [198 37 34 22];
            app.WarnEnd.Value = 4;

            % Create ProjectionTimesecLabel
            app.ProjectionTimesecLabel = uilabel(app.VisualizerOptionsButtonGroup);
            app.ProjectionTimesecLabel.Position = [24 6 120 22];
            app.ProjectionTimesecLabel.Text = '[Projection Time sec:]';

            % Create ProjTime
            app.ProjTime = uieditfield(app.VisualizerOptionsButtonGroup, 'numeric');
            app.ProjTime.LowerLimitInclusive = 'off';
            app.ProjTime.Limits = [0 Inf];
            app.ProjTime.HorizontalAlignment = 'center';
%            app.ProjTime.Tooltip = {'Used to determine if a tracked object is projected to be in the warning zone within _ seconds.'};
            app.ProjTime.Position = [198 6 34 22];
            app.ProjTime.Value = 2;

%             % Create RightPanel
%             app.RightPanel = uipanel(app.GridLayout);
%             app.RightPanel.Layout.Row = 1;
%             app.RightPanel.Layout.Column = 2;

            % Show the figure after all components are created
            app.SetupUIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = setup_as_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.SetupUIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.SetupUIFigure)
        end
    end
end