function [P] = parseCLICommands2Struct(cliCfg, supported_cfgs)
fprintf('Parsing configuration file...\n')
inputCfg = struct('command', [], 'parameters', [], 'units', []);
indxInputCfg = 1;
P = struct();
    for k=1:length(cliCfg)

        % C holds all the command and input parameters in cell array of strings format
        C = strsplit(cliCfg{k});
        % First index is CLI command - all others are input parameters
        commandString = C{1};
        isDuplicateCommand = isfield(P, commandString);

        if(~strcmp(commandString, '%') && ~strcmp(commandString, ' ')) %skip comments and empty return lines
            indxCfgCommand = find((strcmp({supported_cfgs.command}, commandString)));
            if(indxCfgCommand) %check if command is supported command

                % get the expected input parameters for the command
                paramFieldnames =  fieldnames(supported_cfgs(indxCfgCommand).parameters);
                numReqInputParams = numel(paramFieldnames);

                if(numReqInputParams == numel(C)-1) %check if num input parameters entered match expected 
                    % add command to array
                    inputCfg(indxInputCfg) = supported_cfgs(indxCfgCommand);
                    % assign input parameter values
                    for i=1:numReqInputParams                    
                        inputCfg(indxInputCfg).parameters.(paramFieldnames{i}) = str2double(C{i+1});
                        if(~isDuplicateCommand)
                            P.(commandString).(paramFieldnames{i})= str2double(C{i+1});
                        else
                            P.(commandString).(paramFieldnames{i}) = [P.(commandString).(paramFieldnames{i}) str2double(C{i+1})];
                        end
                    end
                    indxInputCfg = indxInputCfg+1;
                else
                    fprintf('Error: number of parameters is incorrect for %s command', commandString);
                end
            else
                if(~isempty(commandString))
                    %fprintf('Skip parsing for: %s command\n', commandString);
                end
            end
        end

    end
end