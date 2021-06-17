function [hCfgPort] = initCfgPort(num_COM)
    comportnum_str=['COM' num2str(num_COM)];
    
    % delete port if already connected
    connectedPorts = instrfind('Type','serial');
    if(~isempty(connectedPorts))
        numPorts = size(connectedPorts,2);
        if(numPorts == 1)
            if(strcmp(connectedPorts.Name, ['Serial-' comportnum_str]))
                delete(connectedPorts)
            end
        else
            indx = 1;
            while(indx <=size(connectedPorts,2))
                if(strcmp(connectedPorts.Name(indx), ['Serial-' comportnum_str]))
                    delete(connectedPorts(indx))
                    connectedPorts = instrfind('Type','serial');
                else
                    indx = indx+1;
                end
            end
        end
    end
    
    % connect to port
    hCfgPort = serial(comportnum_str,'BaudRate',115200);   
    try 
        set(hCfgPort,'Parity','none')    
        set(hCfgPort,'Terminator','LF') 
        fopen(hCfgPort); 
        fprintf([comportnum_str ' opened. \n']);
    catch
        fprintf(['Error: ' comportnum_str ' could not be opened! \n']);
        delete(hCfgPort);
        hCfgPort = -1;
    end
return