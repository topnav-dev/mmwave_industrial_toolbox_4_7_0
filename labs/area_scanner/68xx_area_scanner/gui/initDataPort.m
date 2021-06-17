function [hDataPort] = initDataPort(num_COM)
    comportnum_str=['COM' num2str(num_COM)];
    
    % delete serial object if already connected    
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

    % connect and then open
    hDataPort = serial(comportnum_str,'BaudRate',921600);
    try 
        set(hDataPort,'InputBufferSize', 2^16);
        set(hDataPort,'Timeout',2); 
        fopen(hDataPort);        
        fprintf([comportnum_str ' opened. \n']);
    catch ME
        fprintf(['Error: ' comportnum_str ' could not be opened! \n']);
        delete(hDataPort);
        hDataPort = -1;
    end
    
return