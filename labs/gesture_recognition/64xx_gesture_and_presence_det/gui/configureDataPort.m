function [sphandle] = configureDataPort(num_COM, buffer_size)
    comportnum_str=['COM' num2str(num_COM)];
    
    % delete serial object if already connected    
    connectedPorts = instrfind('Type','serial');
    if(~isempty(connectedPorts))
        for i=1:numel(connectedPorts)
            if(strcmp(connectedPorts.Name(i), ['Serial-' comportnum_str]))
                delete(connectedPorts(i))
            end
        end
    end

    % connect and then open
    sphandle = serial(comportnum_str,'BaudRate',921600);
    set(sphandle,'InputBufferSize', buffer_size);
    set(sphandle,'Timeout',10);
       
    fopen(sphandle);
return