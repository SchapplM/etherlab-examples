% Erster Teil automatisch generiert. Kommentiert.
%
% Master 0, Slave 0
%
function rv = etcio100_slave()

% Slave configuration

rv.SlaveConfig.vendor = 4;
rv.SlaveConfig.product = hex2dec('00000006');
rv.SlaveConfig.description = '';
rv.SlaveConfig.sm = { ...
    {0, 0, {
        }}, ...
    {1, 1, {
        }}, ...
    {2, 0, {
        {hex2dec('1600'), [
            hex2dec('6200'), hex2dec('01'),   8; ...	% Digitale Ausgänge
            ]}, ...
        {hex2dec('1601'), [
            hex2dec('6411'), hex2dec('01'),  16; ...	% ANAOUT1 12 bit
            hex2dec('6411'), hex2dec('02'),  16; ...	% ANAOUT2 12 bit
            ]}, ...
        }}, ...
    {3, 1, {
        {hex2dec('1a00'), [
            hex2dec('6000'), hex2dec('01'),   8; ...	
            ]}, ...
        {hex2dec('1a01'), [
            hex2dec('6401'), hex2dec('01'),  16; ...	% ANAIN1 12 bit
            hex2dec('6401'), hex2dec('02'),  16; ... 	% ANAIN2 12 bit
            ]}, ...
        }}, ...
    };

% Port configuration

% Digitale Ausgänge
rv.PortConfig.input(1).pdo = [2, 0, 0, 0];
rv.PortConfig.input(1).pdo_data_type = 1001;
rv.PortConfig.input(2).pdo = [2, 0, 0, 1];
rv.PortConfig.input(2).pdo_data_type = 1001;
rv.PortConfig.input(3).pdo = [2, 0, 0, 2];
rv.PortConfig.input(3).pdo_data_type = 1001;
rv.PortConfig.input(4).pdo = [2, 0, 0, 3];
rv.PortConfig.input(4).pdo_data_type = 1001;


rv.PortConfig.input(5).pdo = [2, 1, 0, 0];	% ANAOUT1 12 bit
rv.PortConfig.input(5).pdo_data_type = 1016;

rv.PortConfig.input(6).pdo = [2, 1, 1, 0];	% ANAOUT2 12 bit
rv.PortConfig.input(6).pdo_data_type = 1016;

% Digitale Eingänge
rv.PortConfig.output(1).pdo = [3, 0, 0, 0];
rv.PortConfig.output(1).pdo_data_type = 1001;
rv.PortConfig.output(2).pdo = [3, 0, 0, 0];
rv.PortConfig.output(2).pdo_data_type = 1001;	
rv.PortConfig.output(3).pdo = [3, 0, 0, 0];
rv.PortConfig.output(3).pdo_data_type = 1001;	
rv.PortConfig.output(4).pdo = [3, 0, 0, 0];
rv.PortConfig.output(4).pdo_data_type = 1001;	
rv.PortConfig.output(5).pdo = [3, 0, 0, 0];
rv.PortConfig.output(5).pdo_data_type = 1001;
rv.PortConfig.output(6).pdo = [3, 0, 0, 0];
rv.PortConfig.output(6).pdo_data_type = 1001;


rv.PortConfig.output(7).pdo = [3, 1, 0, 0];	% ANAIN1 12 bit
rv.PortConfig.output(7).pdo_data_type = 1016;	

rv.PortConfig.output(8).pdo = [3, 1, 1, 0];	% ANAIN2 12 bit
rv.PortConfig.output(8).pdo_data_type = 1016;	

% --- manuelle Ergänzungen

rv.SlaveConfig.sdo = { hex2dec('2006'),1,8,1;   % USER LED 1
                       hex2dec('2006'),2,8,1;
                       hex2dec('2000'),1,8,6;   % Number of digital Inputs
                       hex2dec('2000'),2,8,1;   % DI1 Debouncing time Value * 10 µs
                       hex2dec('2000'),3,8,1;   %
                       hex2dec('2000'),4,8,1;   %
                       hex2dec('2000'),5,8,1;   %
                       hex2dec('2000'),6,8,1;   %
                       hex2dec('2000'),7,8,1;   %
                       hex2dec('2001'),1,8,4;   % Number of digital Outputs
                       hex2dec('2001'),2,8,0;   % DO1 Default-Value
                       hex2dec('2001'),3,8,0;   %
                       hex2dec('2001'),4,8,0;   %
                       hex2dec('2001'),5,8,0;   %
                       hex2dec('2002'),1,8,2;   % Number of analog Inputs
                       hex2dec('2003'),1,8,2;   % Number of analog Outputs
                       hex2dec('2003'),2,8,0;   % A01 Default-Value
                       hex2dec('2003'),3,8,0;   %

                        
                        
                        }; % USER LED 2

end
