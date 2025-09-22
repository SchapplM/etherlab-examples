
% Master 0, Slave 1, "EPOS4"
%
function rv = Maxon()

% Slave configuration

rv.SlaveConfig.vendor = 251;
rv.SlaveConfig.product = hex2dec('63500000');
rv.SlaveConfig.description = 'EPOS4';
rv.SlaveConfig.sm = { ...
    {0, 0, {
        }}, ...
    {1, 1, {
        }}, ...
    {2, 0, {
        {hex2dec('1600'), [
            hex2dec('6040'), hex2dec('00'),  16; ... % Controlword
            hex2dec('60FF'), hex2dec('00'),  32; ... % target position
            hex2dec('6060'), hex2dec('00'),  8; ... % target position
            hex2dec('3182'), hex2dec('01'),  32; ... % Analog output 1 - Einheit in [mV]
            hex2dec('3182'), hex2dec('02'),  32; ... % Analog output 2 - Einheit in [mV]           
            ]}, ...
        }}, ...
    {3, 1, {
        {hex2dec('1a00'), [
            hex2dec('6041'), hex2dec('00'),  16; ... % Statusword 
            hex2dec('6064'), hex2dec('00'),  32; ... % position actual value   
            hex2dec('606C'), hex2dec('00'),  32;  ... % velocity actual value
            hex2dec('3160'), hex2dec('01'),  16;  ... % analog input
            hex2dec('6061'), hex2dec('00'),  8;  ... % mode of operation
            ]}, ...
        }}, ...
    };

% Port configuration

rv.PortConfig.input(1).pdo = [2, 0, 0, 0];
rv.PortConfig.input(1).pdo_data_type = 1016;

rv.PortConfig.input(2).pdo = [2, 0, 1, 0];
rv.PortConfig.input(2).pdo_data_type = 2032;

rv.PortConfig.input(3).pdo = [2, 0, 2, 0];
rv.PortConfig.input(3).pdo_data_type = 2008;

rv.PortConfig.input(4).pdo = [2, 0, 3, 0];
rv.PortConfig.input(4).pdo_data_type = 2032;

rv.PortConfig.input(5).pdo = [2, 0, 4, 0];
rv.PortConfig.input(5).pdo_data_type = 2032;

rv.PortConfig.output(1).pdo = [3, 0, 0, 0];
rv.PortConfig.output(1).pdo_data_type = 1016;

rv.PortConfig.output(2).pdo = [3, 0, 1, 0];
rv.PortConfig.output(2).pdo_data_type = 2032;

rv.PortConfig.output(3).pdo = [3, 0, 2, 0];
rv.PortConfig.output(3).pdo_data_type = 2032;

rv.PortConfig.output(4).pdo = [3, 0, 3, 0];
rv.PortConfig.output(4).pdo_data_type = 2016;

rv.PortConfig.output(5).pdo = [3, 0, 4, 0];
rv.PortConfig.output(5).pdo_data_type = 2008;

%rv.SlaveConfig.sdo = { hex2dec('6076'),hex2dec('00'),32,0.054*10^6 };

end
