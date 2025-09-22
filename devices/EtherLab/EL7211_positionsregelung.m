
function rv = EL7211_positionsregelung()

% Slave configuration

rv.SlaveConfig.vendor = 2;
rv.SlaveConfig.product = hex2dec('1c2b3052');
rv.SlaveConfig.description = 'EL7211-0010';
rv.SlaveConfig.sm = { ...
    {0, 0, {
        }}, ...
    {1, 1, {
        }}, ...
    {2, 0, {
        {hex2dec('1600'), [
            hex2dec('7010'), hex2dec('01'),  16; ... %Controlword
            ]}, ...
        {hex2dec('1606'), [
            hex2dec('7010'), hex2dec('05'),  32; ... %Target Position
            ]}, ...
        }}, ...
    {3, 1, {
        {hex2dec('1a00'), [hex2dec('6000'), hex2dec('11'),  32; ]}, ...%Current Position
        {hex2dec('1a01'), [hex2dec('6010'), hex2dec('01'),  16; ]}, ...%Statusword
        {hex2dec('1a02'), [hex2dec('6010'), hex2dec('07'),  32; ]},...%Current Velocity
        {hex2dec('1a03'), [hex2dec('6010'), hex2dec('08'),  16; ]}...%Current Torque 
        }}, ...
    };

% Port configuration

rv.PortConfig.input(1).pdo = [2, 0, 0, 0];
rv.PortConfig.input(1).pdo_data_type = 1016;

rv.PortConfig.input(2).pdo = [2, 1, 0, 0];
rv.PortConfig.input(2).pdo_data_type = 2032;

%Output
rv.PortConfig.output(1).pdo = [3, 0, 0, 0];%Current Position
rv.PortConfig.output(1).pdo_data_type = 2032;%4Byte

rv.PortConfig.output(2).pdo = [3, 1, 0, 0];%Statusword
rv.PortConfig.output(2).pdo_data_type = 1016;%2Byte

rv.PortConfig.output(3).pdo = [3, 2, 0, 0];%Current Velocity
rv.PortConfig.output(3).pdo_data_type = 2032;%4Byte

rv.PortConfig.output(4).pdo = [3, 3, 0, 0];%Current Torque
rv.PortConfig.output(4).pdo_data_type = 2016;%2Byte


rv.SlaveConfig.dc = [hex2dec('700'),...
    0, 1, 30000, 1, 0, ...
    0, -1, 1000, 1];



end