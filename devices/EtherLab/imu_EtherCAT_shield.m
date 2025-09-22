function rv = imu_EtherCAT_shield()

% Slave configuration

rv.SlaveConfig.vendor = 1946;                               
rv.SlaveConfig.product = hex2dec('1234abcd');               
rv.SlaveConfig.description = 'imu to ethercat';             
rv.SlaveConfig.sm = { ...                                   
    
    {1, 1, {
        {hex2dec('1A00'), [
            hex2dec('0006'), hex2dec('01'),   32; ...
            hex2dec('0006'), hex2dec('02'),   32; ...
            hex2dec('0006'), hex2dec('03'),   32; ...
            hex2dec('0006'), hex2dec('04'),   32; ...
            hex2dec('0006'), hex2dec('05'),   32; ...
            hex2dec('0006'), hex2dec('06'),   32; ...
            hex2dec('0006'), hex2dec('07'),   32; ...
            ]}, ...
        }}, ...
    };

% Port configuration

rv.PortConfig.output(1).pdo = [0, 0, 0, 0];
rv.PortConfig.output(1).pdo_data_type = 3032;

rv.PortConfig.output(2).pdo = [0, 0, 1, 0];
rv.PortConfig.output(2).pdo_data_type = 3032;

rv.PortConfig.output(3).pdo = [0, 0, 2, 0];
rv.PortConfig.output(3).pdo_data_type = 3032;

rv.PortConfig.output(4).pdo = [0, 0, 3, 0];
rv.PortConfig.output(4).pdo_data_type = 3032;

rv.PortConfig.output(5).pdo = [0, 0, 4, 0];
rv.PortConfig.output(5).pdo_data_type = 3032;

rv.PortConfig.output(6).pdo = [0, 0, 5, 0];
rv.PortConfig.output(6).pdo_data_type = 3032;

rv.PortConfig.output(7).pdo = [0, 0, 6, 0];
rv.PortConfig.output(7).pdo_data_type = 1032;


end
