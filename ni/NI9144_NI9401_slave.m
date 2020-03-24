%
% Master 0, Slave 0, "NI 9144"
%
function rv = NI9144_NI9401_slave()

% Slave configuration

rv.SlaveConfig.vendor = 505;
rv.SlaveConfig.product = hex2dec('000023b8');
rv.SlaveConfig.description = 'NI 9144';
rv.SlaveConfig.sm = { ...
    {0, 0, {
        }}, ...
    {1, 1, {
        }}, ...
    {2, 0, {
        {hex2dec('1600'), [
            hex2dec('7000'), hex2dec('01'),   8; ...
            ]}, ...
        }}, ...
    {3, 1, {
        {hex2dec('1a00'), [
            hex2dec('6000'), hex2dec('01'),   8; ...
            ]}, ...
        }}, ...
    };

% Port configuration

rv.PortConfig.input(1).pdo = [2, 0, 0, 0];
rv.PortConfig.input(1).pdo_data_type = 1001;
rv.PortConfig.input(2).pdo = [2, 0, 0, 1];
rv.PortConfig.input(2).pdo_data_type = 1001;
rv.PortConfig.input(3).pdo = [2, 0, 0, 2];
rv.PortConfig.input(3).pdo_data_type = 1001;
rv.PortConfig.input(4).pdo = [2, 0, 0, 3];
rv.PortConfig.input(4).pdo_data_type = 1001;
rv.PortConfig.input(5).pdo = [2, 0, 0, 4];
rv.PortConfig.input(5).pdo_data_type = 1001;
rv.PortConfig.input(6).pdo = [2, 0, 0, 5];
rv.PortConfig.input(6).pdo_data_type = 1001;
rv.PortConfig.input(7).pdo = [2, 0, 0, 6];
rv.PortConfig.input(7).pdo_data_type = 1001;
rv.PortConfig.input(8).pdo = [2, 0, 0, 7];
rv.PortConfig.input(8).pdo_data_type = 1001;


rv.PortConfig.output(1).pdo = [3, 0, 0, 0];
rv.PortConfig.output(1).pdo_data_type = 1001;
rv.PortConfig.output(2).pdo = [3, 0, 0, 1];
rv.PortConfig.output(2).pdo_data_type = 1001;
rv.PortConfig.output(3).pdo = [3, 0, 0, 2];
rv.PortConfig.output(3).pdo_data_type = 1001;
rv.PortConfig.output(4).pdo = [3, 0, 0, 3];
rv.PortConfig.output(4).pdo_data_type = 1001;
rv.PortConfig.output(5).pdo = [3, 0, 0, 4];
rv.PortConfig.output(5).pdo_data_type = 1001;
rv.PortConfig.output(6).pdo = [3, 0, 0, 5];
rv.PortConfig.output(6).pdo_data_type = 1001;
rv.PortConfig.output(7).pdo = [3, 0, 0, 6];
rv.PortConfig.output(7).pdo_data_type = 1001;
rv.PortConfig.output(8).pdo = [3, 0, 0, 7];
rv.PortConfig.output(8).pdo_data_type = 1001;


% Index, SubIndex, type, Value.
rv.SlaveConfig.sdo = { hex2dec('5fff'),0,32,1; %Erstes Steckmodul, in diesem Fall NI9401, zum Konfigurieren auswählen.
                       hex2dec('2001'),0,32,1; %Wert ist hier "0b01". Um Ein- und Ausgänge festzulegen ergibt sich der Wert 0bAB, wobei B die ersten 4 DIOs, A di letzten 4 DIOs festlegt. 1: Als Ausgang, 0: Als Eingang schalten.
                        };
                
                    
% Gute Einstellung für DC. ggf. einkommentieren. Für 1 kHz
%rv.SlaveConfig.dc =  [hex2dec('300'), 1000000, 0,500000, 0, 0,0, 1,0, 0];

end
