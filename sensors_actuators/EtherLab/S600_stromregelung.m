% Master 0, Slave 1, "S300/S400/S600/S700"
%
function rv = S600_stromregelung()

% Slave configuration

rv.SlaveConfig.vendor = 106;
rv.SlaveConfig.product = hex2dec('03000600');
rv.SlaveConfig.description = 'S300/S400/S600/S700';
rv.SlaveConfig.sm = { ...
    {0, 0, {
        }}, ...
    {1, 1, {
        }}, ...
    {2, 0, {
        {hex2dec('1703'), [ % 1702
            hex2dec('6074'), hex2dec('00'),  16; ... % Strom-Sollwert
            hex2dec('6040'), hex2dec('00'),  16; ... % Steuerwort
            ]}, ...
        }}, ...
    {3, 1, {
        {hex2dec('1b03'), [ % 1b03
            hex2dec('6064'), hex2dec('00'),  32; ... % Lage-Istwert in Inkrementen
            hex2dec('6077'), hex2dec('00'),  16; ... % Strom-Istwert
            hex2dec('6041'), hex2dec('00'),  16; ... % Statuswort
%             hex2dec('606c'), hex2dec('00'),  32; ... % Geschwindigkeits-Istwert            
            ]}, ...
        }}, ...
    };

% Port configuration

rv.PortConfig.input(1).pdo = [2, 0, 0, 0];
rv.PortConfig.input(1).pdo_data_type = 2016; % Strom-Sollwert

rv.PortConfig.input(2).pdo = [2, 0, 1, 0];
rv.PortConfig.input(2).pdo_data_type = 1016; % Steuerwort

rv.PortConfig.output(1).pdo = [3, 0, 0, 0];
rv.PortConfig.output(1).pdo_data_type = 2032; % Lage-Istwert in Inkrementen

rv.PortConfig.output(2).pdo = [3, 0, 1, 0];
rv.PortConfig.output(2).pdo_data_type = 2016; % Strom-Istwert

rv.PortConfig.output(3).pdo = [3, 0, 2, 0];
rv.PortConfig.output(3).pdo_data_type = 1016; % Statuswort


% Aus der XML-Datei der Klemme S600

% -<OpMode>
    % <Name>DcSync</Name>
    % <Desc>DC for synchronization</Desc>
    % <AssignActivate>#x0300</AssignActivate>
    % <CycleTimeSync0 Factor="1">0</CycleTimeSync0>
    % <ShiftTimeSync0>0</ShiftTimeSync0>
    % -<Sm No="2">
        % <SyncType>2</SyncType>
        % <CycleTime Factor="1">0</CycleTime>
        % <ShiftTime MinAfterSync="0">0</ShiftTime>
    % </Sm>
    % -<Sm No="3">
        % <SyncType>2</SyncType>
        % <CycleTime Factor="1">0</CycleTime>
        % <ShiftTime MinBeforeFrame="100">0</ShiftTime>
    % </Sm>
% </OpMode>

% rv.SlaveConfig.dc = [hex2dec('300'),... %AssignActivate
%     0, ... % Cycle Time Sync0
%     1, ... % Cycle Time Sync0 Factor
%     30000, ... % Shift Time Sync0
%     1, ... % Shift Time Sync0 Factor
%     0, ... % Shift Time Sync0 Input
%     0, ... % Cycle Time Sync1
%     -1, ... % Cycle Time Sync1 Factor
%     1000, ... %  Shift Time Sync1
%     1]; % Shift Time Sync1 Factor



end
%

