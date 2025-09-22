function slave = EL7221(MOO)
    
    slave = struct();
    slave.SlaveConfig.vendor = 2;          % Verkäufer ID (2=Beckhoff), Liste unter https://www.ethercat.org/de/vendor_id_list.html
    slave.SlaveConfig.product = hex2dec('1c353052'); % EL7221-9014: '1c353052'
    % Product code des EtherCAT-Slaves auslesen (Index 0x1018:02)
    slave.SlaveConfig.description = 'EL7221-9014';
    
    % SyncManager (SM)
    % Cyclic synchronous position mode
    slave.SlaveConfig.sm = { ...
        {0, 0, {}}, ...
        {1, 1, {}}, ...
        {2, 0, {
            {hex2dec('1600'), [hex2dec('7010'), hex2dec('01'), 16;]}, ... %Controlword
            {}, ...     % inputs (target values) are configured specific to op-mode
        }}, ...   
        {3, 1, {
        {hex2dec('1a00'), [hex2dec('6000'), hex2dec('11'),  32; ]}, ...%Current Position
        {hex2dec('1a01'), [hex2dec('6010'), hex2dec('01'),  16; ]}, ...%Statusword
        {hex2dec('1a02'), [hex2dec('6010'), hex2dec('07'),  32; ]},...%Current Velocity
        {hex2dec('1a03'), [hex2dec('6010'), hex2dec('08'),  16; ]}...%Current Torque 
        }}, ...
        };
    
    % Input ports configuration
    slave.PortConfig.input(1).pdo = [2, 0, 0, 0];      % Controlword
    slave.PortConfig.input(1).pdo_data_type = 1016;
    
    %Output ports configuration
    slave.PortConfig.output(1).pdo = [3, 1, 0, 0];%Statusword
    slave.PortConfig.output(1).pdo_data_type = 1016;%2Byte

    slave.PortConfig.output(2).pdo = [3, 0, 0, 0];%Current Position
    slave.PortConfig.output(2).pdo_data_type = 2032;%4Byte

    slave.PortConfig.output(3).pdo = [3, 2, 0, 0];%Current Velocity
    slave.PortConfig.output(3).pdo_data_type = 2032;%4Byte

    slave.PortConfig.output(4).pdo = [3, 3, 0, 0];%Current Torque
    slave.PortConfig.output(4).pdo_data_type = 2016;%2Byte
    
    % SDO-Config Mode of Operation
    slave.SlaveConfig.sdo = {hex2dec('7010'), hex2dec('03'), 8,MOO;};
    
    % Distributed Clocks (DC)
    slave.SlaveConfig.dc = [hex2dec('700'),...
        0, 1, 30000, 1, 0, ...
        0, -1, 1000, 1];
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % adjust configurations specific to mode of operation (MOO)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % MOO: mode of operation
    if MOO == 8
        slave = slave_el7221_csp(slave);
    elseif MOO == 9
        slave = slave_el7221_csv(slave);
    elseif MOO == 10
        slave = slave_el7221_cst(slave);
    else
        error('Slave Config');
    end

%% documentation
%%%%%%% Einführung
% Dieser Struct ist ein für das Aufsetzen eines Slaves und hat zwei Felder
% auf höchster Ebene: 
%   slave.SlaveConfig
%   slave.PortConfig 

% SlaveConfig
    % SlaveConfig beschreibt die Eigenschaften des Slaves, die fürs Einbinden
    % des Slaves durch den Master ins Netzwerk nötig sind. Diese Info steckt in
    % der EtherCATInfo xml-Datei, welche vom Hersteller bereitgestellt wird.
    % Wir lesen sie aber über den Befehl 
    % /opt/etherlab/bin/ethercat pdos --skin etherlab > ~/pdos_etherlab.m
    % aus der Anleitung https://github.com/SchapplM/etherlab-examples aus

% ethercat pdos
    % die Funktion ethercat pdos listet die SyncManagers, PDO-Zuweisung und
    % -Mapping auf. Durch den Zusatz --skin etherlab werden 
    % Konfigurationsvorlagen der Slaves generiert, die kompatibel mit dem
    % generisch EtherCAT slave block von EtherLab sind. 

% PortConfig
    % PortConfig wird verwendet, um Ein-Ausgänge eines SImulink-Blocks auf
    % Basis der SlaveConfig zu spezifizieren. Hat die Felder input und
    % output
    
% Prozessdatenobjekte (PDO)
    % Prozessdaten, die in Segmente, auch Prozessdatenobjekte, zerteilt werden.
    % PDO dienen dem schnellen Austausch von Echtzeitdaten (zB E/A-Daten, Soll-
    % oder Istwerte)
    
% +++ SyncManager +++
% SyncManager sorgen für konsisten und sicheren Datenaustausch zwischen
% EtherCAT-Master und der lokalen Applikation des SLave-Gerätes. In der
% Konfiguration des SyncManagers werden PDO und Einträge darin spezifiziert. 
% Nach der Spezifikation kann dieser PDO-Eintrag einem Input oder Output zugewiesen werden.  

% Konfiguration der SyncManagers durch slave.SlaveConfig.sm ein Cell Array
% zugeordnert wird. Dieser Cell Array hat soviele Elemente wie der slave
% SyncManagers hat, die konfiguriert werden müssen. Die Elemente des Cell
% Arrays sind ebenfalls Cell Arrays und haben je 3 Elemente:
% {SmIndex, SmDirection, SmPDO}
% - SmIndex:    Zero based index number of the <sm> xml element in the
%               EtherCATInfo xml file
%               % TODO: wo zu finden?
% - SmDirection:    direction as seen by the master: 0 = Output; 1=Input
% - SmPDO:      Cell array of PDO's that are mapped to the SyncManager

% Konfiguration der PDO-Einträge
% Das dritte Element des Cell Arrays vom jeweiligen SyncManager ist ein
% weiterer Cell Array. Jedes Element darin ist ein Cell Array und entspricht jedem einzelnen einkludierten PDO. 
% Jedes Element/Cell Array hat zwei Elemente: 
% {PDOIndex, PDOEntries}
% PDOEntries: [Nx3] array with one row for every Entry. Colums:
% 1. Entry Index
% 2. Entry SubIndex
% 3. BitLen

% Das zweite Element PDOEntries ist ein Array und listet die Einträge des
% PDO auf. Dieser numerische Array  hat Nx3 Zeilen und Spalten mit
% N=Gesamtanzahl aller Einträge. Die Spalten entsprechen 
% [EintragsIndex, EintragsSubindex, BitLen]


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%% SyncManager (SM) configuration %%%%%%%%%%%%%%
% DataTypes EL7221-9014:
% - 7010:01 (Controlword): UINT16
% - 7010:03 (Modes of OperatioN): UINT8
% - 7010:05 (Target Position): UINT32
% - 7010:06 (Target velocity): INT32
% - 7010:09 (Target torque): INT16
% - 6000:11 (Position actual value): UINT32
% - 6010:01 (Statusword): UINT16
% - 6010:07 (Velocity actual value): INT32
% - 6010:08 (torque actual value): INT16


%%%%%%% Input Ports %%%%%%%%
% % Controlword
% slave.PortConfig.input(1).pdo = [2, 0, 0, 0];
% % [SyncManagerIndex, PDOIndex, EntryIndex, ElementIndex]
% slave.PortConfig.input(1).pdo_data_type = 1016;
%
% +++ pdo_data_type 
% Ziffer 1 - 0: signed, 1: unsigned ?
% Ziffer 2 - ? (type?: int/float..)
% Ziffer 3-4 - BitLen

% distributed clocks config: 
%     Assign Activate
%     Cycle Time Sync0
%     Cycle Time Sync0 Factor
%     Shift Time Sync0
%     Shift Time Sync0 Factor
%     Shift Time Sync0 Input
%     Cycle Time Sync1
%     Cycle Time Sync1 Factor
%     Shift Time Sync1
%     Shift Time Sync1 Factor 



end

%% functions: configurations, specific to mode of operation

function slave = slave_el7221_csp(slave)
    %%% Cyclic synchronous position mode
    % SyncManager (SM): Input / target variable
    slave.SlaveConfig.sm{3}{3}{2} = {hex2dec('1606'), [
            hex2dec('7010'), hex2dec('05'),  32;]};  %Target Position
            
    % Input Port Config
    slave.PortConfig.input(2).pdo = [2, 1, 0, 0];
    slave.PortConfig.input(2).pdo_data_type = 2032;
end
function slave = slave_el7221_csv(slave)
    %%% Cyclic synchronous velocity mode
     % SyncManager (SM): Input / target variable
    slave.SlaveConfig.sm{3}{3}{2} = {hex2dec('1601'), [
            hex2dec('7010'), hex2dec('06'),  32;]};  %Target Velocity
    
    % Input Port Config
    slave.PortConfig.input(2).pdo = [2, 1, 0, 0];
    slave.PortConfig.input(2).pdo_data_type = 2032;
end
function slave = slave_el7221_cst(slave)
    %%% Cyclic synchronous torque mode
    % SyncManager (SM): Input / target variable
    slave.SlaveConfig.sm{3}{3}{2} = {hex2dec('1602'), [
            hex2dec('7010'), hex2dec('09'),  16;]};  %Target Torque
    
    % Input Port Config
    slave.PortConfig.input(2).pdo = [2, 1, 0, 0];
    slave.PortConfig.input(2).pdo_data_type = 2016;
end