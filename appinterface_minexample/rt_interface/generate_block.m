% Erstelle den Block zur Kommunikation mit der ROS-Seite des App-Interface
% Dieser Block muss jedes Mal neu generiert werden, wenn sich der Bus SL_IN
% oder SL_out ändert.
% Der Ordner rt_interface muss im Matlab-Pfad sein.
% Ein Ersetzen des Blocks AppInterface in der Library ist nicht notwendig.
% Im Simulink-Modell wird nur der hier erzeugte Code aufgerufen (libros_sl_interface.so, SL_func.h,SL_func_dummy.cpp).

% Lucas Jürgens (BA), lucas.juergens@zubox.de, 2017-02
% Moritz Schappler, schappler@irt.uni-hannover.de
% (c) Institut für Regelungstechnik, Universität Hannover

[folder, name, ext] = fileparts(which(mfilename));
builddir = fullfile(folder, 'build'); % build-Ordner im selben Verzeichnis wie dieses Skript
mkdir(builddir);
cd(builddir);

try
    def = legacy_code('initialize')
    def.SourceFiles = {'../rt_interface/ros_rt_core/SL_func_dummy.cpp'};   % Die Angabe dieser Datei ist trotz der shared Library nötig, da diese ja nur mit ARM kompatibel ist.
    def.HeaderFiles = {'SL_func.h'};
    def.IncPaths = {'../rt_interface/ros_rt_core'};
    def.TargetLibFiles = {'libros_sl_interface.so'};
    def.LibPaths = {'../build'};

    def.SFunctionName = 'rt_interface';
    
    def.StartFcnSpec = 'void SL_start_func()';
    def.OutputFcnSpec = 'void SL_io_func(SL_OUT_type u1[1], SL_IN_type y1[1])';
    def.TerminateFcnSpec = 'void SL_terminate_func()';

    legacy_code('sfcn_cmex_generate', def);
    legacy_code('compile', def, '-DDUMMY');    % Erstellt MEX-File. Ist leider nötig
    legacy_code('sfcn_tlc_generate', def);
    legacy_code('rtwmakecfg_generate', def);
    legacy_code('slblock_generate', def);
    
catch ME
    cd(folder);
    rethrow(ME)
end

cd(folder);
