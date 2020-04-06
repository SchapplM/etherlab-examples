# Anleitung zum Aufsetzen eines Linux-Echtzeit-PCs

Im Folgenden wird beschrieben, wie ein Linux-Echtzeit-PC aufgesetzt wird.

## Autoren und Förderung

Diese Anleitung ist am Institut für Regelungstechnik, Leibniz Universität Hannover unter Mitarbeit von Johannes Kühn, Alexander Tödtheide, Lars Johannsmeier und Moritz Schappler entstanden (2015 bis 2017).
Die Autoren erhielten Förderung durch die Europäische Union (7. Forschungsrahmenprogramm Horizon 2020, Förder-Nr. 688857, "SoftPro") und das Bundesministerium für Bildung und Forschung (Förder-Nr. 16SV6175, "3. Arm").

(C) Institut für Regelungstechnik, Leibniz Universität Hannover

## Syntax

* Befehle auf Entwicklungsrechner (§)
* Normale Befehle auf Echtzeitrechner ($)
* Root-Befehle auf Echtzeitrechner (#)

## Linux-Distribution auswählen

Folgende Kriterien spielen bei der Auswahl des Linux-Kernels eine Rolle:
* Der Kernel muss mit RT-Patch unterstützt sein: https://www.kernel.org/pub/linux/kernel/projects/rt
* Optional: Sollte langzeit-unterstützt sein: https://de.wikipedia.org/wiki/Linux_(Kernel)#Versionen_mit_Langzeitunterst.C3.BCtzung

Folgende Kriterien spielen bei der Auswahl der Linux-Distribution eine Rolle:
* Welche Kernel-Versionen werden unterstützt?
* Distribution sollte langzeituntstützt sein
* Soll ROS unterstützt werden? (z.B. für RT-Interface)
  * Ubuntu 14.04 (Trusty) für ROS Indigo, Ubuntu 16.04 (Xenial) für ROS Kinetic, Ubuntu 18.04 (Bionic) für ROS Melodic
  
Siehe auch:
* https://wiki.ubuntu.com/Kernel/Support

Erprobt wurden bisher folgende Konfigurationen:
* LUbuntu 18.04 mit Kernel 4.19 (LUbuntu ist eine leichte Ubuntu-Variante)
* LUbuntu 16.04.1 mit Kernel 4.4
* LUbuntu 14.04.5 mit Kernel 4.4

## Distribution installieren

Allgemein: Installation von Live-USB-Stick.
Kopiere iso-Image auf USB-Stick. Am einfachstenunter Ubuntu mit dem "Startmedienersteller"

### Ubuntu

Herunterladen über offizielle Internetseite von Ubuntu

### LUbuntu

Die Alternate-Variante hat eine besonders einfache Desktop-Umgebung. Die Installation von ROS ist trotzdem möglich.

* LUbuntu 18.04: [lubuntu-18.04-alternate-amd64.iso](http://cdimage.ubuntu.com/lubuntu/releases/18.04/release/)
* LUbuntu 16.04: [lubuntu-16.04.1-alternate-amd64.iso](http://cdimage.ubuntu.com/lubuntu/releases/16.04/release/)

### Kernel-Version der Linux-Distribution aktualisieren

manche Distributionen kommen bereits mit einer Kernel-Version, die RT-unterstützt ist (z.B. LUbuntu 16.04.1 mit Kernel 4.4). Bei anderen ist dies nicht der Fall (z.B. LUbuntu 18.04 mit Kernel 4.15).

Modifiziere die [Anleitung zur Installation des Kernels 4.19](http://ubuntuhandbook.org/index.php/2018/10/linux-kernel-4-19-released-install-ubuntu/):

Nach Recherche des aktuellen Kernels: Hier z.B. 4.19.106, weil 4.19.106-rt45 die aktuellste RT-Patch-Version ist.

Passe die Download-Liste an: Wähle "generic", weil durch den RT-Patch bessere Eigenschaften als bei der Variante "lowlatency" erreicht werden.

    $mkdir -p /tmp/kernel419
    $cd /tmp/kernel419
    $wget -c https://kernel.ubuntu.com/~kernel-ppa/mainline/v4.19.106/linux-headers-4.19.106-0419106_4.19.106-0419106.202002240333_all.deb
    $wget -c https://kernel.ubuntu.com/~kernel-ppa/mainline/v4.19.106/linux-headers-4.19.106-0419106-generic_4.19.106-0419106.202002240333_amd64.deb
    $wget -c https://kernel.ubuntu.com/~kernel-ppa/mainline/v4.19.106/linux-image-unsigned-4.19.106-0419106-generic_4.19.106-0419106.202002240333_amd64.deb
    $wget -c https://kernel.ubuntu.com/~kernel-ppa/mainline/v4.19.106/linux-modules-4.19.106-0419106-generic_4.19.106-0419106.202002240333_amd64.deb
    $sudo dpkg -i *.deb

Anschließend sollte in diesen Kernel gebootet werden. Wenn das nicht funktioniert (aus Hardware-Inkompatibilität) braucht man gar nicht mit dem Echtzeitpatch weitermachen. Dafür neustarten und im Grub-Bootloader den Kernel auswählen.  
Tipp: Teilweise muss man beim Booten Shift (links) drücken, damit der Bootloader angezeigt wird.

## Konfiguration Teil 1

### Allgemeine Konfiguration

Benutzer anlegen (hier am Beispiel des EtherCAT-Benutzers "ec")

    $sudo adduser ec

SSH-Schlüssel für Benutzer erzeugen: https://wiki.ubuntuusers.de/SSH/#Authentifizierung-ueber-Public-Keys

    §ssh-keygen -t rsa -b 4096 -f ~/.ssh/RTPC_ec
    §ssh-copy-id -i ~/.ssh/RTPC_ec.pub ec@10.144.130.83


SSH-Profil auf Entwicklungsrechner anlegen: https://wiki.ubuntuusers.de/SSH/#ssh-config

    §nano ~/.ssh/config

Hineinkopieren (IP-Adresse dabei anpassen)

    Host RTPC_ec
        HostName 10.144.130.83
        Port 22
        User ec
        IdentityFile ~/.ssh/RTPC_ec

Optional: Paketquellen auf Uni-Server umstellen: http://mirrors.ubuntu.com/mirrors.txt  
Je nach Inhalt ist der eingestellte Server auch z.B. us.archive.ubuntu.com.

    $sudo cp /etc/apt/sources.list /etc/apt/sources.list.orig
    OLDMIRROR=http://de.archive.ubuntu.com/ubuntu/
    NEWMIRROR=http://ftp.rrzn.uni-hannover.de/pub/mirror/linux/ubuntu/
    $sudo sed -i "s|$OLDMIRROR|$NEWMIRROR|g" /etc/apt/sources.list

Programme installieren

    $sudo apt-get install openssh-server sudo make gcc vim cmake
    
## Echtzeit-Kernel installieren

Die Umgebungsvariablen `KERNEL` und `RTPATCH` beschreiben die entsprechenden Versionen und sind abhängig vom installierten System (s.o.).

Für Ubuntu 16.04.1 und Ubuntu 14.04.5 bspw folgende Werte. Diese Variablen müssen an die gewählten Versionen angepasst werden:

    KERNEL=4.4.66
    RTPATCH=4.4.66-rt79

### Konfigurieren

Abhängigkeiten dafür installieren

    sudo apt-get install bison flex

Kernel-Quellen herunterladen

    $uname -a

    $mkdir -p ~/src
    $cd ~/src
    $wget https://www.kernel.org/pub/linux/kernel/v4.x/linux-$KERNEL.tar.gz
    
    $tar xvzf linux-$KERNEL.tar.gz
    
RT-Patch. Suche in https://www.kernel.org/pub/linux/kernel/projects/rt  
Die Zahl "4.4" muss ggf. noch angepasst werden.
Herunterladen und entpacken:

    $wget https://www.kernel.org/pub/linux/kernel/projects/rt/4.4/patch-$RTPATCH.patch.gz
    $gzip -d patch-$RTPATCH.patch.gz

Patch anwenden:
http://linux-sxs.org/upgrading/patching.html

    $cd linux-$KERNEL
    $cat ../patch-$RTPATCH.patch | patch -p1
    
Konfigurieren: Option "5" wählen (Fully Preemptible Kernel (RT) (PREEMPT_RT_FULL)). Den Rest auf Standard lassen (Enter gedrückt halten)

    $cp /boot/config-$(uname -r) .config && make oldconfig

Setze parameter CONFIG_MODULE_SIG = No

    $nano .config
    
### Kompilieren

Bauen: https://wiki.ubuntuusers.de/Kernel/Kompilierung/

Abhängigkeiten installieren:

    $sudo apt-get install libssl-dev kernel-package
    
Kompilieren
    
    $sed -rie 's/echo "\+"/#echo "\+"/' scripts/setlocalversion
    $make-kpkg clean
    $CONCURRENCY_LEVEL=$(getconf _NPROCESSORS_ONLN) fakeroot make-kpkg --initrd --revision=0 kernel_image kernel_headers

### Installieren

Installation

    $sudo dpkg -i ../linux-{headers,image}-$RTPATCH_*.deb

### Standard-Booteintrag ändern (RT-Kernel soll automatisch geändert werden)

Anpassen der Werte: https://wiki.ubuntuusers.de/GRUB_2/Konfiguration/#Bedeutung-der-Variablen

    $sudo nano /etc/default/grub
    
#### Alternative 1: Standard-Boot-Eintrag speichern und einmal händisch in den RT-Kernel Booten

    GRUB_DEFAULT=saved
    GRUB_SAVEDEFAULT=true
    
Übernehmen

    $sudo update-grub
    
Anschließend kann in den RT-Kernel geboot werden (Auswahl des Eintrags im GRUB-Menü)

#### Alternative 2: Standard-Boot-Eintrag statisch festlegen.

Namen des Boot-Eintrags herausfinden: Suche nach Stichwort "-rt"

    $nano /boot/grub/grub.cfg
    
Liefert (z.B.): "Ubuntu, with Linux 4.19.106-rt45". Einsetzen in Grub-Eintrag (englische Lokalisierung vorausgesetzt):

    GRUB_DEFAULT="Previous Linux versions>Ubuntu, with Linux 4.19.106-rt45"

Testweise übernehmen:

    $sudo update-grub

Liefert (in diesem Beispiel):

    Warning: Please don't use old title `Previous Linux versions>Ubuntu, with Linux 4.19.106-rt45' for GRUB_DEFAULT, use `Advanced options for Ubuntu>Ubuntu, with Linux 4.19.106-rt45' (for versions before 2.00) or `gnulinux-advanced-1c2a0fb0-7a54-456d-8c60-830a6ed6feac>gnulinux-4.19.106-rt45-advanced-1c2a0fb0-7a54-456d-8c60-830a6ed6feac' (for 2.00 or later)

Neuen korrekten Wert eintragen (für dieses Beispiel):

    GRUB_DEFAULT="gnulinux-advanced-1c2a0fb0-7a54-456d-8c60-830a6ed6feac>gnulinux-4.19.106-rt45-advanced-1c2a0fb0-7a54-456d-8c60-830a6ed6feac"

Übernehmen

    $sudo update-grub
    
Anschließend wird automatisch in den Echtzeit-Kernel gebootet (Keine manuelle Auswahl des Eintrags im GRUB-Menü erforderlich).

    $sudo reboot now

## Konfiguration Teil 2

Nutzerrechte setzen

    $sudo addgroup realtime
    $sudo adduser ec realtime

Prioritäten

    $sudo nano /etc/security/limits.conf

Eintragen der folgenden Zeilen.

    @realtime soft rtprio 99
    @realtime soft priority 99
    @realtime soft memlock 102400
    @realtime hard rtprio 99
    @realtime hard priority 99
    @realtime hard memlock 102400
    
Energiesparmodi deaktivieren

    $sudo apt-get install sysfsutils

CPU-Frequenz und -energieverhalten
https://wiki.ubuntuusers.de/Prozessortaktung/


Anzahl CPUs bestimmen:

    $ls /sys/devices/system/cpu/ | grep cpu
    

Konfiguration bearbeiten:

    $sudo nano /etc/sysfs.conf
    
Füge in Datei ein (Anpassung an tatsächliche Anzahl CPUs)

    devices/system/cpu/cpu0/cpufreq/scaling_governor = performance
    devices/system/cpu/cpu1/cpufreq/scaling_governor = performance
    devices/system/cpu/cpu2/cpufreq/scaling_governor = performance
    devices/system/cpu/cpu3/cpufreq/scaling_governor = performance
    devices/system/cpu/cpu4/cpufreq/scaling_governor = performance
    devices/system/cpu/cpu5/cpufreq/scaling_governor = performance
    devices/system/cpu/cpu6/cpufreq/scaling_governor = performance
    devices/system/cpu/cpu7/cpufreq/scaling_governor = performance
    
Kernel-Argumente für CPU

    $sudo nano /etc/default/grub 
    
In Zeile GRUB_CMDLINE_LINUX_DEFAULT ergänzen:

    processor.max_cstate=1 idle=poll
    
Aktualisieren

    $sudo update-grub

## Quellen

Die folgenden Quellen bieten einen Einstieg in ein tieferes Verständnis der Echtzeit-PC Konfiguration:

* [Linux-Magazin: Harte Echtzeit mit Linux durch Preempt-RT zum Messen und Steuern](https://www.linux-magazin.de/ausgaben/2017/03/echtzeit-praxis/2/)
* [HOWTO setup Linux with PREEMPT_RT properly](https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/preemptrt_setup)

