M30XY / BDC Restore-Skripte
===========================

Zweck
-----
Dieser Ordner enthaelt separate Wartungs-/Reparaturskripte fuer die
Thorlabs M30XY Stage am Benchtop DC Servo Controller (BDC).

Die Skripte sind bewusst NICHT Teil des normalen Plugin-Ablaufs. Sie sollen
nur manuell verwendet werden, wenn M30XY/BDC Controller-Settings inkonsistent
oder korrupt wirken.

Die Skripte:

- oeffnen den BDC Controller direkt ueber die Thorlabs Kinesis C-API
- lesen aktuelle Motion-, Limit-, Velocity- und Jog-Parameter aus
- koennen M30XY Defaultwerte fuer Motion- und Sicherheitsparameter setzen
- koennen diese Werte optional dauerhaft auf den Controller schreiben

Die Skripte senden KEINE Bewegung, KEIN Home, KEIN Enable und KEIN Disable.


Warum gibt es das?
------------------
Beim KVS30/M Controller wurden bereits korrupte Motion-/Limit-Parameter
beobachtet. Fuer die BDC/M30XY Achsen soll derselbe Sicherheitsanker
existieren: erst auslesen, dann bei Bedarf Defaultwerte direkt auf
Controller-Ebene wiederherstellen.

Das Plugin selbst enthaelt zusaetzliche Schutzabfragen. Dieses Werkzeug ist
fuer direkte Maintenance am Controller gedacht.


Wichtige Sicherheitsregeln
--------------------------
Vor der Ausfuehrung:

1. Kinesis GUI schliessen.
2. Plugin/SmartLab schliessen oder sicherstellen, dass der BDC Controller
   nicht verbunden/geoeffnet ist.
3. Stage nicht bewegen, waehrend das Skript laeuft.
4. Zuerst immer nur read/query ausfuehren.
5. Bei zwei M30XY-Controllern immer die richtige Base-Serial angeben.

Bekannte M30XY Base-Serials aus den Logs:

- 101517004
- 101519064


Dateien
-------
m30xy_read_motion_params.ps1
    Read-only Wrapper. Ruft nur Query auf.

m30xy_restore_motion_defaults.ps1
    Hauptskript. Kann Query oder Restore ausfuehren.


Read-only Aufruf
----------------
PowerShell im Ordner

  thorKinesisPlugin\maintenance\m30xy_restore

oeffnen und dann z.B.:

  powershell -ExecutionPolicy Bypass -File .\m30xy_read_motion_params.ps1 -Serial 101517004

oder direkt:

  powershell -ExecutionPolicy Bypass -File .\m30xy_restore_motion_defaults.ps1 -Serial 101517004 -Action Query

Nur einen Kanal lesen:

  powershell -ExecutionPolicy Bypass -File .\m30xy_read_motion_params.ps1 -Serial 101517004 -Channels 1


Restore ohne dauerhaftes Speichern
----------------------------------
Zum Testen der Defaultwerte, ohne sie dauerhaft in den Controller-Flash zu
schreiben:

  powershell -ExecutionPolicy Bypass -File .\m30xy_restore_motion_defaults.ps1 -Serial 101517004 -Action Restore

Das Skript fragt interaktiv nach Bestaetigung. Es schreibt die Werte in die
aktuelle Controller-Konfiguration, persistiert sie aber nicht dauerhaft.


Restore mit dauerhaftem Speichern
---------------------------------
Erst verwenden, wenn Query/Restore plausibel aussehen:

  powershell -ExecutionPolicy Bypass -File .\m30xy_restore_motion_defaults.ps1 -Serial 101517004 -Action Restore -Persist

Mit -Persist ruft das Skript BDC_PersistSettings(...) pro Kanal auf. Damit
werden die Settings dauerhaft auf dem Controller gespeichert.


Nicht-interaktiver Aufruf
-------------------------
Nur verwenden, wenn vorher eine andere eindeutige Sicherheitsabfrage erfolgt
ist:

  powershell -ExecutionPolicy Bypass -File .\m30xy_restore_motion_defaults.ps1 -Serial 101517004 -Action Restore -Persist -Force


Default-Zielwerte
-----------------
Die Zielwerte stammen aus ThorlabsDefaultSettings.xml, DeviceSettingsDefinition
"M30 Series":

- SettingsName: M30 Series
- Motor scale: 10000 device units/mm
- Travel range: -15..+15 mm
- Stage axis limits: -150000..+150000 device units
- Software limit policy: DisallowIllegalMoves
- Move acceleration: 5 mm/s^2
- Move max velocity: 2.3 mm/s
- Motor max acceleration: 5 mm/s^2
- Motor max velocity: 2.6 mm/s
- Jog mode: single step
- Jog step: 0.5 mm
- Jog acceleration: 4 mm/s^2
- Jog max velocity: 2.6 mm/s
- Jog stop mode: profiled stop


Andere Settings-Namen
---------------------
Falls Kinesis fuer euren Controller statt "M30 Series" explizit "M30X"
verwendet, kann der Settings-Name ueberschrieben werden:

  powershell -ExecutionPolicy Bypass -File .\m30xy_restore_motion_defaults.ps1 -Serial 101517004 -Action Restore -SettingsName "M30X"


Hinweis
-------
Dieses Skript ist ein Wartungswerkzeug. Es sollte nicht automatisch beim
Plugin-Start ausgefuehrt werden. Wenn es spaeter aus dem Plugin heraus
gestartet wird, dann nur als explizite Maintenance-Aktion mit vorherigem
Schliessen der BDC-Verbindung im Plugin und klarer Benutzerbestaetigung.
