KVS30/M Restore-Skript
======================

Zweck
-----
Dieser Ordner enthaelt ein separates Wartungs-/Reparaturskript fuer die
Thorlabs KVS30/M Vertical Stage. Das Skript ist bewusst NICHT Teil des normalen
Plugin-Ablaufs. Es soll nur manuell verwendet werden, wenn die KVS30/M
Controller-Settings inkonsistent oder korrupt wirken.

Das Skript:

- oeffnet den KVS30/M Controller direkt ueber die Thorlabs Kinesis C-API
- liest aktuelle Motion-/Limit-/Track-Settle-Parameter aus
- kann KVS30/M Defaultwerte fuer Motion- und Sicherheitsparameter setzen
- kann diese Werte optional dauerhaft auf den Controller schreiben

Das Skript sendet KEINE Bewegung, KEIN Home, KEIN Enable und KEIN Disable.


Warum gibt es das?
------------------
Bei der KVS30/M wurden inkonsistente Settings beobachtet, z.B. unrealistische
Software-Limits oder Track/Settle-Werte. Solche Werte koennen dazu fuehren,
dass kleine GUI-Schritte nicht mehr sicher begrenzt werden.

Das Plugin selbst enthaelt inzwischen zusaetzliche Schutzabfragen. Dieses
Skript dient dagegen zur direkten Reparatur der Controller-Settings.


Wichtige Sicherheitsregeln
--------------------------
Vor der Ausfuehrung:

1. Kinesis GUI schliessen.
2. Plugin/SmartLab schliessen oder zumindest sicherstellen, dass die KVS nicht
   verbunden/geoeffnet ist.
3. Stage nicht bewegen, waehrend das Skript laeuft.
4. Zuerst immer nur Query ausfuehren.


Aufruf
------
PowerShell im Ordner

  thorKinesisPlugin\maintenance\kvs30_restore

oeffnen und dann:

  powershell -ExecutionPolicy Bypass -File .\kvs30_restore_motion_defaults.ps1 -Action Query

Das liest nur die aktuellen Werte aus.


Restore ohne dauerhaftes Speichern
----------------------------------
Zum Testen der Defaultwerte, ohne sie dauerhaft zu persistieren:

  powershell -ExecutionPolicy Bypass -File .\kvs30_restore_motion_defaults.ps1 -Action Restore

Das Skript fragt interaktiv nach Bestaetigung. Es schreibt die Werte in die
aktuelle Controller-Konfiguration, persistiert sie aber nicht in den Flash.


Restore mit dauerhaftem Speichern
---------------------------------
Erst verwenden, wenn Query/Restore plausibel aussehen:

  powershell -ExecutionPolicy Bypass -File .\kvs30_restore_motion_defaults.ps1 -Action Restore -Persist

Mit -Persist ruft das Skript KVS_PersistSettings(...) auf. Damit werden die
Settings dauerhaft auf dem Controller gespeichert.


Nicht-interaktiver Aufruf
-------------------------
Nur verwenden, wenn vorher eine andere eindeutige Sicherheitsabfrage erfolgt
ist:

  powershell -ExecutionPolicy Bypass -File .\kvs30_restore_motion_defaults.ps1 -Action Restore -Persist -Force


Default-Zielwerte
-----------------
Das Skript verwendet fuer KVS30/M u.a. diese Defaultwerte:

- Serial: 24522994
- SettingsName: KVS30
- Motor scale: 20000 device units/mm
- Travel range: 0..30 mm
- Stage axis limits: 0..600000 device units
- Software limit policy: DisallowIllegalMoves
- Move acceleration: 1 mm/s^2
- Move max velocity: 2 mm/s
- Motor max acceleration: 5 mm/s^2
- Motor max velocity: 8 mm/s
- Track/Settle time: 197 cycles = 20.1728 ms
- Track/Settle settled error/window: 20 counts
- Track/Settle max tracking error: 0 counts


Andere Seriennummer
-------------------
Falls eine andere KVS30/M repariert werden soll:

  powershell -ExecutionPolicy Bypass -File .\kvs30_restore_motion_defaults.ps1 -Serial 12345678 -Action Query


Hinweis
-------
Dieses Skript ist ein Wartungswerkzeug. Es sollte nicht automatisch beim
Plugin-Start ausgefuehrt werden. Wenn es spaeter aus dem Plugin heraus
gestartet wird, dann nur als explizite Maintenance-Aktion mit vorherigem
Schliessen der KVS-Verbindung im Plugin und klarer Benutzerbestaetigung.
