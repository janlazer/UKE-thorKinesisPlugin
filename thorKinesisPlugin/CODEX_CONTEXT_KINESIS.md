# CODEX CONTEXT – THORLABS KINESIS PLUGIN

## Projektziel

Entwicklung eines generischen Stage-Plugins für ein Messsystem.

Aktuell werden unterstützt:

* Thorlabs M30XY (XY Stage)
* Thorlabs KVS30 (Z Stage)

Langfristiges Ziel:

* beliebige Stage-Plugins
* gemeinsame Scanlogik
* Laserablation
* 3D-Rasterung
* Trigger-basierte Laserschüsse
* Kamera-Trigger
* beliebige Mess-Workflows

---

# Aktueller Stand

## M30XY

Hardware:

* Thorlabs M30XY

Kinesis API:

* Benchtop DC Servo (BDC)

Header:

```cpp
Thorlabs.MotionControl.Benchtop.DCServo.h
```

Controller:

```text
Device Type = 3
Settings ID = 101
```

M30XY besitzt:

```text
Channel 1 = X
Channel 2 = Y
```

Die Stage wird nur EINMAL geöffnet.

Danach werden beide Achsen über die Channels angesprochen.

Beispiel:

```cpp
BDC_Open(serial);

BDC_StartPolling(serial,1,200);
BDC_StartPolling(serial,2,200);

BDC_EnableChannel(serial,1);
BDC_EnableChannel(serial,2);
```

---

## KVS30

Hardware:

* KVS30 Vertical Stage

Header:

```cpp
Thorlabs.MotionControl.KCube.DCServo.h
```

Detection:

```cpp
TLI_GetDeviceListByTypeExt(...,24)
```

Device Type:

```text
24
```

---

# Pluginstruktur

Aktuell:

```text
thorlabsKinesisPlugin
¦
+-- BDCStage
¦     +-- Channel 1 = X
¦     +-- Channel 2 = Y
¦
+-- KVSStage
      +-- Z
```

---

# Detection

## KVS

```cpp
TLI_GetDeviceListByTypeExt(...,24)
```

liefert:

```text
24522994
```

---

## M30XY

WICHTIG:

NICHT Type 43 verwenden.

M30XY wird erkannt über:

```cpp
TLI_GetDeviceListByTypeExt(...,101)
```

liefert:

```text
101517004
101519064
...
```

Plugin erzeugt daraus:

```text
M30XY X
M30XY Y
```

---

# Aktuelle Triggerfunktion

Verwendete Kinesis Typen:

```cpp
KMOT_TriggerPortMode
KMOT_TriggerPortPolarity
KMOT_TriggerParams
```

Relevanter Modus:

```cpp
KMOT_TrigOut_AtPositionStepFwd
KMOT_TrigOut_AtPositionStepRev
KMOT_TrigOut_AtPositionStepBoth
```

Parameter:

```cpp
KMOT_TriggerParams
{
    TriggerStartPositionFwd;
    TriggerIntervalFwd;
    TriggerPulseCountFwd;

    TriggerStartPositionRev;
    TriggerIntervalRev;
    TriggerPulseCountRev;

    TriggerPulseWidth;
    CycleCount;
};
```

---

# Laserablations-Konzept

Laser:

```text
max. Triggerfrequenz = 20 Hz
```

Spotabstand:

```text
100 µm
```

Scanstrategie:

```text
Serpentine
```

Beispiel:

?????
?????
?????
?????

---

# Kritische Anforderung

Während Positionierbewegungen dürfen KEINE Laserschüsse ausgelöst werden.

Insbesondere:

* Wechsel zur nächsten Linie
* Rückfahrt
* Y-Versatz
* Startposition anfahren

müssen triggerfrei erfolgen.

Ablauf pro Linie:

1. Trigger OFF

2. Fahre Startpunkt an

3. Warten bis Bewegung beendet

4. Trigger konfigurieren

5. Trigger ON

6. Linie scannen

7. Warten bis Bewegung beendet

8. Trigger OFF

Danach nächste Linie.

---

# Geplante Zielarchitektur

Es soll KEINE Scanlogik in den einzelnen Stages liegen.

Die Stages sollen nur Bewegungsprimitive bereitstellen.

---

## Stage Interface

Beispiel:

```cpp
class IStage
{
public:

    virtual bool moveAbs(...) = 0;
    virtual bool moveRel(...) = 0;

    virtual bool home() = 0;

    virtual bool isMoving() = 0;

    virtual bool enablePositionTrigger(...) = 0;
    virtual bool disableTrigger() = 0;
};
```

---

# Scan-Datenstrukturen

## ScanLine

```cpp
struct ScanLine
{
    double xStart;
    double yStart;

    double xEnd;
    double yEnd;
};
```

---

## ScanLayer

```cpp
struct ScanLayer
{
    double z;

    std::vector<ScanLine> lines;
};
```

---

## ScanJob

```cpp
struct ScanJob
{
    std::vector<ScanLayer> layers;

    double velocity_mm_s;

    double triggerSpacing_um;
};
```

---

# Verantwortlichkeiten

## Hauptprojekt

Erzeugt ScanJobs.

Beispiele:

* Punktliste
* Polygon
* STL Slice
* ROI
* Freiformkontur

Das Plugin kennt die Herkunft der Daten NICHT.

Es erhält lediglich:

```cpp
ScanJob
```

---

## Plugin

Führt nur aus:

```cpp
runScanJob(job);
```

---

# Geplanter ScanExecutor

Neue generische Klasse:

```text
ScanExecutor
```

Aufgabe:

Ausführen eines ScanJobs über beliebige Stages.

---

Beispiel:

```cpp
ScanExecutor executor(
    xyStage,
    zStage
);

executor.run(job);
```

---

# 3D Scan

Für jeden Layer:

1. Z Position anfahren
2. Warten
3. XY Linien scannen
4. Nächster Layer

Beispiel:

Layer 0

XY Raster

?

Layer 1

XY Raster

?

Layer 2

XY Raster

---

# Wichtig

KVS30 dient primär als Z-Achse.

Obwohl die KVS Trigger unterstützt, werden diese aktuell nicht benötigt.

Lasertrigger werden ausschließlich über die XY-Stage erzeugt.

---

# Frühere Architektur

Früheres Projekt enthielt:

```text
ScanPatterns
```

mit:

* line()
* parallelLines()
* cross()
* connectPoints()

Dort wurden vollständige Punktlisten erzeugt.

Neue Architektur soll NICHT mehr Millionen Punkte generieren.

Stattdessen:

```cpp
ScanLine
```

repräsentiert eine komplette Linie.

Die Stage fährt diese Linie kontinuierlich ab.

Der Trigger erzeugt die Laserschüsse hardwareseitig über:

```cpp
KMOT_TrigOut_AtPositionStepBoth
```

Dadurch:

* weniger Speicher
* höhere Geschwindigkeit
* sauberere Architektur

---

# Offene Aufgaben

1. ScanJob implementieren

2. ScanExecutor implementieren

3. Trigger API sauber in BDCStage integrieren

4. Generische Stage Interfaces definieren

5. Plugin-API erweitern:

```cpp
runScanJob(...)
abortScanJob(...)
pauseScanJob(...)
```

6. Kamera-Trigger Workflow integrieren

7. Mehrere Stage-Hersteller unterstützen
