#ifndef THORLABSKINESISPLUGIN_H
#define THORLABSKINESISPLUGIN_H

#include "thorlabsKinesisPlugin_global.h"
#include "ui_thorlabsKinesisPlugin_UI.h"
#include "interfaces.h"

#include "KinesisDetect.h"
#include "BDCStage.h"
#include "KVSStage.h"

#include <QDockWidget>
#include <QMap>
#include <QVector>
#include <QSet>
#include <QString>
#include <QDebug>

#include <memory>
#include <unordered_map>
#include <array>

class thorlabsKinesisPlugin : public IStage
{
    Q_OBJECT
        Q_PLUGIN_METADATA(IID IStage_iid)
        Q_INTERFACES(IStage)
        Q_INTERFACES(IHardware)

public:
    thorlabsKinesisPlugin();
    ~thorlabsKinesisPlugin();

    // hardware interface
    virtual bool detect() override;
    virtual bool initialize() override;
    virtual QDockWidget* getView() override;
    virtual bool release() override;
    virtual IOutput* getOutput() override;
    virtual QMap<QString, QString> getSaveValueInformation() override;
    virtual void setLoadValueInformation(QMap<QString, QString> map) override;
    virtual void showSettingsWindow() override;

    // stage interface
    // public standard unit: micrometers (µm)
    virtual bool moveTo(double pos, int id = 0) override;
    virtual bool moveTo(double* pos, const char* axes) override;
    virtual bool moveSteps(double steps, int id = 0) override;
    bool moveSteps(double* steps, const char* axes);   // Zusatzfunktion fuer mehrere relative Achsen
    virtual double* getPosition(const char* axes) override;
    virtual double* getPosition() override;
    virtual bool stop() override;

    // legacy methods kept (stubs)
    void setHWSerialNr(QString* serialNr);
    void setHWSerialNr(QString serialNr);
    void setStepSize(QString stepSize);
    void setVelocityParameters(QString minVelocity, QString acceleration, QString maxVelocity);
    bool isStopped();

private:

    struct AxisEntry
    {
        QString baseSerial;  // BDC base serial for M30XY; KVS serial for Z
        int channel = 1;     // 1/2 for M30XY
        bool isM30xy = false;
        QString display;
    };

    void initGUI();
    void refreshAxisUi();

    BDCStage* m30xyForBase(const QString& baseSerial);
    KVSStage* kvsForSerial(const QString& serial);

    bool axisMatches(const AxisEntry& ax, QChar axisName) const;
    bool readAxisPositionUm(int id, double& valueUm) const;
    int findAxisIndexByName(QChar axisName) const;

private:
    Ui::DockWidget ui;
    QDockWidget* dock = nullptr;
    QDockWidget* dockLogger = nullptr;

    QString author;
    QString className;
    QString xmlName;

    QVector<AxisEntry> m_axes;

    std::unordered_map<std::string, std::unique_ptr<BDCStage>> m_m30xy;
    std::unordered_map<std::string, std::unique_ptr<KVSStage>> m_kvs;

    std::array<double, 3> m_lastPositionsUm{ 0.0, 0.0, 0.0 };

public slots:
    void slot_refresh();
    void slot_deviceChanged(int);
    void slot_goHome();
    void slot_openLogger();
    void slot_moveStepForward();
    void slot_moveStepBackward();
    void slot_stopMotor();
    void slot_moveTo();
    void slot_getPosition();

    void slot_applyTrigger();
    void slot_disableTrigger();
};

#endif // THORLABSKINESISPLUGIN_H