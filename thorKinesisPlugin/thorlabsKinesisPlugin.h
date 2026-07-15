#ifndef THORLABSKINESISPLUGIN_H
#define THORLABSKINESISPLUGIN_H

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4005)
#endif

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

#include "thorlabsKinesisPlugin_global.h"
#include "ui_thorlabsKinesisPlugin_UI.h"
#include "interfaces.h"

#include "KinesisDetect.h"
#include "BDCStage.h"
#include "KVSStage.h"
#include "WindowsGamepadInput.h"

#include <QDockWidget>
#include <QMap>
#include <QVector>
#include <QSet>
#include <QString>
#include <QDebug>
#include <QThread>
#include <QStringList>

#include <memory>
#include <unordered_map>
#include <array>
#include <functional>
#include <mutex>
#include <atomic>
#include <cstddef>
#include <chrono>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

class QFrame;
class QLabel;
class QLCDNumber;
class QDoubleSpinBox;
class QLineEdit;
class QPushButton;
class QTimer;
class QToolButton;
class ThorlabsPositionManagerDialog;

class thorlabsKinesisPlugin : public IScanStage
{
    Q_OBJECT
        Q_PLUGIN_METADATA(IID IStage_iid)
        Q_INTERFACES(IScanStage)
        Q_INTERFACES(IStageMultiAxis)
        Q_INTERFACES(IStage)
        Q_INTERFACES(IHardware)

public:
    using IStageMultiAxis::moveTo;
    using IStageMultiAxis::moveSteps;

    thorlabsKinesisPlugin();
    ~thorlabsKinesisPlugin();

    struct AxisInfo
    {
        int globalAxisID = 0;
        QString axisName;
        QString serial;
        double minimumTravelUm = 0.0;
        double maximumTravelUm = 0.0;
        bool travelLimitsValid = false;
    };

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
    virtual bool moveTo(const double* positions, const int* axes, int axisCount) override;

    virtual bool moveSteps(double steps, int id = 0) override;
    bool moveSteps(double* steps, const char* axes);   // Zusatzfunktion fuer mehrere relative Achsen
    virtual bool moveSteps(const double* steps, const int* axes, int axisCount) override;
    virtual double* getPosition(const char* axes) override;
    virtual double* getPosition() override;
    virtual bool stop() override;

    // scan interface
    ScanCapabilities getScanCapabilities() const override;
    ScanValidationResult validateScanJob(const ScanJob& job) const override;
    bool executeScanJob(const ScanJob& job) override;
    bool abortScanJob() override;

    QVector<AxisInfo> getAxisInfo() const;
    QStringList getPositionConfigNames() const;
    QString getActivePositionConfigName() const;
    bool getPositionConfig(const QString& name, QVector<int>& globalAxisIDs,
        QVector<double>& positionsUm) const;
    bool savePositionConfig(const QString& name, const QVector<int>& globalAxisIDs,
        const QVector<double>& positionsUm);
    bool removePositionConfig(const QString& name);
    Q_INVOKABLE bool choosePositionConfig(const QString& name);
    Q_INVOKABLE bool goToPositionConfig();
    Q_INVOKABLE bool goToPositionConfig(const QString& name);

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
        int globalAxisId = 0; // public/backend axis ID, 1-based and contiguous
        QString axisName;
        QString display;
    };

    struct AxisUi
    {
        QFrame* frame = nullptr;
        QLabel* title = nullptr;
        QLabel* statusLabel = nullptr;
        QLCDNumber* positionLcd = nullptr;
        QDoubleSpinBox* positionEdit = nullptr;
        QDoubleSpinBox* stepEdit = nullptr;
        QLineEdit* triggerStartEdit = nullptr;
        QLineEdit* triggerIntervalEdit = nullptr;
        QLineEdit* triggerCountEdit = nullptr;
        QLineEdit* triggerWidthEdit = nullptr;
        QLineEdit* triggerVelocityEdit = nullptr;
        QLabel* triggerFrequencyValue = nullptr;
        QPushButton* homeButton = nullptr;
        QPushButton* moveButton = nullptr;
        QPushButton* stopButton = nullptr;
        QPushButton* stepDownButton = nullptr;
        QPushButton* stepUpButton = nullptr;
        QPushButton* applyTriggerButton = nullptr;
        QPushButton* disableTriggerButton = nullptr;
        QToolButton* triggerMenuButton = nullptr;
    };

    struct PositionConfig
    {
        QString name;
        QVector<int> globalAxisIDs;
        QVector<double> positionsUm;
    };

    struct GamepadDirectionBinding
    {
        int globalAxisId = 0;
        int sign = 1;
    };

    struct GamepadConfig
    {
        bool enabled = false;
        QString deviceKey;
        double axisLeftXCenter = 0.0;
        double axisLeftYCenter = 0.0;
        double deadzone = 0.25;
        double baseJogVelocityMmS = 0.5;
        double fastMultiplier = 3.0;
        double slowMultiplier = 0.25;
        int triggerButton = 0;
        int slowButton = 1;
        int fastButton = 2;
        int zDownButton = 4;
        int zUpButton = 5;
        std::array<GamepadDirectionBinding, 4> directionBindings;
        int zAxisGlobalId = 0;
        bool zSoftLimitsEnabled = true;
        double zMinUm = 0.0;
        double zMaxUm = 30000.0;
        int triggerAxisGlobalId = 0;
        int triggerOutputPort = 1;
        int triggerPulseMs = 50;
    };

    struct GamepadActiveJog
    {
        int sign = 0;
        double velocityMmS = 0.0;
    };

    void initGUI();
    void refreshAxisUi();
    void rebuildAxisFrames();
    void clearAxisFrames();
    void selectAxis(int id);
    void refreshAxisPositionUi(int id);
    void refreshAxisStatusUi(int id);
    void refreshAllAxisPositionsUi();
    void syncLegacyMotionInputsFromAxisUi(int id);
    void syncLegacyTriggerInputsFromAxisUi(int id);
    void updateTriggerFrequencyUi(int id);
    void setupAxisTriggerMenu(AxisUi& axisUi, QWidget* parent);
    void updateAxisFrameAreaHeight();
    void setMotionUiBusy(bool busy);
    void setAxisControlsBusy(int id, bool busy);
    bool isAxisUiBusy(int id) const;
    void startMotionTask(const QString& operation, std::function<bool()> task);
    void startAxisMotionTask(int axisIndex, const QString& operation,
        std::function<bool()> task, bool referencing = false);
    void waitForMotionToFinish();
    bool waitUntilAllAxesStopped(int timeoutMs = 120000);
    void closeDevices();
    bool disableAllTriggers(bool force = false);
    bool chooseAxesForInitialization();
    void assignGlobalAxisIds(QVector<AxisEntry>& axes) const;
    QString axisKey(const AxisEntry& axis) const;
    QString axisDisplayText(const AxisEntry& axis) const;
    void refreshAxisCombo();
    int axisIndexFromGlobalId(int globalAxisId) const;
    int axisIndexFromPublicId(int id) const;
    bool resolveAxisRequest(const char* axes, QVector<int>& axisIndices) const;
    bool axisTravelLimitsUm(const AxisEntry& axis, double& minUm, double& maxUm) const;
    bool moveToAxisIndex(double posUm, int axisIndex, bool waitForOtherAxes = true);
    bool moveStepsAxisIndex(double stepsUm, int axisIndex, bool waitForOtherAxes = true);
    bool moveAxesCoordinated(const double* values, const char* axes, bool relative);
    bool moveAxesByGlobalIds(const double* values, const int* globalAxisIds, int axisCount, bool relative);
    bool moveToAxisCodes(const double* positions, const int* axes, std::size_t count);
    bool executeScanLine(const LaserLine& line, const ScanJob& job);
    bool configureLineTrigger(BDCStage* stage, unsigned channel, double startUm, double endUm, const ScanJob& job);
    void openPositionManager();
    int positionConfigIndex(const QString& name) const;

    BDCStage* m30xyForBase(const QString& baseSerial);
    KVSStage* kvsForSerial(const QString& serial);

    bool axisMatches(const AxisEntry& ax, QChar axisName) const;
    bool readAxisPositionUm(int id, double& valueUm) const;
    bool readAxisMotionState(int id, bool& moving, bool& homed) const;
    bool stopAxisIndex(int id);
    int findAxisIndexByName(QChar axisName) const;
    void setupGamepadControls();
    void refreshGamepadControlBar();
    void openGamepadConfigDialog();
    void ensureGamepadDefaults();
    void updateGamepadDevice();
    void pollGamepad();
    void stopGamepadJogs(bool sendStop, bool immediate = false);
    void clearGamepadJogState();
    bool startGamepadJogAxis(int axisIndex, int sign, double velocityMmS);
    void stopGamepadJogAxis(int axisIndex, bool sendStop, bool immediate = false);
    bool isGamepadZJogAllowed(int axisIndex, int sign, double velocityMmS) const;
    bool pulseGamepadTrigger();

private:
    Ui::DockWidget ui;
    QDockWidget* dock = nullptr;

    QString author;
    QString className;
    QString xmlName;

    QVector<AxisEntry> m_detectedAxes;
    QVector<AxisEntry> m_axes;
    QVector<AxisUi> m_axisUi;
    QMap<int, QThread*> m_axisMotionThreads;
    QSet<int> m_busyAxisIndices;
    QSet<int> m_referencingAxisIndices;
    QSet<QString> m_selectedAxisKeys;
    QVector<PositionConfig> m_positionConfigs;
    int m_activePositionConfigIndex = -1;
    ThorlabsPositionManagerDialog* m_positionManagerWindow = nullptr;

    std::unordered_map<std::string, std::unique_ptr<BDCStage>> m_m30xy;
    std::unordered_map<std::string, std::unique_ptr<KVSStage>> m_kvs;
    std::mutex m_deviceMapMutex;

    std::vector<double> m_lastPositionsUm;
    QThread* m_motionThread = nullptr;
    QTimer* m_positionRefreshTimer = nullptr;
    QTimer* m_gamepadPollTimer = nullptr;
    std::unique_ptr<WindowsGamepadInput> m_gamepadInput;
    GamepadConfig m_gamepadConfig;
    QMap<int, GamepadActiveJog> m_gamepadActiveJogs;
    QMap<int, std::chrono::steady_clock::time_point> m_gamepadRestartNotBefore;
    bool m_gamepadTriggerWasPressed = false;
    bool m_gamepadSuppressUntilNeutral = false;
    int m_gamepadReconnectPolls = 0;
    bool m_guiInitialized = false;
    std::atomic_bool m_motionTaskActive{ false };
    std::atomic_bool m_stopRequested{ false };

public slots:
    void slot_detect();
    void slot_deviceChanged(int);
    void slot_goHome();
    void slot_moveStepForward();
    void slot_moveStepBackward();
    void slot_stopMotor();
    void slot_moveTo();
    void slot_getPosition();
    void slot_openPositionManager();

    void slot_applyTrigger();
    void slot_disableTrigger();
};

#endif // THORLABSKINESISPLUGIN_H
