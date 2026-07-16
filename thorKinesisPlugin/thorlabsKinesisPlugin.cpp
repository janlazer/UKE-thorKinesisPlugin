#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4005)
#endif

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <chrono>
#include <limits>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "thorlabsKinesisPlugin.h"
#include "ScanValidation.h"
#include "ThorlabsPositionManagerDialog.h"
#include "ui_stageFrame.h"

#include <QAbstractSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QLocale>
#include <QMenu>
#include <QMessageBox>
#include <QMetaObject>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSize>
#include <QSizePolicy>
#include <QSpinBox>
#include <QTimer>
#include <QToolButton>
#include <QWidget>
#include <QWidgetAction>
#include <QVBoxLayout>
#include <array>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

namespace
{
    constexpr double kMaximumLaserTriggerFrequencyHz = 20.0;
    constexpr double kScanLineAxisToleranceUm = 1e-6;
    constexpr int kPositionRefreshIntervalMs = 1000;
    constexpr int kStageFrameWidth = 680;
    constexpr int kSerialButtonHeight = 24;
    constexpr int kPanicButtonWidth = 120;
    constexpr int kTopBarSpacing = 2;
    constexpr int kTopActionButtonWidth =
        (kStageFrameWidth - kPanicButtonWidth - 3 * kTopBarSpacing) / 3;
    constexpr int kDetectButtonWidth =
        kStageFrameWidth - kPanicButtonWidth - kTopActionButtonWidth - 2 * kTopBarSpacing;
    constexpr int kGroupHeaderSpacing = 4;
    constexpr int kGamepadPollIntervalMs = 50;
    constexpr int kGamepadDirections = 4;
    constexpr int kGamepadDirectionUp = 0;
    constexpr int kGamepadDirectionDown = 1;
    constexpr int kGamepadDirectionLeft = 2;
    constexpr int kGamepadDirectionRight = 3;
    constexpr double kGamepadM30MaxJogVelocityMmS = 2.6;
    constexpr double kGamepadKvsMaxJogVelocityMmS = 2.0;
    constexpr double kGamepadLimitToleranceUm = 1.0;
    constexpr int kGamepadProfiledStopStatusDelayMs = 250;
    constexpr double kGamepadVelocityChangeTolerance = 0.001;

    int preferredWidgetHeight(QWidget* widget)
    {
        if (!widget)
            return 0;

        widget->ensurePolished();
        const QSize hint = widget->sizeHint();
        return qMax(widget->minimumHeight(), hint.height());
    }

    ScanValidationResult scanValidationError(ScanValidationError error,
        std::size_t layerIndex = 0,
        std::size_t lineIndex = 0)
    {
        ScanValidationResult result;
        result.error = error;
        result.layerIndex = layerIndex;
        result.lineIndex = lineIndex;
        return result;
    }

    bool nearlyEqual(double a, double b)
    {
        return std::fabs(a - b) <= kScanLineAxisToleranceUm;
    }

    bool scanJobNeedsZAxis(const ScanJob& job)
    {
        if (job.layers.empty())
            return false;

        const double firstZUm = job.layers.front().zUm;
        for (const ScanLayer& layer : job.layers)
        {
            if (!nearlyEqual(layer.zUm, 0.0) || !nearlyEqual(layer.zUm, firstZUm))
                return true;
        }

        return false;
    }

    bool scanAxisForLine(const LaserLine& line, QChar& axisName)
    {
        const bool horizontal = nearlyEqual(line.start.yUm, line.end.yUm)
            && !nearlyEqual(line.start.xUm, line.end.xUm);
        const bool vertical = nearlyEqual(line.start.xUm, line.end.xUm)
            && !nearlyEqual(line.start.yUm, line.end.yUm);

        if (horizontal)
        {
            axisName = QChar('x');
            return true;
        }

        if (vertical)
        {
            axisName = QChar('y');
            return true;
        }

        return false;
    }

    int32_t pulseCountForLine(double startUm, double endUm, double triggerSpacingUm)
    {
        const double lengthUm = std::fabs(endUm - startUm);
        const double pulseCount = std::floor(lengthUm / triggerSpacingUm) + 1.0;
        if (!std::isfinite(pulseCount)
            || pulseCount < 1.0
            || pulseCount > static_cast<double>((std::numeric_limits<int32_t>::max)()))
            return 0;

        return static_cast<int32_t>(pulseCount);
    }

    void applyQMotionLikeWidgetStyle(QWidget* widget)
    {
        if (!widget)
            return;

        widget->setStyleSheet(QStringLiteral(
            "QPushButton, QToolButton { min-height: 22px; max-height: 24px; padding-left: 6px; padding-right: 6px; }"
            "QLineEdit, QComboBox, QDoubleSpinBox { min-height: 22px; max-height: 24px; }"));
    }
}

static bool parseI32(const QString& s, int32_t& value)
{
    bool ok = false;
    const qlonglong parsed = s.trimmed().toLongLong(&ok);
    if (!ok
        || parsed < std::numeric_limits<int32_t>::min()
        || parsed > std::numeric_limits<int32_t>::max())
        return false;

    value = static_cast<int32_t>(parsed);
    return true;
}

static bool parseFiniteDouble(const QString& s, double& value)
{
    bool ok = false;
    QString normalized = s.trimmed();
    normalized.replace(',', '.');
    const double parsed = normalized.toDouble(&ok);
    if (!ok || !std::isfinite(parsed))
        return false;

    value = parsed;
    return true;
}

static QString guiDecimalText(double value, int precision = 3)
{
    return QString::number(value, 'f', precision).replace('.', ',');
}

static QString mmDisplayTextFromUm(double valueUm)
{
    return guiDecimalText(valueUm / 1000.0);
}

static QString mmDisplayWithUnitFromUm(double valueUm)
{
    return mmDisplayTextFromUm(valueUm) + QStringLiteral(" mm");
}

static QString frequencyDisplayText(double velocityMmS, double spacingMm)
{
    if (!std::isfinite(velocityMmS)
        || !std::isfinite(spacingMm)
        || velocityMmS <= 0.0
        || spacingMm <= 0.0)
    {
        return QStringLiteral("n/a");
    }

        return guiDecimalText(velocityMmS / spacingMm);
    }

    QString boolSaveText(bool value)
    {
        return value ? QStringLiteral("1") : QStringLiteral("0");
    }

    bool loadBoolValue(const QMap<QString, QString>& map, const QString& key, bool fallback)
    {
        const QString value = map.value(key).trimmed().toLower();
        if (value == QStringLiteral("1") || value == QStringLiteral("true"))
            return true;
        if (value == QStringLiteral("0") || value == QStringLiteral("false"))
            return false;
        return fallback;
    }

    int loadIntValue(const QMap<QString, QString>& map, const QString& key, int fallback)
    {
        bool ok = false;
        const int value = map.value(key).toInt(&ok);
        return ok ? value : fallback;
    }

    double loadDoubleValue(const QMap<QString, QString>& map, const QString& key, double fallback)
    {
        bool ok = false;
        QString normalized = map.value(key).trimmed();
        normalized.replace(',', '.');
        const double value = normalized.toDouble(&ok);
        return ok && std::isfinite(value) ? value : fallback;
    }

    static bool looksLikeBaseM30XYSerial(const QString& serial)
    {
    // Kinesis sometimes shows "base-1" / "base-2" for bay/channel views.
    // We only accept the base serial (no dash) as the BDC base.
    return !serial.contains('-');
}

thorlabsKinesisPlugin::thorlabsKinesisPlugin()
{
    this->setName("Thorlabs Kinesis Plugin");
    this->author = "JH";
    this->xmlName = "Thorlabs";
    this->className = "thorlabsKinesisPlugin";

    this->setType(STAGE);
    this->setDetectable(true);
    this->setDetected(false);
    this->setInitialized(false);
    this->setOutput(true);
    m_gamepadInput = std::make_unique<WindowsGamepadInput>();

    dock = new QDockWidget();
    ui.setupUi(dock);
    dock->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    dock->setMinimumWidth(kStageFrameWidth + 8);
    if (dock->widget())
    {
        dock->widget()->setMinimumWidth(kStageFrameWidth);
        dock->widget()->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    }
    ui.verticalLayout->setAlignment(Qt::AlignTop);
    ui.topBarLayout->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    ui.pushButton_detect->setFixedSize(kDetectButtonWidth, kSerialButtonHeight);
    ui.pushButton_positionManager->setFixedSize(kTopActionButtonWidth, kSerialButtonHeight);
    ui.pushButton_stop->setFixedSize(kPanicButtonWidth, kSerialButtonHeight);
    ui.stageFrameContainer->setMinimumWidth(kStageFrameWidth);
    dock->setWindowTitle(getName());

    connect(dock, &QObject::destroyed, this, [this]() { dock = nullptr; });

    initGUI();

    qDebug() << className << "::ctor created";
}

thorlabsKinesisPlugin::~thorlabsKinesisPlugin()
{
    release();

    if (m_positionManagerWindow)
    {
        delete m_positionManagerWindow;
        m_positionManagerWindow = nullptr;
    }
    if (dock && !dock->parent())
        delete dock;
}

QDockWidget* thorlabsKinesisPlugin::getView()
{
    return dock;
}

IOutput* thorlabsKinesisPlugin::getOutput()
{
    return NULL;
}

QMap<QString, QString> thorlabsKinesisPlugin::getSaveValueInformation()
{
    QMap<QString, QString> map;
    map.insert(QStringLiteral("gamepad/schemaVersion"), QString::number(m_gamepadConfig.schemaVersion));
    map.insert(QStringLiteral("gamepad/enabled"), boolSaveText(m_gamepadConfig.enabled));
    map.insert(QStringLiteral("gamepad/deviceKey"), m_gamepadConfig.deviceKey);
    map.insert(QStringLiteral("gamepad/axisLeftXCenter"), QString::number(m_gamepadConfig.axisLeftXCenter, 'g', 12));
    map.insert(QStringLiteral("gamepad/axisLeftYCenter"), QString::number(m_gamepadConfig.axisLeftYCenter, 'g', 12));
    map.insert(QStringLiteral("gamepad/deadzone"), QString::number(m_gamepadConfig.deadzone, 'g', 12));
    map.insert(QStringLiteral("gamepad/baseJogVelocityMmS"), QString::number(m_gamepadConfig.baseJogVelocityMmS, 'g', 12));
    map.insert(QStringLiteral("gamepad/fastMultiplier"), QString::number(m_gamepadConfig.fastMultiplier, 'g', 12));
    map.insert(QStringLiteral("gamepad/slowMultiplier"), QString::number(m_gamepadConfig.slowMultiplier, 'g', 12));
    map.insert(QStringLiteral("gamepad/triggerButton"), QString::number(m_gamepadConfig.triggerButton));
    map.insert(QStringLiteral("gamepad/slowButton"), QString::number(m_gamepadConfig.slowButton));
    map.insert(QStringLiteral("gamepad/fastButton"), QString::number(m_gamepadConfig.fastButton));
    map.insert(QStringLiteral("gamepad/zDownButton"), QString::number(m_gamepadConfig.zDownButton));
    map.insert(QStringLiteral("gamepad/zUpButton"), QString::number(m_gamepadConfig.zUpButton));
    for (int direction = 0; direction < kGamepadDirections; ++direction)
    {
        const QString prefix = QStringLiteral("gamepad/direction%1/").arg(direction);
        map.insert(prefix + QStringLiteral("axis"), QString::number(m_gamepadConfig.directionBindings[direction].globalAxisId));
        map.insert(prefix + QStringLiteral("sign"), QString::number(m_gamepadConfig.directionBindings[direction].sign));
    }
    map.insert(QStringLiteral("gamepad/zAxis"), QString::number(m_gamepadConfig.zAxisGlobalId));
    map.insert(QStringLiteral("gamepad/zSoftLimitsEnabled"), boolSaveText(m_gamepadConfig.zSoftLimitsEnabled));
    map.insert(QStringLiteral("gamepad/zMinUm"), QString::number(m_gamepadConfig.zMinUm, 'g', 12));
    map.insert(QStringLiteral("gamepad/zMaxUm"), QString::number(m_gamepadConfig.zMaxUm, 'g', 12));
    map.insert(QStringLiteral("gamepad/triggerAxis"), QString::number(m_gamepadConfig.triggerAxisGlobalId));
    map.insert(QStringLiteral("gamepad/triggerOutputPort"), QString::number(m_gamepadConfig.triggerOutputPort));
    map.insert(QStringLiteral("gamepad/triggerPulseMs"), QString::number(m_gamepadConfig.triggerPulseMs));
    return map;
}

void thorlabsKinesisPlugin::setLoadValueInformation(QMap<QString, QString> map)
{
    m_gamepadConfig.schemaVersion = qBound(0,
        loadIntValue(map, QStringLiteral("gamepad/schemaVersion"), 0), 4);
    m_gamepadConfig.enabled = loadBoolValue(map, QStringLiteral("gamepad/enabled"), m_gamepadConfig.enabled);
    m_gamepadConfig.deviceKey = map.value(QStringLiteral("gamepad/deviceKey"), m_gamepadConfig.deviceKey);
    m_gamepadConfig.axisLeftXCenter = loadDoubleValue(map, QStringLiteral("gamepad/axisLeftXCenter"), m_gamepadConfig.axisLeftXCenter);
    m_gamepadConfig.axisLeftYCenter = loadDoubleValue(map, QStringLiteral("gamepad/axisLeftYCenter"), m_gamepadConfig.axisLeftYCenter);
    m_gamepadConfig.deadzone = loadDoubleValue(map, QStringLiteral("gamepad/deadzone"), m_gamepadConfig.deadzone);
    m_gamepadConfig.baseJogVelocityMmS = loadDoubleValue(map, QStringLiteral("gamepad/baseJogVelocityMmS"), m_gamepadConfig.baseJogVelocityMmS);
    m_gamepadConfig.fastMultiplier = loadDoubleValue(map, QStringLiteral("gamepad/fastMultiplier"), m_gamepadConfig.fastMultiplier);
    m_gamepadConfig.slowMultiplier = loadDoubleValue(map, QStringLiteral("gamepad/slowMultiplier"), m_gamepadConfig.slowMultiplier);
    m_gamepadConfig.triggerButton = qBound(0,
        loadIntValue(map, QStringLiteral("gamepad/triggerButton"), m_gamepadConfig.triggerButton),
        WindowsGamepadInput::ButtonCount - 1);
    m_gamepadConfig.slowButton = qBound(0,
        loadIntValue(map, QStringLiteral("gamepad/slowButton"), m_gamepadConfig.slowButton),
        WindowsGamepadInput::ButtonCount - 1);
    m_gamepadConfig.fastButton = qBound(0,
        loadIntValue(map, QStringLiteral("gamepad/fastButton"), m_gamepadConfig.fastButton),
        WindowsGamepadInput::ButtonCount - 1);
    m_gamepadConfig.zDownButton = qBound(0,
        loadIntValue(map, QStringLiteral("gamepad/zDownButton"), m_gamepadConfig.zDownButton),
        WindowsGamepadInput::ButtonCount - 1);
    m_gamepadConfig.zUpButton = qBound(0,
        loadIntValue(map, QStringLiteral("gamepad/zUpButton"), m_gamepadConfig.zUpButton),
        WindowsGamepadInput::ButtonCount - 1);
    for (int direction = 0; direction < kGamepadDirections; ++direction)
    {
        const QString prefix = QStringLiteral("gamepad/direction%1/").arg(direction);
        m_gamepadConfig.directionBindings[direction].globalAxisId =
            loadIntValue(map, prefix + QStringLiteral("axis"), m_gamepadConfig.directionBindings[direction].globalAxisId);
        m_gamepadConfig.directionBindings[direction].sign =
            loadIntValue(map, prefix + QStringLiteral("sign"), m_gamepadConfig.directionBindings[direction].sign);
    }
    m_gamepadConfig.zAxisGlobalId = loadIntValue(map, QStringLiteral("gamepad/zAxis"), m_gamepadConfig.zAxisGlobalId);
    m_gamepadConfig.zSoftLimitsEnabled =
        loadBoolValue(map, QStringLiteral("gamepad/zSoftLimitsEnabled"), m_gamepadConfig.zSoftLimitsEnabled);
    m_gamepadConfig.zMinUm = loadDoubleValue(map, QStringLiteral("gamepad/zMinUm"), m_gamepadConfig.zMinUm);
    m_gamepadConfig.zMaxUm = loadDoubleValue(map, QStringLiteral("gamepad/zMaxUm"), m_gamepadConfig.zMaxUm);
    m_gamepadConfig.triggerAxisGlobalId =
        loadIntValue(map, QStringLiteral("gamepad/triggerAxis"), m_gamepadConfig.triggerAxisGlobalId);
    m_gamepadConfig.triggerOutputPort =
        qBound(1, loadIntValue(map, QStringLiteral("gamepad/triggerOutputPort"), m_gamepadConfig.triggerOutputPort), 2);
    m_gamepadConfig.triggerPulseMs =
        qBound(1, loadIntValue(map, QStringLiteral("gamepad/triggerPulseMs"), m_gamepadConfig.triggerPulseMs), 1000);

    if (m_gamepadConfig.zMaxUm < m_gamepadConfig.zMinUm)
        std::swap(m_gamepadConfig.zMinUm, m_gamepadConfig.zMaxUm);
    updateGamepadDevice();
}

void thorlabsKinesisPlugin::showSettingsWindow()
{
}

BDCStage* thorlabsKinesisPlugin::m30xyForBase(const QString& baseSerial)
{
    std::lock_guard<std::mutex> lock(m_deviceMapMutex);
    const std::string key = baseSerial.toStdString();
    auto it = m_m30xy.find(key);
    if (it == m_m30xy.end())
        it = m_m30xy.emplace(key, std::make_unique<BDCStage>(key)).first;
    return it->second.get();
}

KVSStage* thorlabsKinesisPlugin::kvsForSerial(const QString& serial)
{
    std::lock_guard<std::mutex> lock(m_deviceMapMutex);
    const std::string key = serial.toStdString();
    auto it = m_kvs.find(key);
    if (it == m_kvs.end())
        it = m_kvs.emplace(key, std::make_unique<KVSStage>(key)).first;
    return it->second.get();
}

bool thorlabsKinesisPlugin::isAxisUiBusy(int id) const
{
    return m_busyAxisIndices.contains(id) || m_axisMotionThreads.contains(id);
}

void thorlabsKinesisPlugin::setAxisControlsBusy(int id, bool busy)
{
    if (id < 0 || id >= m_axisUi.size())
        return;

    AxisUi& axisUi = m_axisUi[id];
    const bool enabled = isInitialized() && !busy;

    if (axisUi.homeButton) axisUi.homeButton->setEnabled(enabled);
    if (axisUi.moveButton) axisUi.moveButton->setEnabled(enabled);
    if (axisUi.stepDownButton) axisUi.stepDownButton->setEnabled(enabled);
    if (axisUi.stepUpButton) axisUi.stepUpButton->setEnabled(enabled);
    if (axisUi.positionEdit) axisUi.positionEdit->setEnabled(enabled);
    if (axisUi.stepEdit) axisUi.stepEdit->setEnabled(enabled);
    if (axisUi.triggerStartEdit) axisUi.triggerStartEdit->setEnabled(enabled);
    if (axisUi.triggerIntervalEdit) axisUi.triggerIntervalEdit->setEnabled(enabled);
    if (axisUi.triggerCountEdit) axisUi.triggerCountEdit->setEnabled(enabled);
    if (axisUi.triggerWidthEdit) axisUi.triggerWidthEdit->setEnabled(enabled);
    if (axisUi.triggerVelocityEdit) axisUi.triggerVelocityEdit->setEnabled(enabled);
    if (axisUi.applyTriggerButton) axisUi.applyTriggerButton->setEnabled(enabled);
    if (axisUi.disableTriggerButton) axisUi.disableTriggerButton->setEnabled(enabled);
    if (axisUi.triggerMenuButton) axisUi.triggerMenuButton->setEnabled(enabled);
    if (axisUi.stopButton) axisUi.stopButton->setEnabled(isInitialized());

    refreshAxisStatusUi(id);
}

void thorlabsKinesisPlugin::setMotionUiBusy(bool busy)
{
    const bool anyBusy = busy || m_motionTaskActive.load()
        || !m_axisMotionThreads.isEmpty()
        || !m_gamepadActiveJogs.isEmpty();

    ui.pushButton_detect->setEnabled(!anyBusy);
    ui.comboBox_devices->setEnabled(!anyBusy);
    ui.pushButton_home->setEnabled(!anyBusy);
    ui.pushButton_position->setEnabled(!anyBusy);
    ui.pushButton_stepUp->setEnabled(!anyBusy);
    ui.pushButton_stepDown->setEnabled(!anyBusy);
    ui.pushButton_applyTrigger->setEnabled(!anyBusy);
    ui.pushButton_disableTrigger->setEnabled(!anyBusy);
    ui.pushButton_stop->setEnabled(anyBusy || isInitialized());
    ui.pushButton_positionManager->setEnabled(!anyBusy && isInitialized());
    ui.pushButton_gamepadSettings->setEnabled(!anyBusy);

    for (int id = 0; id < m_axisUi.size(); ++id)
        setAxisControlsBusy(id, busy || isAxisUiBusy(id));
}

void thorlabsKinesisPlugin::startMotionTask(const QString& operation, std::function<bool()> task)
{
    if (m_motionThread)
    {
        QMessageBox::warning(dock, "Thorlabs", "A motion task is already running.");
        return;
    }

    m_stopRequested.store(false);
    m_motionTaskActive.store(true);
    setMotionUiBusy(true);

    QThread* thread = QThread::create([this, operation, task = std::move(task)]() mutable {
        bool ok = false;
        try
        {
            ok = task();
        }
        catch (...)
        {
            qDebug() << className << "- unhandled exception in" << operation;
        }

        if (!ok)
        {
            QMetaObject::invokeMethod(this, [this, operation]() {
                QMessageBox::warning(dock, "Thorlabs", operation + " failed.");
            }, Qt::QueuedConnection);
        }
    });

    m_motionThread = thread;
    connect(thread, &QThread::finished, this, [this, thread]() {
        if (m_motionThread == thread)
        {
            m_motionThread = nullptr;
            m_motionTaskActive.store(false);
            setMotionUiBusy(false);
            refreshAllAxisPositionsUi();
        }
        thread->deleteLater();
    });
    thread->start();
}

void thorlabsKinesisPlugin::startAxisMotionTask(int axisIndex, const QString& operation,
    std::function<bool()> task, bool referencing)
{
    if (axisIndex < 0 || axisIndex >= m_axes.size())
        return;

    if (m_axisMotionThreads.contains(axisIndex))
    {
        QMessageBox::warning(dock, "Thorlabs", "This axis is already busy.");
        return;
    }

    m_stopRequested.store(false);
    m_busyAxisIndices.insert(axisIndex);
    if (referencing)
        m_referencingAxisIndices.insert(axisIndex);

    QThread* thread = QThread::create([this, operation, task = std::move(task)]() mutable {
        bool ok = false;
        try
        {
            ok = task();
        }
        catch (...)
        {
            qDebug() << className << "- unhandled exception in" << operation;
        }

        if (!ok)
        {
            QMetaObject::invokeMethod(this, [this, operation]() {
                QMessageBox::warning(dock, "Thorlabs", operation + " failed.");
            }, Qt::QueuedConnection);
        }
    });

    m_axisMotionThreads.insert(axisIndex, thread);
    setMotionUiBusy(false);

    connect(thread, &QThread::finished, this, [this, thread, axisIndex]() {
        if (m_axisMotionThreads.value(axisIndex, nullptr) == thread)
            m_axisMotionThreads.remove(axisIndex);
        m_busyAxisIndices.remove(axisIndex);
        m_referencingAxisIndices.remove(axisIndex);
        refreshAxisPositionUi(axisIndex);
        setAxisControlsBusy(axisIndex, false);
        setMotionUiBusy(false);
        thread->deleteLater();
    });

    thread->start();
}

void thorlabsKinesisPlugin::waitForMotionToFinish()
{
    QThread* thread = m_motionThread;
    if (thread)
    {
        stop();
        while (!thread->wait(100))
            stop();
        disconnect(thread, nullptr, this, nullptr);
        if (m_motionThread == thread) m_motionThread = nullptr;
        m_motionTaskActive.store(false);
        delete thread;
    }

    const QList<QThread*> axisThreads = m_axisMotionThreads.values();
    if (!axisThreads.isEmpty())
        stop();

    for (QThread* axisThread : axisThreads)
    {
        if (!axisThread)
            continue;
        while (!axisThread->wait(100))
            stop();
        disconnect(axisThread, nullptr, this, nullptr);
        delete axisThread;
    }

    m_axisMotionThreads.clear();
    m_busyAxisIndices.clear();
    m_referencingAxisIndices.clear();
    if (dock)
        setMotionUiBusy(false);
}

bool thorlabsKinesisPlugin::waitUntilAllAxesStopped(int timeoutMs)
{
    const auto t0 = std::chrono::steady_clock::now();
    while (true)
    {
        if (isStopped())
            return true;

        const auto now = std::chrono::steady_clock::now();
        const int elapsed = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count());
        if (elapsed > timeoutMs)
            return false;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void thorlabsKinesisPlugin::closeDevices()
{
    std::lock_guard<std::mutex> lock(m_deviceMapMutex);
    m_m30xy.clear();
    m_kvs.clear();
}

bool thorlabsKinesisPlugin::disableAllTriggers(bool force)
{
    std::lock_guard<std::mutex> lock(m_deviceMapMutex);
    bool ok = true;

    for (auto& entry : m_m30xy)
    {
        BDCStage* stage = entry.second.get();
        if (!stage || !stage->isOpen())
            continue;

        short err = 0;
        if (!stage->disableTrigger(1, force, &err)) ok = false;
        if (!stage->disableTrigger(2, force, &err)) ok = false;
    }

    for (auto& entry : m_kvs)
    {
        KVSStage* stage = entry.second.get();
        if (!stage || !stage->isOpen())
            continue;

        short err = 0;
        if ((force || stage->triggerConfig().enabled) && !stage->disableTrigger(&err)) ok = false;
    }

    return ok;
}

QString thorlabsKinesisPlugin::axisKey(const AxisEntry& axis) const
{
    return QString("%1:%2:%3")
        .arg(axis.isM30xy ? QStringLiteral("M30XY") : QStringLiteral("KVS"))
        .arg(axis.baseSerial)
        .arg(axis.channel);
}

QString thorlabsKinesisPlugin::axisDisplayText(const AxisEntry& axis) const
{
    if (axis.globalAxisId > 0)
        return QString("#%1  %2").arg(axis.globalAxisId).arg(axis.display);
    return axis.display;
}

void thorlabsKinesisPlugin::assignGlobalAxisIds(QVector<AxisEntry>& axes) const
{
    for (int i = 0; i < axes.size(); ++i)
        axes[i].globalAxisId = i + 1;
}

void thorlabsKinesisPlugin::refreshAxisCombo()
{
    QSignalBlocker blocker(ui.comboBox_devices);
    ui.comboBox_devices->clear();
    for (const AxisEntry& axis : m_axes)
        ui.comboBox_devices->addItem(axisDisplayText(axis));
}

int thorlabsKinesisPlugin::axisIndexFromGlobalId(int globalAxisId) const
{
    if (globalAxisId < 1)
        return -1;

    for (int i = 0; i < m_axes.size(); ++i)
    {
        if (m_axes[i].globalAxisId == globalAxisId)
            return i;
    }
    return -1;
}

int thorlabsKinesisPlugin::axisIndexFromPublicId(int id) const
{
    // Legacy IStage default: id=0 means first stage. Backend/global IDs are
    // QMotion-style and start at 1.
    if (id == 0)
        return m_axes.isEmpty() ? -1 : 0;
    return axisIndexFromGlobalId(id);
}

bool thorlabsKinesisPlugin::resolveAxisRequest(const char* axes, QVector<int>& axisIndices) const
{
    axisIndices.clear();

    QString req = axes ? QString::fromLatin1(axes).trimmed() : QString();
    if (req.isEmpty())
    {
        for (int i = 0; i < m_axes.size(); ++i)
            axisIndices.append(i);
        return !axisIndices.isEmpty();
    }

    const QString lower = req.toLower();
    bool hasAxisLetters = false;
    bool hasDigits = false;
    for (const QChar ch : lower)
    {
        if (ch == QChar('x') || ch == QChar('y') || ch == QChar('z'))
            hasAxisLetters = true;
        else if (ch.isDigit())
            hasDigits = true;
    }

    QSet<int> seen;
    if (hasAxisLetters && !hasDigits)
    {
        for (const QChar ch : lower)
        {
            if (ch.isSpace() || ch == QChar(',') || ch == QChar(';'))
                continue;

            const int axisIndex = findAxisIndexByName(ch);
            if (axisIndex < 0 || seen.contains(axisIndex))
                return false;
            seen.insert(axisIndex);
            axisIndices.append(axisIndex);
        }
        return !axisIndices.isEmpty();
    }

    for (const QChar ch : lower)
    {
        if (ch.isSpace() || ch == QChar(',') || ch == QChar(';'))
            continue;
        if (!ch.isDigit())
            return false;

        const int globalAxisId = ch.digitValue();
        const int axisIndex = axisIndexFromGlobalId(globalAxisId);
        if (axisIndex < 0 || seen.contains(axisIndex))
            return false;
        seen.insert(axisIndex);
        axisIndices.append(axisIndex);
    }

    return !axisIndices.isEmpty();
}

bool thorlabsKinesisPlugin::axisTravelLimitsUm(const AxisEntry& axis, double& minUm, double& maxUm) const
{
    if (axis.isM30xy)
    {
        minUm = -15000.0;
        maxUm = 15000.0;
        return true;
    }

    minUm = 0.0;
    maxUm = 30000.0;
    return true;
}

void thorlabsKinesisPlugin::ensureGamepadDefaults()
{
    auto defaultAxisId = [this](QChar axisName) -> int {
        const int axisIndex = findAxisIndexByName(axisName);
        return axisIndex >= 0 ? m_axes[axisIndex].globalAxisId : 0;
    };

    const int xAxisId = defaultAxisId(QChar('x'));
    const int yAxisId = defaultAxisId(QChar('y'));
    const int zAxisId = defaultAxisId(QChar('z'));

    if (m_gamepadConfig.schemaVersion < 2)
    {
        const GamepadDirectionBinding& oldUp =
            m_gamepadConfig.directionBindings[kGamepadDirectionUp];
        const GamepadDirectionBinding& oldDown =
            m_gamepadConfig.directionBindings[kGamepadDirectionDown];
        const GamepadDirectionBinding& oldLeft =
            m_gamepadConfig.directionBindings[kGamepadDirectionLeft];
        const GamepadDirectionBinding& oldRight =
            m_gamepadConfig.directionBindings[kGamepadDirectionRight];
        const bool legacyDefaultDirections = xAxisId > 0 && yAxisId > 0
            && oldUp.globalAxisId == yAxisId && oldUp.sign > 0
            && oldDown.globalAxisId == yAxisId && oldDown.sign < 0
            && oldLeft.globalAxisId == xAxisId && oldLeft.sign < 0
            && oldRight.globalAxisId == xAxisId && oldRight.sign > 0;

        if (legacyDefaultDirections)
        {
            m_gamepadConfig.directionBindings[kGamepadDirectionUp] = { xAxisId, +1 };
            m_gamepadConfig.directionBindings[kGamepadDirectionDown] = { xAxisId, -1 };
            m_gamepadConfig.directionBindings[kGamepadDirectionLeft] = { yAxisId, -1 };
            m_gamepadConfig.directionBindings[kGamepadDirectionRight] = { yAxisId, +1 };

            if (m_gamepadConfig.triggerAxisGlobalId == zAxisId)
                m_gamepadConfig.triggerAxisGlobalId = xAxisId;
        }
        m_gamepadConfig.schemaVersion = 2;
    }

    if (m_gamepadConfig.schemaVersion < 3)
    {
        GamepadDirectionBinding& left =
            m_gamepadConfig.directionBindings[kGamepadDirectionLeft];
        GamepadDirectionBinding& right =
            m_gamepadConfig.directionBindings[kGamepadDirectionRight];
        if (yAxisId > 0
            && left.globalAxisId == yAxisId && left.sign < 0
            && right.globalAxisId == yAxisId && right.sign > 0)
        {
            left.sign = +1;
            right.sign = -1;
        }
        m_gamepadConfig.schemaVersion = 3;
    }

    if (m_gamepadConfig.schemaVersion < 4)
    {
        constexpr double oldDefaultZMinUm = 0.0;
        constexpr double oldDefaultZMaxUm = 30000.0;
        constexpr double newDefaultZMinUm = 10000.0;
        constexpr double newDefaultZMaxUm = 29700.0;
        if (std::abs(m_gamepadConfig.zMinUm - oldDefaultZMinUm) < 0.5
            && std::abs(m_gamepadConfig.zMaxUm - oldDefaultZMaxUm) < 0.5)
        {
            m_gamepadConfig.zMinUm = newDefaultZMinUm;
            m_gamepadConfig.zMaxUm = newDefaultZMaxUm;
        }
        m_gamepadConfig.schemaVersion = 4;
    }

    auto ensureBinding = [&](int direction, int axisId, int sign) {
        GamepadDirectionBinding& binding = m_gamepadConfig.directionBindings[direction];
        if (binding.globalAxisId <= 0 || axisIndexFromGlobalId(binding.globalAxisId) < 0)
            binding.globalAxisId = axisId;
        binding.sign = binding.sign < 0 ? -1 : 1;
        if (axisId > 0 && binding.globalAxisId == axisId)
            binding.sign = sign;
    };

    ensureBinding(kGamepadDirectionUp, xAxisId, +1);
    ensureBinding(kGamepadDirectionDown, xAxisId, -1);
    ensureBinding(kGamepadDirectionLeft, yAxisId, +1);
    ensureBinding(kGamepadDirectionRight, yAxisId, -1);

    if (m_gamepadConfig.zAxisGlobalId <= 0
        || axisIndexFromGlobalId(m_gamepadConfig.zAxisGlobalId) < 0)
    {
        m_gamepadConfig.zAxisGlobalId = zAxisId;
    }

    if (m_gamepadConfig.triggerAxisGlobalId <= 0
        || axisIndexFromGlobalId(m_gamepadConfig.triggerAxisGlobalId) < 0)
    {
        m_gamepadConfig.triggerAxisGlobalId = xAxisId > 0
            ? xAxisId
            : (zAxisId > 0
                ? zAxisId
                : (m_axes.isEmpty() ? 0 : m_axes.front().globalAxisId));
    }

    m_gamepadConfig.deadzone = qBound(0.05, m_gamepadConfig.deadzone, 0.90);
    m_gamepadConfig.baseJogVelocityMmS =
        qBound(0.01, m_gamepadConfig.baseJogVelocityMmS, kGamepadKvsMaxJogVelocityMmS);
    m_gamepadConfig.fastMultiplier = qBound(1.0, m_gamepadConfig.fastMultiplier, 10.0);
    m_gamepadConfig.slowMultiplier = qBound(0.05, m_gamepadConfig.slowMultiplier, 1.0);
    m_gamepadConfig.triggerOutputPort = qBound(1, m_gamepadConfig.triggerOutputPort, 2);
    m_gamepadConfig.triggerPulseMs = qBound(1, m_gamepadConfig.triggerPulseMs, 1000);

    if (m_gamepadConfig.zMaxUm < m_gamepadConfig.zMinUm)
        std::swap(m_gamepadConfig.zMinUm, m_gamepadConfig.zMaxUm);
}

void thorlabsKinesisPlugin::updateGamepadDevice()
{
    if (!m_gamepadInput)
        m_gamepadInput = std::make_unique<WindowsGamepadInput>();

    const quintptr windowHandle = dock && dock->window()
        ? static_cast<quintptr>(dock->window()->winId())
        : 0;
    const QVector<WindowsGamepadInput::DeviceInfo> devices =
        m_gamepadInput->refreshDevices(windowHandle);

    if (m_gamepadConfig.deviceKey.isEmpty() && !devices.isEmpty())
        m_gamepadConfig.deviceKey = devices.front().key;

    const auto selected = std::find_if(devices.cbegin(), devices.cend(),
        [this](const WindowsGamepadInput::DeviceInfo& device) {
            return device.key == m_gamepadConfig.deviceKey;
        });
    if (selected == devices.cend())
    {
        stopGamepadJogs(true, true);
        m_gamepadInput->clearSelection();
        m_gamepadSuppressUntilNeutral = true;
        refreshGamepadControlBar();
        return;
    }

    if (m_gamepadInput->selectedDeviceKey() != selected->key)
    {
        m_gamepadInput->selectDevice(selected->key, windowHandle);
        m_gamepadTriggerWasPressed = false;
        m_gamepadSuppressUntilNeutral = true;
    }

    m_gamepadReconnectPolls = 0;
    refreshGamepadControlBar();
}

void thorlabsKinesisPlugin::clearGamepadJogState()
{
    m_gamepadActiveJogs.clear();
    m_gamepadRestartNotBefore.clear();
    m_gamepadTriggerWasPressed = false;
    m_gamepadSuppressUntilNeutral = false;
}

bool thorlabsKinesisPlugin::isGamepadZJogAllowed(int axisIndex, int sign) const
{
    if (axisIndex < 0 || axisIndex >= m_axes.size())
        return false;
    if (sign == 0)
        return true;

    const AxisEntry& axis = m_axes[axisIndex];
    const bool configuredZAxis =
        axis.globalAxisId == m_gamepadConfig.zAxisGlobalId;
    const bool enforceConfiguredLimits =
        configuredZAxis && m_gamepadConfig.zSoftLimitsEnabled;
    if (axis.isM30xy && !enforceConfiguredLimits)
        return true;

    double minUm = 0.0;
    double maxUm = 0.0;
    if (!axisTravelLimitsUm(axis, minUm, maxUm))
        return false;
    if (enforceConfiguredLimits)
    {
        minUm = (std::max)(minUm, m_gamepadConfig.zMinUm);
        maxUm = (std::min)(maxUm, m_gamepadConfig.zMaxUm);
    }

    double posUm = 0.0;
    if (!readAxisPositionUm(axisIndex, posUm) || !std::isfinite(posUm))
        return false;

    // KVS gamepad motion is a normal position move whose target is the active
    // soft limit. The controller therefore owns the complete deceleration
    // profile and cannot run past that target; a velocity-based braking margin
    // would unnecessarily block short but valid Z ranges.
    if (maxUm - minUm <= 2.0 * kGamepadLimitToleranceUm)
        return false;
    if (sign > 0)
        return posUm < maxUm - kGamepadLimitToleranceUm;
    if (sign < 0)
        return posUm > minUm + kGamepadLimitToleranceUm;
    return true;
}

bool thorlabsKinesisPlugin::startGamepadJogAxis(int axisIndex, int sign, double velocityMmS)
{
    if (!isInitialized()
        || axisIndex < 0
        || axisIndex >= m_axes.size()
        || sign == 0
        || m_motionTaskActive.load()
        || !m_axisMotionThreads.isEmpty())
    {
        return false;
    }

    AxisEntry& axis = m_axes[axisIndex];
    const auto showKvsGamepadStatus = [this, &axis](const QString& detail) {
        if (!axis.isM30xy && ui.lineEdit_gamepadStatus)
            ui.lineEdit_gamepadStatus->setText(
                QStringLiteral("%1: %2").arg(axisDisplayText(axis), detail));
    };
    const double maxVelocityMmS = axis.isM30xy
        ? kGamepadM30MaxJogVelocityMmS
        : kGamepadKvsMaxJogVelocityMmS;
    const double clampedVelocityMmS = qBound(0.01, velocityMmS, maxVelocityMmS);

    const auto restartIt = m_gamepadRestartNotBefore.constFind(axisIndex);
    if (restartIt != m_gamepadRestartNotBefore.cend()
        && std::chrono::steady_clock::now() < restartIt.value())
    {
        return false;
    }

    bool moving = false;
    bool homed = false;
    if (!readAxisMotionState(axisIndex, moving, homed))
    {
        showKvsGamepadStatus(QStringLiteral("status read failed"));
        return false;
    }
    if (moving)
    {
        showKvsGamepadStatus(QStringLiteral("already moving"));
        return false;
    }
    if (!homed)
    {
        showKvsGamepadStatus(QStringLiteral("not homed"));
        return false;
    }
    m_gamepadRestartNotBefore.remove(axisIndex);
    if (!isGamepadZJogAllowed(axisIndex, sign))
    {
        showKvsGamepadStatus(QStringLiteral("at configured limit"));
        return false;
    }

    short err = 0;
    bool ok = false;
    if (axis.isM30xy)
    {
        BDCStage* stage = m30xyForBase(axis.baseSerial);
        ok = stage
            && stage->isOpen()
            && disableAllTriggers()
            && stage->configureContinuousJog(static_cast<unsigned>(axis.channel), clampedVelocityMmS, &err)
            && stage->moveJog(static_cast<unsigned>(axis.channel), sign > 0, &err);
    }
    else
    {
        KVSStage* stage = kvsForSerial(axis.baseSerial);
        double minUm = 0.0;
        double maxUm = 0.0;
        if (stage && stage->isOpen() && axisTravelLimitsUm(axis, minUm, maxUm))
        {
            const bool configuredZAxis =
                axis.globalAxisId == m_gamepadConfig.zAxisGlobalId;
            if (configuredZAxis && m_gamepadConfig.zSoftLimitsEnabled)
            {
                minUm = (std::max)(minUm, m_gamepadConfig.zMinUm);
                maxUm = (std::min)(maxUm, m_gamepadConfig.zMaxUm);
            }

            const double targetUm = sign > 0 ? maxUm : minUm;
            const int32_t targetDevice = stage->umToDevice(targetUm, &err);
            if (err != 0)
            {
                showKvsGamepadStatus(
                    QStringLiteral("target conversion rejected (error %1)").arg(err));
            }
            else if (!disableAllTriggers())
            {
                showKvsGamepadStatus(QStringLiteral("trigger disable failed"));
            }
            else if (!stage->setMaxVelocityMmS(clampedVelocityMmS, &err))
            {
                showKvsGamepadStatus(
                    QStringLiteral("velocity rejected (error %1)").arg(err));
            }
            else if (!stage->beginMoveTo(targetDevice, &err))
            {
                showKvsGamepadStatus(
                    QStringLiteral("move command rejected (error %1)").arg(err));
            }
            else
            {
                ok = true;
            }

            qDebug() << "Gamepad KVS hold-to-move"
                << "axis=" << axis.globalAxisId
                << "sign=" << sign
                << "targetUm=" << targetUm
                << "velocityMmS=" << clampedVelocityMmS
                << "err=" << err
                << "started=" << ok;
        }
    }

    if (!ok)
    {
        m_gamepadRestartNotBefore[axisIndex] = std::chrono::steady_clock::now()
            + std::chrono::milliseconds(kGamepadProfiledStopStatusDelayMs);
        return false;
    }

    showKvsGamepadStatus(QStringLiteral("active"));
    m_gamepadActiveJogs[axisIndex] = { sign, clampedVelocityMmS };
    m_busyAxisIndices.insert(axisIndex);
    refreshAxisStatusUi(axisIndex);
    setAxisControlsBusy(axisIndex, true);
    setMotionUiBusy(false);
    return true;
}

void thorlabsKinesisPlugin::stopGamepadJogAxis(
    int axisIndex, bool sendStop, bool immediate)
{
    if (!m_gamepadActiveJogs.contains(axisIndex))
        return;

    if (sendStop)
    {
        bool stopped = false;
        bool profiledStop = false;
        if (!immediate && axisIndex >= 0 && axisIndex < m_axes.size()
            && !m_axes[axisIndex].isM30xy)
        {
            KVSStage* stage = kvsForSerial(m_axes[axisIndex].baseSerial);
            short err = 0;
            stopped = stage && stage->isOpen() && stage->stopProfiled(&err);
            profiledStop = stopped;
        }
        if (!stopped)
            stopAxisIndex(axisIndex);

        if (profiledStop)
        {
            m_gamepadRestartNotBefore[axisIndex] = std::chrono::steady_clock::now()
                + std::chrono::milliseconds(kGamepadProfiledStopStatusDelayMs);
        }
        else
        {
            m_gamepadRestartNotBefore.remove(axisIndex);
        }
    }
    else
    {
        m_gamepadRestartNotBefore.remove(axisIndex);
    }

    m_gamepadActiveJogs.remove(axisIndex);
    m_busyAxisIndices.remove(axisIndex);
    refreshAxisPositionUi(axisIndex);
    setAxisControlsBusy(axisIndex, false);
    setMotionUiBusy(false);
}

void thorlabsKinesisPlugin::stopGamepadJogs(bool sendStop, bool immediate)
{
    const QList<int> activeAxes = m_gamepadActiveJogs.keys();
    for (int axisIndex : activeAxes)
        stopGamepadJogAxis(axisIndex, sendStop, immediate);
}

bool thorlabsKinesisPlugin::pulseGamepadTrigger()
{
    if (!isInitialized())
        return false;

    const int axisIndex = axisIndexFromGlobalId(m_gamepadConfig.triggerAxisGlobalId);
    if (axisIndex < 0 || axisIndex >= m_axes.size())
        return false;

    const unsigned outputPort = static_cast<unsigned>(m_gamepadConfig.triggerOutputPort);
    const AxisEntry axis = m_axes[axisIndex];
    short err = 0;
    bool ok = false;

    if (axis.isM30xy)
    {
        BDCStage* stage = m30xyForBase(axis.baseSerial);
        ok = stage
            && stage->isOpen()
            && stage->configureTriggerOutputGpo(static_cast<unsigned>(axis.channel), outputPort, &err)
            && stage->setDigitalOutput(static_cast<unsigned>(axis.channel), outputPort, true, &err);
    }
    else
    {
        KVSStage* stage = kvsForSerial(axis.baseSerial);
        ok = stage
            && stage->isOpen()
            && stage->configureTriggerOutputGpo(outputPort, &err)
            && stage->setDigitalOutput(outputPort, true, &err);
    }

    if (!ok)
        return false;

    QTimer::singleShot(m_gamepadConfig.triggerPulseMs, this, [this, axisIndex, outputPort]() {
        if (axisIndex < 0 || axisIndex >= m_axes.size())
            return;
        const AxisEntry axis = m_axes[axisIndex];
        short err = 0;
        if (axis.isM30xy)
        {
            if (BDCStage* stage = m30xyForBase(axis.baseSerial))
            {
                if (stage->isOpen())
                    stage->setDigitalOutput(static_cast<unsigned>(axis.channel), outputPort, false, &err);
            }
        }
        else
        {
            if (KVSStage* stage = kvsForSerial(axis.baseSerial))
            {
                if (stage->isOpen())
                    stage->setDigitalOutput(outputPort, false, &err);
            }
        }
    });

    return true;
}

void thorlabsKinesisPlugin::pollGamepad()
{
    if (!m_gamepadConfig.enabled)
    {
        stopGamepadJogs(true, true);
        return;
    }

    if (!m_gamepadInput)
        updateGamepadDevice();

    WindowsGamepadInput::State input;
    if (!m_gamepadInput || !m_gamepadInput->poll(input) || !input.connected)
    {
        stopGamepadJogs(true, true);
        m_gamepadSuppressUntilNeutral = true;
        if (++m_gamepadReconnectPolls >= 20)
        {
            m_gamepadReconnectPolls = 0;
            updateGamepadDevice();
        }
        return;
    }
    m_gamepadReconnectPolls = 0;
    if (!isInitialized())
    {
        stopGamepadJogs(true, true);
        return;
    }

    const bool motionBusy = m_motionTaskActive.load() || !m_axisMotionThreads.isEmpty();
    if (motionBusy)
    {
        stopGamepadJogs(true, true);
        m_gamepadSuppressUntilNeutral = true;
        return;
    }

    const auto buttonPressed = [&input](int index) {
        return index >= 0
            && index < WindowsGamepadInput::ButtonCount
            && input.buttons[static_cast<size_t>(index)];
    };

    const double axisX = input.axisX - m_gamepadConfig.axisLeftXCenter;
    const double axisY = input.axisY - m_gamepadConfig.axisLeftYCenter;
    bool up = input.up;
    bool down = input.down;
    bool left = input.left;
    bool right = input.right;

    if (!up && !down && !left && !right)
    {
        const double absX = std::abs(axisX);
        const double absY = std::abs(axisY);
        if (absX > m_gamepadConfig.deadzone || absY > m_gamepadConfig.deadzone)
        {
            if (absX >= absY)
            {
                right = axisX > 0.0;
                left = axisX < 0.0;
            }
            else
            {
                down = axisY > 0.0;
                up = axisY < 0.0;
            }
        }
    }

    const bool zDown = buttonPressed(m_gamepadConfig.zDownButton);
    const bool zUp = buttonPressed(m_gamepadConfig.zUpButton);
    const bool triggerPressed = buttonPressed(m_gamepadConfig.triggerButton);
    const bool slowPressed = buttonPressed(m_gamepadConfig.slowButton);
    const bool fastPressed = buttonPressed(m_gamepadConfig.fastButton);
    const bool anyMappedInput = up || down || left || right || zDown || zUp
        || triggerPressed || slowPressed || fastPressed;
    if (m_gamepadSuppressUntilNeutral)
    {
        if (!anyMappedInput)
            m_gamepadSuppressUntilNeutral = false;
        else
            return;
    }

    if (triggerPressed && !m_gamepadTriggerWasPressed)
        pulseGamepadTrigger();
    m_gamepadTriggerWasPressed = triggerPressed;

    double velocityMmS = m_gamepadConfig.baseJogVelocityMmS;
    if (fastPressed && !slowPressed)
        velocityMmS *= m_gamepadConfig.fastMultiplier;
    else if (slowPressed && !fastPressed)
        velocityMmS *= m_gamepadConfig.slowMultiplier;

    QMap<int, int> desiredSigns;
    const auto addDirection = [&](int direction) {
        const GamepadDirectionBinding& binding = m_gamepadConfig.directionBindings[direction];
        const int axisIndex = axisIndexFromGlobalId(binding.globalAxisId);
        if (axisIndex < 0)
            return;
        const int sign = binding.sign < 0 ? -1 : 1;
        if (desiredSigns.contains(axisIndex) && desiredSigns.value(axisIndex) != sign)
            desiredSigns[axisIndex] = 0;
        else
            desiredSigns[axisIndex] = sign;
    };

    if (up) addDirection(kGamepadDirectionUp);
    if (down) addDirection(kGamepadDirectionDown);
    if (left) addDirection(kGamepadDirectionLeft);
    if (right) addDirection(kGamepadDirectionRight);

    const int zAxisIndex = axisIndexFromGlobalId(m_gamepadConfig.zAxisGlobalId);
    if (zAxisIndex >= 0 && zDown != zUp)
        desiredSigns[zAxisIndex] = zUp ? +1 : -1;
    else if (zAxisIndex >= 0 && zDown && zUp)
        desiredSigns[zAxisIndex] = 0;

    const QList<int> activeAxes = m_gamepadActiveJogs.keys();
    for (int axisIndex : activeAxes)
    {
        const int desiredSign = desiredSigns.value(axisIndex, 0);
        const GamepadActiveJog activeJog = m_gamepadActiveJogs.value(axisIndex);
        if (desiredSign == 0
            || !isGamepadZJogAllowed(axisIndex, desiredSign))
            stopGamepadJogAxis(axisIndex, true);
    }

    for (auto it = desiredSigns.cbegin(); it != desiredSigns.cend(); ++it)
    {
        const int axisIndex = it.key();
        const int desiredSign = it.value();
        if (desiredSign == 0
            || !isGamepadZJogAllowed(axisIndex, desiredSign))
            continue;

        const GamepadActiveJog activeJog = m_gamepadActiveJogs.value(axisIndex);
        if (m_gamepadActiveJogs.contains(axisIndex)
            && activeJog.sign == desiredSign
            && std::abs(activeJog.velocityMmS - velocityMmS) <= kGamepadVelocityChangeTolerance)
        {
            continue;
        }

        if (m_gamepadActiveJogs.contains(axisIndex))
            stopGamepadJogAxis(axisIndex, true);
        startGamepadJogAxis(axisIndex, desiredSign, velocityMmS);
    }
}

bool thorlabsKinesisPlugin::chooseAxesForInitialization()
{
    if (m_detectedAxes.isEmpty())
        return false;

    QDialog dialog(dock);
    dialog.setWindowTitle("Select Thorlabs Axes");
    dialog.setModal(true);

    auto* layout = new QVBoxLayout(&dialog);
    auto* infoLabel = new QLabel(
        "Select the Thorlabs axes that should be initialized and used by this plugin.",
        &dialog);
    infoLabel->setWordWrap(true);
    layout->addWidget(infoLabel);

    auto* list = new QListWidget(&dialog);
    list->setSelectionMode(QAbstractItemView::NoSelection);
    layout->addWidget(list);

    const bool hasPreviousSelection = !m_selectedAxisKeys.isEmpty();
    for (int i = 0; i < m_detectedAxes.size(); ++i)
    {
        const AxisEntry& axis = m_detectedAxes[i];
        auto* item = new QListWidgetItem(axis.display, list);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setData(Qt::UserRole, i);
        const bool checked = !hasPreviousSelection || m_selectedAxisKeys.contains(axisKey(axis));
        item->setCheckState(checked ? Qt::Checked : Qt::Unchecked);
    }

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    layout->addWidget(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    if (dialog.exec() != QDialog::Accepted)
        return false;

    QVector<AxisEntry> selectedAxes;
    QSet<QString> selectedKeys;
    for (int row = 0; row < list->count(); ++row)
    {
        const QListWidgetItem* item = list->item(row);
        if (!item || item->checkState() != Qt::Checked)
            continue;

        const int detectedIndex = item->data(Qt::UserRole).toInt();
        if (detectedIndex < 0 || detectedIndex >= m_detectedAxes.size())
            continue;

        AxisEntry axis = m_detectedAxes[detectedIndex];
        selectedKeys.insert(axisKey(axis));
        selectedAxes.append(axis);
    }

    if (selectedAxes.isEmpty())
    {
        QMessageBox::warning(dock, "Thorlabs", "Please select at least one axis.");
        return false;
    }

    assignGlobalAxisIds(selectedAxes);
    m_selectedAxisKeys = selectedKeys;
    m_axes = selectedAxes;
    ensureGamepadDefaults();
    refreshAxisCombo();
    rebuildAxisFrames();
    ui.comboBox_devices->setCurrentIndex(0);
    refreshAxisUi();
    if (m_positionManagerWindow)
        m_positionManagerWindow->refresh();

    qDebug() << className << "::chooseAxesForInitialization selected axes=" << m_axes.size();
    for (const AxisEntry& axis : m_axes)
    {
        qDebug() << className << "::chooseAxesForInitialization axis"
            << axis.globalAxisId
            << axis.display
            << "serial=" << axis.baseSerial
            << "channel=" << axis.channel;
    }

    return true;
}

void thorlabsKinesisPlugin::clearAxisFrames()
{
    while (QLayoutItem* item = ui.axisFramesLayout->takeAt(0))
    {
        if (QWidget* widget = item->widget())
            delete widget;
        delete item;
    }

    m_axisUi.clear();
}

void thorlabsKinesisPlugin::updateAxisFrameAreaHeight()
{
    if (!ui.stageFrameContainer || !ui.axisFramesLayout)
        return;

    ui.axisFramesLayout->activate();
    const QMargins margins = ui.axisFramesLayout->contentsMargins();
    int height = margins.top() + margins.bottom();
    int visibleWidgets = 0;
    const int spacing = qMax(0, ui.axisFramesLayout->spacing());

    for (int index = 0; index < ui.axisFramesLayout->count(); ++index)
    {
        QLayoutItem* item = ui.axisFramesLayout->itemAt(index);
        QWidget* widget = item ? item->widget() : nullptr;
        if (!widget || widget->isHidden())
            continue;

        height += preferredWidgetHeight(widget);
        ++visibleWidgets;
    }

    if (visibleWidgets > 1)
        height += (visibleWidgets - 1) * spacing;

    ui.stageFrameContainer->setMinimumHeight(height);
    ui.stageFrameContainer->setMaximumHeight(height > 0 ? height : QWIDGETSIZE_MAX);
    ui.stageFrameContainer->updateGeometry();

    if (dock && dock->widget() && dock->widget()->layout())
        dock->widget()->layout()->activate();
    if (dock)
    {
        const int dockHeight = qMax(90, dock->minimumSizeHint().height());
        dock->setMinimumHeight(dockHeight);
        dock->resize(qMax(dock->width(), dock->minimumWidth()), dockHeight);
        dock->updateGeometry();
    }
}

void thorlabsKinesisPlugin::selectAxis(int id)
{
    if (id < 0 || id >= m_axes.size())
        return;

    QSignalBlocker blocker(ui.comboBox_devices);
    ui.comboBox_devices->setCurrentIndex(id);
    refreshAxisUi();
}

void thorlabsKinesisPlugin::syncLegacyMotionInputsFromAxisUi(int id)
{
    if (id < 0 || id >= m_axisUi.size())
        return;

    const AxisUi& axisUi = m_axisUi[id];
    if (axisUi.positionEdit)
        ui.lineEdit_position->setText(QString::number(axisUi.positionEdit->value() * 1000.0, 'f', 3));

    if (axisUi.stepEdit)
        ui.lineEdit_step->setText(QString::number(axisUi.stepEdit->value() * 1000.0, 'f', 3));
}

void thorlabsKinesisPlugin::syncLegacyTriggerInputsFromAxisUi(int id)
{
    if (id < 0 || id >= m_axisUi.size())
        return;

    const AxisUi& axisUi = m_axisUi[id];
    if (axisUi.triggerStartEdit)
    {
        double startMm = 0.0;
        if (parseFiniteDouble(axisUi.triggerStartEdit->text(), startMm))
            ui.lineEdit_trigStart->setText(QString::number(startMm * 1000.0, 'f', 3));
        else
            ui.lineEdit_trigStart->setText(axisUi.triggerStartEdit->text());
    }
    if (axisUi.triggerIntervalEdit)
    {
        double intervalMm = 0.0;
        if (parseFiniteDouble(axisUi.triggerIntervalEdit->text(), intervalMm))
            ui.lineEdit_trigInterval->setText(QString::number(intervalMm * 1000.0, 'f', 3));
        else
            ui.lineEdit_trigInterval->setText(axisUi.triggerIntervalEdit->text());
    }
    if (axisUi.triggerCountEdit)
        ui.lineEdit_trigCount->setText(axisUi.triggerCountEdit->text());
    if (axisUi.triggerWidthEdit)
        ui.lineEdit_trigWidth->setText(axisUi.triggerWidthEdit->text());
}

void thorlabsKinesisPlugin::refreshAxisPositionUi(int id)
{
    if (id < 0 || id >= m_axisUi.size())
        return;

    double positionUm = 0.0;
    const bool hasPosition = readAxisPositionUm(id, positionUm);
    const QString text = hasPosition
        ? mmDisplayWithUnitFromUm(positionUm)
        : QStringLiteral("n/a");

    if (m_axisUi[id].positionLcd)
    {
        if (hasPosition)
            m_axisUi[id].positionLcd->display(positionUm / 1000.0);
        else
            m_axisUi[id].positionLcd->display(QStringLiteral("--------"));
    }

    if (ui.comboBox_devices->currentIndex() == id)
        ui.label_positionValue->setText(text);

    refreshAxisStatusUi(id);
}

bool thorlabsKinesisPlugin::readAxisMotionState(int id, bool& moving, bool& homed) const
{
    moving = false;
    homed = false;

    if (id < 0 || id >= m_axes.size())
        return false;

    const AxisEntry& axis = m_axes[id];
    short err = 0;

    if (axis.isM30xy)
    {
        auto it = m_m30xy.find(axis.baseSerial.toStdString());
        if (it == m_m30xy.end() || !it->second || !it->second->isOpen())
            return false;
        const BDCStage* stage = it->second.get();
        moving = stage->isMoving(static_cast<unsigned>(axis.channel), &err);
        if (err != 0) return false;
        homed = stage->isHomed(static_cast<unsigned>(axis.channel), &err);
        return err == 0;
    }

    auto it = m_kvs.find(axis.baseSerial.toStdString());
    if (it == m_kvs.end() || !it->second || !it->second->isOpen())
        return false;
    const KVSStage* stage = it->second.get();
    moving = stage->isMoving(&err);
    if (err != 0) return false;
    homed = stage->isHomed(&err);
    return err == 0;
}

void thorlabsKinesisPlugin::refreshAxisStatusUi(int id)
{
    if (id < 0 || id >= m_axisUi.size())
        return;

    QString text = QStringLiteral("not homed");
    QString color = QStringLiteral("#dddddd");

    if (m_referencingAxisIndices.contains(id))
    {
        text = QStringLiteral("referencing");
        color = QStringLiteral("#ffd966");
    }
    else if (m_busyAxisIndices.contains(id))
    {
        text = QStringLiteral("busy");
        color = QStringLiteral("#f4b183");
    }
    else
    {
        bool moving = false;
        bool homed = false;
        if (readAxisMotionState(id, moving, homed))
        {
            if (moving && !homed)
            {
                text = QStringLiteral("referencing");
                color = QStringLiteral("#ffd966");
            }
            else if (moving)
            {
                text = QStringLiteral("busy");
                color = QStringLiteral("#f4b183");
            }
            else if (homed)
            {
                text = QStringLiteral("homed");
                color = QStringLiteral("#93c47d");
            }
        }
        else
        {
            text = QStringLiteral("status n/a");
            color = QStringLiteral("#cccccc");
        }
    }

    if (m_axisUi[id].statusLabel)
    {
        m_axisUi[id].statusLabel->setText(text);
        m_axisUi[id].statusLabel->setStyleSheet(
            QStringLiteral("background-color: %1; color: black; border: none; padding: 2px 6px;")
                .arg(color));
    }
}

void thorlabsKinesisPlugin::updateTriggerFrequencyUi(int id)
{
    if (id < 0 || id >= m_axisUi.size())
        return;

    AxisUi& axisUi = m_axisUi[id];
    if (!axisUi.triggerFrequencyValue)
        return;

    double spacingMm = 0.0;
    double velocityMmS = 0.0;
    const bool ok = axisUi.triggerIntervalEdit
        && axisUi.triggerVelocityEdit
        && parseFiniteDouble(axisUi.triggerIntervalEdit->text(), spacingMm)
        && parseFiniteDouble(axisUi.triggerVelocityEdit->text(), velocityMmS);

    axisUi.triggerFrequencyValue->setText(ok
        ? frequencyDisplayText(velocityMmS, spacingMm)
        : QStringLiteral("n/a"));
}

void thorlabsKinesisPlugin::setupAxisTriggerMenu(AxisUi& axisUi, QWidget* parent)
{
    if (!axisUi.triggerMenuButton || !parent)
        return;

    auto* menu = new QMenu(axisUi.triggerMenuButton);
    menu->setObjectName(QStringLiteral("triggerMenu"));

    auto* panel = new QWidget(menu);
    panel->setObjectName(QStringLiteral("triggerMenuPanel"));
    panel->setStyleSheet(QStringLiteral(
        "QWidget#triggerMenuPanel { background-color: rgb(235, 235, 235); }"
        "QLabel { color: black; }"));

    auto* layout = new QGridLayout(panel);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setHorizontalSpacing(6);
    layout->setVerticalSpacing(4);

    const auto makeLabel = [](const QString& text, QWidget* owner) {
        auto* label = new QLabel(text, owner);
        label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        return label;
    };

    const auto makeEdit = [](const QString& objectName, const QString& text, int width, QWidget* owner) {
        auto* edit = new QLineEdit(owner);
        edit->setObjectName(objectName);
        edit->setText(text);
        edit->setMinimumHeight(22);
        edit->setMaximumSize(QSize(width, 24));
        return edit;
    };

    axisUi.triggerStartEdit = makeEdit(
        QStringLiteral("lineEdit_triggerStart"), QStringLiteral("0,000"), 82, panel);
    axisUi.triggerIntervalEdit = makeEdit(
        QStringLiteral("lineEdit_triggerInterval"), QStringLiteral("0,100"), 82, panel);
    axisUi.triggerCountEdit = makeEdit(
        QStringLiteral("lineEdit_triggerCount"), QStringLiteral("1"), 74, panel);
    axisUi.triggerWidthEdit = makeEdit(
        QStringLiteral("lineEdit_triggerWidth"), QStringLiteral("500"), 82, panel);
    axisUi.triggerVelocityEdit = makeEdit(
        QStringLiteral("lineEdit_triggerVelocity"), QStringLiteral("2,000"), 82, panel);
    axisUi.triggerFrequencyValue = new QLabel(QStringLiteral("n/a"), panel);
    axisUi.triggerFrequencyValue->setObjectName(QStringLiteral("label_triggerFrequency"));
    axisUi.triggerFrequencyValue->setMinimumWidth(70);

    axisUi.applyTriggerButton = new QPushButton(QStringLiteral("Apply Trigger"), panel);
    axisUi.applyTriggerButton->setObjectName(QStringLiteral("pushButton_applyTrigger"));
    axisUi.disableTriggerButton = new QPushButton(QStringLiteral("Disable Trigger"), panel);
    axisUi.disableTriggerButton->setObjectName(QStringLiteral("pushButton_disableTrigger"));

    layout->addWidget(makeLabel(QStringLiteral("Start [mm]:"), panel), 0, 0);
    layout->addWidget(axisUi.triggerStartEdit, 0, 1);
    layout->addWidget(makeLabel(QStringLiteral("Spacing [mm]:"), panel), 0, 2);
    layout->addWidget(axisUi.triggerIntervalEdit, 0, 3);
    layout->addWidget(makeLabel(QStringLiteral("Pulse count:"), panel), 1, 0);
    layout->addWidget(axisUi.triggerCountEdit, 1, 1);
    layout->addWidget(makeLabel(QStringLiteral("Pulse width [us]:"), panel), 1, 2);
    layout->addWidget(axisUi.triggerWidthEdit, 1, 3);
    layout->addWidget(makeLabel(QStringLiteral("Velocity [mm/s]:"), panel), 2, 0);
    layout->addWidget(axisUi.triggerVelocityEdit, 2, 1);
    layout->addWidget(makeLabel(QStringLiteral("Frequency [Hz]:"), panel), 2, 2);
    layout->addWidget(axisUi.triggerFrequencyValue, 2, 3);
    layout->addWidget(axisUi.applyTriggerButton, 3, 0, 1, 2);
    layout->addWidget(axisUi.disableTriggerButton, 3, 2, 1, 2);

    auto* triggerAction = new QWidgetAction(menu);
    triggerAction->setDefaultWidget(panel);
    menu->addAction(triggerAction);

    axisUi.triggerMenuButton->setMenu(menu);
    axisUi.triggerMenuButton->setPopupMode(QToolButton::InstantPopup);
}

void thorlabsKinesisPlugin::setupGamepadControls()
{
    if (!ui.checkBox_gamepadEnabled
        || !ui.lineEdit_gamepadStatus
        || !ui.pushButton_gamepadSettings)
    {
        return;
    }

    ui.gamepadControlLayout->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    ui.checkBox_gamepadEnabled->setFixedHeight(kSerialButtonHeight);
    ui.pushButton_gamepadSettings->setFixedHeight(kSerialButtonHeight);

    connect(ui.pushButton_gamepadSettings, &QPushButton::clicked,
        this, &thorlabsKinesisPlugin::openGamepadConfigDialog, Qt::UniqueConnection);
    connect(ui.checkBox_gamepadEnabled, &QCheckBox::toggled,
        this, [this](bool enabled) {
            if (m_gamepadConfig.enabled == enabled)
                return;

            if (!enabled)
                stopGamepadJogs(true);
            m_gamepadConfig.enabled = enabled;
            m_gamepadSuppressUntilNeutral = enabled;
            updateGamepadDevice();
            refreshGamepadControlBar();
        });

    refreshGamepadControlBar();
}

void thorlabsKinesisPlugin::refreshGamepadControlBar()
{
    if (!ui.checkBox_gamepadEnabled
        || !ui.lineEdit_gamepadStatus
        || !ui.pushButton_gamepadSettings)
    {
        return;
    }

    {
        QSignalBlocker blocker(ui.checkBox_gamepadEnabled);
        ui.checkBox_gamepadEnabled->setChecked(m_gamepadConfig.enabled);
    }

    QString status = QStringLiteral("No gamepad connected");
    if (m_gamepadInput && !m_gamepadInput->selectedDeviceName().isEmpty())
    {
        status = m_gamepadInput->selectedDeviceName();
    }
    else if (!m_gamepadConfig.deviceKey.isEmpty())
    {
        status = QStringLiteral("Configured gamepad not connected");
    }
    ui.lineEdit_gamepadStatus->setText(status);
    ui.lineEdit_gamepadStatus->setEnabled(m_gamepadConfig.enabled);
}

void thorlabsKinesisPlugin::openGamepadConfigDialog()
{
    ensureGamepadDefaults();
    updateGamepadDevice();
    const bool restartPollTimer = m_gamepadPollTimer && m_gamepadPollTimer->isActive();
    if (m_gamepadPollTimer)
        m_gamepadPollTimer->stop();
    stopGamepadJogs(true);

    QDialog dialog(dock);
    dialog.setWindowTitle(QStringLiteral("Gamepad Configuration"));
    dialog.setModal(true);
    dialog.setMinimumWidth(520);
    applyQMotionLikeWidgetStyle(&dialog);

    GamepadConfig editedConfig = m_gamepadConfig;
    double calibratedCenterX = editedConfig.axisLeftXCenter;
    double calibratedCenterY = editedConfig.axisLeftYCenter;

    auto* rootLayout = new QVBoxLayout(&dialog);
    rootLayout->setContentsMargins(8, 8, 8, 8);
    rootLayout->setSpacing(8);

    auto* enableCheck = new QCheckBox(QStringLiteral("Enable gamepad"), &dialog);
    enableCheck->setChecked(editedConfig.enabled);
    rootLayout->addWidget(enableCheck);

    auto* deviceGroup = new QGroupBox(QStringLiteral("Device"), &dialog);
    auto* deviceLayout = new QGridLayout(deviceGroup);
    deviceLayout->setContentsMargins(8, 8, 8, 8);
    deviceLayout->setHorizontalSpacing(6);
    deviceLayout->setVerticalSpacing(4);
    auto* deviceCombo = new QComboBox(deviceGroup);
    auto* deviceStateLabel = new QLabel(deviceGroup);
    auto* neutralLabel = new QLabel(deviceGroup);
    auto* calibrateButton = new QPushButton(QStringLiteral("Calibrate Neutral"), deviceGroup);
    auto* refreshDevicesButton = new QPushButton(QStringLiteral("Refresh Devices"), deviceGroup);
    deviceLayout->addWidget(new QLabel(QStringLiteral("Gamepad:"), deviceGroup), 0, 0);
    deviceLayout->addWidget(deviceCombo, 0, 1, 1, 3);
    deviceLayout->addWidget(deviceStateLabel, 1, 0, 1, 4);
    deviceLayout->addWidget(neutralLabel, 2, 0, 1, 4);
    deviceLayout->addWidget(calibrateButton, 3, 0, 1, 2);
    deviceLayout->addWidget(refreshDevicesButton, 3, 2, 1, 2);
    rootLayout->addWidget(deviceGroup);

    WindowsGamepadInput previewInput;
    WindowsGamepadInput::State previewState;
    const quintptr previewWindowHandle = static_cast<quintptr>(dialog.winId());

    auto populateDeviceCombo = [&]() {
        const QString previousKey = deviceCombo->currentData().toString().isEmpty()
            ? editedConfig.deviceKey
            : deviceCombo->currentData().toString();
        const QVector<WindowsGamepadInput::DeviceInfo> devices =
            previewInput.refreshDevices(previewWindowHandle);

        QSignalBlocker blocker(deviceCombo);
        deviceCombo->clear();
        for (const WindowsGamepadInput::DeviceInfo& device : devices)
            deviceCombo->addItem(device.name, device.key);
        if (devices.isEmpty())
            deviceCombo->addItem(QStringLiteral("No gamepad connected"), QString());

        const int requestedIndex = deviceCombo->findData(previousKey);
        deviceCombo->setCurrentIndex(requestedIndex >= 0 ? requestedIndex : 0);
        const QString selectedKey = deviceCombo->currentData().toString();
        if (selectedKey.isEmpty())
            previewInput.clearSelection();
        else
            previewInput.selectDevice(selectedKey, previewWindowHandle);
    };

    auto updateNeutralLabel = [&]() {
        neutralLabel->setText(QStringLiteral("Neutral: X %1 / Y %2")
            .arg(guiDecimalText(calibratedCenterX, 3))
            .arg(guiDecimalText(calibratedCenterY, 3)));
    };
    auto updateDeviceState = [&]() {
        if (deviceCombo->currentData().toString().isEmpty()
            || !previewInput.poll(previewState)
            || !previewState.connected)
        {
            deviceStateLabel->setText(QStringLiteral("No input from selected gamepad."));
            return;
        }

        QStringList pressedButtons;
        for (int index = 0; index < WindowsGamepadInput::ButtonCount; ++index)
        {
            if (previewState.buttons[static_cast<size_t>(index)])
                pressedButtons.append(QString::number(index + 1));
        }
        deviceStateLabel->setText(QStringLiteral("Input: LX %1 / LY %2  Buttons: %3")
            .arg(guiDecimalText(previewState.axisX, 3))
            .arg(guiDecimalText(previewState.axisY, 3))
            .arg(pressedButtons.isEmpty() ? QStringLiteral("-") : pressedButtons.join(QStringLiteral(", "))));
    };
    populateDeviceCombo();
    updateNeutralLabel();
    updateDeviceState();

    auto* inputTimer = new QTimer(&dialog);
    inputTimer->setInterval(200);
    connect(inputTimer, &QTimer::timeout, &dialog, updateDeviceState);
    inputTimer->start();

    connect(deviceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
        &dialog, [&](int) {
            const QString selectedKey = deviceCombo->currentData().toString();
            if (selectedKey.isEmpty())
                previewInput.clearSelection();
            else
                previewInput.selectDevice(selectedKey, previewWindowHandle);
            updateDeviceState();
        });
    connect(calibrateButton, &QPushButton::clicked, &dialog, [&]() {
        if (!previewInput.poll(previewState) || !previewState.connected)
        {
            QMessageBox::warning(&dialog, "Thorlabs", "No connected gamepad is selected.");
            return;
        }
        calibratedCenterX = previewState.axisX;
        calibratedCenterY = previewState.axisY;
        updateNeutralLabel();
    });
    connect(refreshDevicesButton, &QPushButton::clicked,
        &dialog, [&]() { populateDeviceCombo(); updateDeviceState(); });

    auto* speedGroup = new QGroupBox(QStringLiteral("Speed"), &dialog);
    auto* speedLayout = new QGridLayout(speedGroup);
    speedLayout->setContentsMargins(8, 8, 8, 8);
    speedLayout->setHorizontalSpacing(6);
    speedLayout->setVerticalSpacing(4);
    auto* deadzoneSpin = new QDoubleSpinBox(speedGroup);
    auto* baseVelocitySpin = new QDoubleSpinBox(speedGroup);
    auto* fastMultiplierSpin = new QDoubleSpinBox(speedGroup);
    auto* slowMultiplierSpin = new QDoubleSpinBox(speedGroup);
    for (QDoubleSpinBox* spin : { deadzoneSpin, baseVelocitySpin, fastMultiplierSpin, slowMultiplierSpin })
    {
        spin->setLocale(QLocale(QLocale::German, QLocale::Germany));
        spin->setDecimals(3);
    }
    deadzoneSpin->setRange(0.050, 0.900);
    deadzoneSpin->setSingleStep(0.050);
    deadzoneSpin->setValue(editedConfig.deadzone);
    baseVelocitySpin->setRange(0.010, kGamepadKvsMaxJogVelocityMmS);
    baseVelocitySpin->setSingleStep(0.100);
    baseVelocitySpin->setSuffix(QStringLiteral(" mm/s"));
    baseVelocitySpin->setValue(editedConfig.baseJogVelocityMmS);
    fastMultiplierSpin->setRange(1.000, 10.000);
    fastMultiplierSpin->setSingleStep(0.250);
    fastMultiplierSpin->setValue(editedConfig.fastMultiplier);
    slowMultiplierSpin->setRange(0.050, 1.000);
    slowMultiplierSpin->setSingleStep(0.050);
    slowMultiplierSpin->setValue(editedConfig.slowMultiplier);
    speedLayout->addWidget(new QLabel(QStringLiteral("Deadzone:"), speedGroup), 0, 0);
    speedLayout->addWidget(deadzoneSpin, 0, 1);
    speedLayout->addWidget(new QLabel(QStringLiteral("Base jog:"), speedGroup), 0, 2);
    speedLayout->addWidget(baseVelocitySpin, 0, 3);
    speedLayout->addWidget(new QLabel(QStringLiteral("Fast multiplier:"), speedGroup), 1, 0);
    speedLayout->addWidget(fastMultiplierSpin, 1, 1);
    speedLayout->addWidget(new QLabel(QStringLiteral("Slow multiplier:"), speedGroup), 1, 2);
    speedLayout->addWidget(slowMultiplierSpin, 1, 3);
    rootLayout->addWidget(speedGroup);

    const auto setComboByData = [](QComboBox* combo, int data) {
        const int index = combo ? combo->findData(data) : -1;
        if (index >= 0)
            combo->setCurrentIndex(index);
    };
    const auto fillAxisCombo = [this, &setComboByData](QComboBox* combo, int selectedGlobalId, bool m30Only) {
        combo->addItem(QStringLiteral("None"), 0);
        for (const AxisEntry& axis : m_axes)
        {
            if (m30Only && !axis.isM30xy)
                continue;
            combo->addItem(axisDisplayText(axis), axis.globalAxisId);
        }
        setComboByData(combo, selectedGlobalId);
    };
    const auto makeSignCombo = [&](QWidget* parent, int sign) {
        auto* combo = new QComboBox(parent);
        combo->addItem(QStringLiteral("+"), +1);
        combo->addItem(QStringLiteral("-"), -1);
        setComboByData(combo, sign < 0 ? -1 : +1);
        return combo;
    };
    const auto makeButtonCombo = [&](QWidget* parent, int selectedButton) {
        auto* combo = new QComboBox(parent);
        for (int index = 0; index < WindowsGamepadInput::ButtonCount; ++index)
            combo->addItem(QStringLiteral("Button %1").arg(index + 1), index);
        setComboByData(combo, selectedButton);
        return combo;
    };

    auto* buttonGroup = new QGroupBox(QStringLiteral("Button Mapping"), &dialog);
    auto* buttonLayout = new QGridLayout(buttonGroup);
    buttonLayout->setContentsMargins(8, 8, 8, 8);
    buttonLayout->setHorizontalSpacing(6);
    buttonLayout->setVerticalSpacing(4);
    auto* triggerButtonCombo = makeButtonCombo(buttonGroup, editedConfig.triggerButton);
    auto* slowButtonCombo = makeButtonCombo(buttonGroup, editedConfig.slowButton);
    auto* fastButtonCombo = makeButtonCombo(buttonGroup, editedConfig.fastButton);
    auto* zDownButtonCombo = makeButtonCombo(buttonGroup, editedConfig.zDownButton);
    auto* zUpButtonCombo = makeButtonCombo(buttonGroup, editedConfig.zUpButton);
    buttonLayout->addWidget(new QLabel(QStringLiteral("Trigger (A):"), buttonGroup), 0, 0);
    buttonLayout->addWidget(triggerButtonCombo, 0, 1);
    buttonLayout->addWidget(new QLabel(QStringLiteral("Faster (X):"), buttonGroup), 1, 0);
    buttonLayout->addWidget(fastButtonCombo, 1, 1);
    buttonLayout->addWidget(new QLabel(QStringLiteral("Slower (B):"), buttonGroup), 1, 2);
    buttonLayout->addWidget(slowButtonCombo, 1, 3);
    buttonLayout->addWidget(new QLabel(QStringLiteral("Z down (L1):"), buttonGroup), 2, 0);
    buttonLayout->addWidget(zDownButtonCombo, 2, 1);
    buttonLayout->addWidget(new QLabel(QStringLiteral("Z up (R1):"), buttonGroup), 2, 2);
    buttonLayout->addWidget(zUpButtonCombo, 2, 3);
    rootLayout->addWidget(buttonGroup);

    auto* directionGroup = new QGroupBox(QStringLiteral("XY Directions"), &dialog);
    auto* directionLayout = new QGridLayout(directionGroup);
    directionLayout->setContentsMargins(8, 8, 8, 8);
    directionLayout->setHorizontalSpacing(6);
    directionLayout->setVerticalSpacing(4);
    struct DirectionControls
    {
        QComboBox* axis = nullptr;
        QComboBox* sign = nullptr;
    };
    std::array<DirectionControls, kGamepadDirections> directionControls;
    const QStringList directionNames = {
        QStringLiteral("Up"),
        QStringLiteral("Down"),
        QStringLiteral("Left"),
        QStringLiteral("Right")
    };
    for (int direction = 0; direction < kGamepadDirections; ++direction)
    {
        directionControls[direction].axis = new QComboBox(directionGroup);
        directionControls[direction].sign = makeSignCombo(
            directionGroup, editedConfig.directionBindings[direction].sign);
        fillAxisCombo(directionControls[direction].axis,
            editedConfig.directionBindings[direction].globalAxisId,
            true);
        directionLayout->addWidget(new QLabel(directionNames[direction] + QStringLiteral(":"), directionGroup), direction, 0);
        directionLayout->addWidget(directionControls[direction].axis, direction, 1);
        directionLayout->addWidget(directionControls[direction].sign, direction, 2);
    }
    rootLayout->addWidget(directionGroup);

    auto* zGroup = new QGroupBox(QStringLiteral("Z Jog"), &dialog);
    auto* zLayout = new QGridLayout(zGroup);
    zLayout->setContentsMargins(8, 8, 8, 8);
    zLayout->setHorizontalSpacing(6);
    zLayout->setVerticalSpacing(4);
    auto* zAxisCombo = new QComboBox(zGroup);
    fillAxisCombo(zAxisCombo, editedConfig.zAxisGlobalId, false);
    auto* zSoftLimitCheck = new QCheckBox(QStringLiteral("Softlimits active"), zGroup);
    zSoftLimitCheck->setChecked(editedConfig.zSoftLimitsEnabled);
    auto* zMinSpin = new QDoubleSpinBox(zGroup);
    auto* zMaxSpin = new QDoubleSpinBox(zGroup);
    for (QDoubleSpinBox* spin : { zMinSpin, zMaxSpin })
    {
        spin->setLocale(QLocale(QLocale::German, QLocale::Germany));
        spin->setDecimals(3);
        spin->setRange(-1000.000, 1000.000);
        spin->setSingleStep(0.100);
        spin->setSuffix(QStringLiteral(" mm"));
    }
    zMinSpin->setValue(editedConfig.zMinUm / 1000.0);
    zMaxSpin->setValue(editedConfig.zMaxUm / 1000.0);
    zLayout->addWidget(new QLabel(QStringLiteral("Axis:"), zGroup), 0, 0);
    zLayout->addWidget(zAxisCombo, 0, 1, 1, 3);
    zLayout->addWidget(zSoftLimitCheck, 1, 0, 1, 2);
    zLayout->addWidget(new QLabel(QStringLiteral("z_min:"), zGroup), 2, 0);
    zLayout->addWidget(zMinSpin, 2, 1);
    zLayout->addWidget(new QLabel(QStringLiteral("z_max:"), zGroup), 2, 2);
    zLayout->addWidget(zMaxSpin, 2, 3);
    rootLayout->addWidget(zGroup);

    auto* triggerGroup = new QGroupBox(QStringLiteral("Trigger Output"), &dialog);
    auto* triggerLayout = new QGridLayout(triggerGroup);
    triggerLayout->setContentsMargins(8, 8, 8, 8);
    triggerLayout->setHorizontalSpacing(6);
    triggerLayout->setVerticalSpacing(4);
    auto* triggerAxisCombo = new QComboBox(triggerGroup);
    fillAxisCombo(triggerAxisCombo, editedConfig.triggerAxisGlobalId, false);
    auto* triggerPortCombo = new QComboBox(triggerGroup);
    triggerPortCombo->addItem(QStringLiteral("I/O 1"), 1);
    triggerPortCombo->addItem(QStringLiteral("I/O 2"), 2);
    setComboByData(triggerPortCombo, editedConfig.triggerOutputPort);
    auto* triggerPulseSpin = new QSpinBox(triggerGroup);
    triggerPulseSpin->setRange(1, 1000);
    triggerPulseSpin->setSingleStep(1);
    triggerPulseSpin->setSuffix(QStringLiteral(" ms"));
    triggerPulseSpin->setValue(editedConfig.triggerPulseMs);
    triggerLayout->addWidget(new QLabel(QStringLiteral("Axis:"), triggerGroup), 0, 0);
    triggerLayout->addWidget(triggerAxisCombo, 0, 1, 1, 3);
    triggerLayout->addWidget(new QLabel(QStringLiteral("Output:"), triggerGroup), 1, 0);
    triggerLayout->addWidget(triggerPortCombo, 1, 1);
    triggerLayout->addWidget(new QLabel(QStringLiteral("Pulse:"), triggerGroup), 1, 2);
    triggerLayout->addWidget(triggerPulseSpin, 1, 3);
    rootLayout->addWidget(triggerGroup);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    rootLayout->addWidget(buttons);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, [&]() {
        GamepadConfig newConfig = editedConfig;
        newConfig.enabled = enableCheck->isChecked();
        newConfig.deviceKey = deviceCombo->currentData().toString();
        newConfig.axisLeftXCenter = calibratedCenterX;
        newConfig.axisLeftYCenter = calibratedCenterY;
        newConfig.deadzone = deadzoneSpin->value();
        newConfig.baseJogVelocityMmS = baseVelocitySpin->value();
        newConfig.fastMultiplier = fastMultiplierSpin->value();
        newConfig.slowMultiplier = slowMultiplierSpin->value();
        newConfig.triggerButton = triggerButtonCombo->currentData().toInt();
        newConfig.slowButton = slowButtonCombo->currentData().toInt();
        newConfig.fastButton = fastButtonCombo->currentData().toInt();
        newConfig.zDownButton = zDownButtonCombo->currentData().toInt();
        newConfig.zUpButton = zUpButtonCombo->currentData().toInt();
        for (int direction = 0; direction < kGamepadDirections; ++direction)
        {
            newConfig.directionBindings[direction].globalAxisId =
                directionControls[direction].axis->currentData().toInt();
            newConfig.directionBindings[direction].sign =
                directionControls[direction].sign->currentData().toInt() < 0 ? -1 : +1;
        }
        newConfig.zAxisGlobalId = zAxisCombo->currentData().toInt();
        newConfig.zSoftLimitsEnabled = zSoftLimitCheck->isChecked();
        newConfig.zMinUm = zMinSpin->value() * 1000.0;
        newConfig.zMaxUm = zMaxSpin->value() * 1000.0;
        newConfig.triggerAxisGlobalId = triggerAxisCombo->currentData().toInt();
        newConfig.triggerOutputPort = triggerPortCombo->currentData().toInt();
        newConfig.triggerPulseMs = triggerPulseSpin->value();

        if (newConfig.zSoftLimitsEnabled && newConfig.zMinUm >= newConfig.zMaxUm)
        {
            QMessageBox::warning(&dialog, "Thorlabs", "z_min must be smaller than z_max.");
            return;
        }
        const QSet<int> mappedButtons = {
            newConfig.triggerButton,
            newConfig.slowButton,
            newConfig.fastButton,
            newConfig.zDownButton,
            newConfig.zUpButton
        };
        if (mappedButtons.size() != 5)
        {
            QMessageBox::warning(&dialog, "Thorlabs", "Each gamepad function must use a different button.");
            return;
        }

        stopGamepadJogs(true);
        m_gamepadConfig = newConfig;
        ensureGamepadDefaults();
        updateGamepadDevice();
        refreshGamepadControlBar();
        dialog.accept();
    });

    dialog.exec();
    m_gamepadSuppressUntilNeutral = true;
    if (restartPollTimer && m_gamepadPollTimer)
        m_gamepadPollTimer->start();
}

void thorlabsKinesisPlugin::refreshAllAxisPositionsUi()
{
    if (!dock || !isInitialized())
        return;

    for (int id = 0; id < m_axisUi.size(); ++id)
        refreshAxisPositionUi(id);
}

void thorlabsKinesisPlugin::rebuildAxisFrames()
{
    clearAxisFrames();

    if (m_axes.isEmpty())
    {
        auto* emptyLabel = new QLabel("No axes detected. Please run Refresh / Detect.",
            ui.stageFrameContainer);
        emptyLabel->setAlignment(Qt::AlignCenter);
        emptyLabel->setWordWrap(true);
        emptyLabel->setMinimumHeight(80);
        ui.axisFramesLayout->addWidget(emptyLabel);
        updateAxisFrameAreaHeight();
        return;
    }

    m_axisUi.resize(m_axes.size());
    QMap<QString, QVBoxLayout*> serialLayouts;

    for (int id = 0; id < m_axes.size(); ++id)
    {
        const AxisEntry& ax = m_axes[id];
        AxisUi axisUi;

        auto* frame = new QFrame(ui.stageFrameContainer);
        Ui::stageFrame stageFrameUi;
        stageFrameUi.setupUi(frame);

        axisUi.frame = frame;
        axisUi.title = stageFrameUi.label_title;
        axisUi.statusLabel = stageFrameUi.label_status;
        axisUi.positionEdit = stageFrameUi.lineEdit_position;
        axisUi.stepEdit = stageFrameUi.lineEdit_step;
        axisUi.positionLcd = stageFrameUi.lcdNumber_position;
        axisUi.homeButton = stageFrameUi.pushButton_home;
        axisUi.moveButton = stageFrameUi.pushButton_go;
        axisUi.stopButton = stageFrameUi.pushButton_stop;
        axisUi.stepDownButton = stageFrameUi.pushButton_stepDown;
        axisUi.stepUpButton = stageFrameUi.pushButton_stepUp;
        axisUi.triggerMenuButton = stageFrameUi.toolButton_preferences;
        setupAxisTriggerMenu(axisUi, frame);

        stageFrameUi.label_globalStageID->setText(QString("%1.").arg(ax.globalAxisId));
        axisUi.title->setText(ax.axisName.isEmpty()
            ? ax.display
            : QString("%1 %2").arg(ax.isM30xy ? QStringLiteral("M30XY") : QStringLiteral("KVS30"), ax.axisName));
        frame->setToolTip(QStringLiteral("Serial: %1").arg(ax.baseSerial));

        const QLocale guiLocale(QLocale::German, QLocale::Germany);
        axisUi.positionEdit->setLocale(guiLocale);
        axisUi.stepEdit->setLocale(guiLocale);
        axisUi.positionEdit->setDecimals(3);
        axisUi.stepEdit->setDecimals(3);
        axisUi.positionEdit->setButtonSymbols(QAbstractSpinBox::NoButtons);
        axisUi.stepEdit->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
        axisUi.positionEdit->setKeyboardTracking(false);
        axisUi.stepEdit->setKeyboardTracking(false);

        double minUm = 0.0;
        double maxUm = 0.0;
        if (axisTravelLimitsUm(ax, minUm, maxUm) && maxUm > minUm)
        {
            axisUi.positionEdit->setRange(minUm / 1000.0, maxUm / 1000.0);
            axisUi.stepEdit->setRange(0.001, (maxUm - minUm) / 1000.0);
        }
        else
        {
            axisUi.positionEdit->setRange(-1000000.0, 1000000.0);
            axisUi.stepEdit->setRange(0.001, 1000000.0);
        }

        axisUi.positionEdit->setSingleStep(0.100);
        axisUi.stepEdit->setSingleStep(ax.isM30xy ? 0.100 : 0.010);
        axisUi.positionEdit->setValue(0.0);
        axisUi.stepEdit->setValue(ax.isM30xy ? 0.100 : 0.010);

        axisUi.triggerStartEdit->setText(QStringLiteral("0,000"));
        axisUi.triggerIntervalEdit->setText(QStringLiteral("0,100"));
        axisUi.triggerCountEdit->setText(QStringLiteral("1"));
        axisUi.triggerWidthEdit->setText(ax.isM30xy ? QStringLiteral("500") : QStringLiteral("1000"));
        axisUi.positionLcd->display(0.0);

        {
            QString velocityText = QStringLiteral("2,000");
            short err = 0;
            if (ax.isM30xy)
            {
                if (BDCStage* stage = m30xyForBase(ax.baseSerial))
                {
                    if (stage->isOpen())
                    {
                        const double velocityMmS = stage->getMaxVelocityMmS(
                            static_cast<unsigned>(ax.channel), &err);
                        if (err == 0 && std::isfinite(velocityMmS) && velocityMmS > 0.0)
                            velocityText = guiDecimalText(velocityMmS);
                    }
                }
            }
            else
            {
                if (KVSStage* stage = kvsForSerial(ax.baseSerial))
                {
                    if (stage->isOpen())
                    {
                        const double velocityMmS = stage->getMaxVelocityMmS(&err);
                        if (err == 0 && std::isfinite(velocityMmS) && velocityMmS > 0.0)
                            velocityText = guiDecimalText(velocityMmS);
                    }
                }
            }

            axisUi.triggerVelocityEdit->setText(velocityText);
        }
        const QString serialKey = ax.baseSerial.isEmpty() ? axisKey(ax) : ax.baseSerial;
        if (!serialLayouts.contains(serialKey))
        {
            const QString serialTitle = QString("%1 %2")
                .arg(ax.isM30xy ? QStringLiteral("M30XY") : QStringLiteral("KVS30"), serialKey);
            auto* headerWidget = new QWidget(ui.stageFrameContainer);
            headerWidget->setFixedSize(kStageFrameWidth, kSerialButtonHeight);
            auto* headerLayout = new QGridLayout(headerWidget);
            headerLayout->setContentsMargins(0, 0, 0, 0);
            headerLayout->setHorizontalSpacing(kGroupHeaderSpacing);
            headerLayout->setVerticalSpacing(0);

            auto* serialButton = new QPushButton(serialTitle + QStringLiteral(" >>>"),
                headerWidget);
            serialButton->setCheckable(true);
            serialButton->setFixedSize(
                kStageFrameWidth - kPanicButtonWidth - kGroupHeaderSpacing,
                kSerialButtonHeight);
            headerLayout->addWidget(serialButton, 0, 0);

            auto* serialPanicButton = new QPushButton(QStringLiteral("Panic Stop"),
                headerWidget);
            serialPanicButton->setFixedSize(kPanicButtonWidth, kSerialButtonHeight);
            serialPanicButton->setStyleSheet("background-color: red; color: black;");
            headerLayout->addWidget(serialPanicButton, 0, 1);

            ui.axisFramesLayout->addWidget(headerWidget, 0, Qt::AlignLeft);

            auto* serialWidget = new QWidget(ui.stageFrameContainer);
            serialWidget->setFixedWidth(kStageFrameWidth);
            auto* serialLayout = new QVBoxLayout(serialWidget);
            serialLayout->setContentsMargins(0, 0, 0, 0);
            serialLayout->setSpacing(2);
            ui.axisFramesLayout->addWidget(serialWidget, 0, Qt::AlignLeft);

            connect(serialButton, &QPushButton::toggled, this,
                [this, serialButton, serialWidget, serialTitle](bool collapsed) {
                    serialWidget->setVisible(!collapsed);
                    serialButton->setText(serialTitle + (collapsed
                        ? QStringLiteral(" <<<")
                        : QStringLiteral(" >>>")));
                    updateAxisFrameAreaHeight();
                });
            connect(serialPanicButton, &QPushButton::clicked, this,
                [this, serialKey]() {
                    for (int axisIndex = 0; axisIndex < m_axes.size(); ++axisIndex)
                    {
                        const AxisEntry& groupAxis = m_axes[axisIndex];
                        const QString groupKey = groupAxis.baseSerial.isEmpty()
                            ? axisKey(groupAxis)
                            : groupAxis.baseSerial;
                        if (groupKey == serialKey)
                        {
                            stopAxisIndex(axisIndex);
                            stopGamepadJogAxis(axisIndex, false);
                            refreshAxisStatusUi(axisIndex);
                        }
                    }
                    m_gamepadSuppressUntilNeutral = true;
                });
            serialLayouts.insert(serialKey, serialLayout);
        }

        serialLayouts.value(serialKey)->addWidget(frame, 0, Qt::AlignLeft | Qt::AlignTop);

        m_axisUi[id] = axisUi;

        connect(axisUi.homeButton, &QPushButton::clicked, this, [this, id]() {
            selectAxis(id);
            slot_goHome();
        });
        connect(axisUi.moveButton, &QPushButton::clicked, this, [this, id]() {
            selectAxis(id);
            syncLegacyMotionInputsFromAxisUi(id);
            slot_moveTo();
        });
        connect(axisUi.stopButton, &QPushButton::clicked, this, [this, id]() {
            stopAxisIndex(id);
            stopGamepadJogAxis(id, false);
            m_gamepadSuppressUntilNeutral = true;
            refreshAxisStatusUi(id);
        });
        connect(axisUi.stepDownButton, &QPushButton::clicked, this, [this, id]() {
            selectAxis(id);
            syncLegacyMotionInputsFromAxisUi(id);
            slot_moveStepBackward();
        });
        connect(axisUi.stepUpButton, &QPushButton::clicked, this, [this, id]() {
            selectAxis(id);
            syncLegacyMotionInputsFromAxisUi(id);
            slot_moveStepForward();
        });
        if (axisUi.applyTriggerButton)
        {
            connect(axisUi.applyTriggerButton, &QPushButton::clicked, this, [this, id]() {
                selectAxis(id);
                syncLegacyTriggerInputsFromAxisUi(id);
                slot_applyTrigger();
            });
        }

        if (axisUi.disableTriggerButton)
        {
            connect(axisUi.disableTriggerButton, &QPushButton::clicked, this, [this, id]() {
                selectAxis(id);
                slot_disableTrigger();
            });
        }

        if (axisUi.triggerIntervalEdit)
        {
            connect(axisUi.triggerIntervalEdit, &QLineEdit::textChanged,
                this, [this, id]() { updateTriggerFrequencyUi(id); });
        }
        if (axisUi.triggerVelocityEdit)
        {
            connect(axisUi.triggerVelocityEdit, &QLineEdit::textChanged,
                this, [this, id]() { updateTriggerFrequencyUi(id); });
        }

        updateTriggerFrequencyUi(id);
        refreshAxisStatusUi(id);
    }

    setMotionUiBusy(m_motionTaskActive.load());
    updateAxisFrameAreaHeight();
}

void thorlabsKinesisPlugin::refreshAxisUi()
{
    const int id = ui.comboBox_devices->currentIndex();
    if (id < 0 || id >= m_axes.size()) return;

    ui.lineEdit_serial->setText(m_axes[id].baseSerial);
}

bool thorlabsKinesisPlugin::axisMatches(const AxisEntry& ax, QChar axisName) const
{
    const QChar a = axisName.toLower();

    if (a == 'x' && ax.isM30xy && ax.channel == 1) return true;
    if (a == 'y' && ax.isM30xy && ax.channel == 2) return true;
    if (a == 'z' && !ax.isM30xy) return true;

    return false;
}

int thorlabsKinesisPlugin::findAxisIndexByName(QChar axisName) const
{
    for (int i = 0; i < m_axes.size(); ++i)
    {
        if (axisMatches(m_axes[i], axisName))
            return i;
    }
    return -1;
}

bool thorlabsKinesisPlugin::readAxisPositionUm(int id, double& valueUm) const
{
    if (id < 0 || id >= m_axes.size())
        return false;

    const AxisEntry& ax = m_axes[id];

    if (ax.isM30xy)
    {
        auto it = m_m30xy.find(ax.baseSerial.toStdString());
        if (it == m_m30xy.end() || !it->second || !it->second->isOpen())
            return false;

        valueUm = it->second->getPositionUm(ax.channel);
        return true;
    }
    else
    {
        auto it = m_kvs.find(ax.baseSerial.toStdString());
        if (it == m_kvs.end() || !it->second || !it->second->isOpen())
            return false;

        valueUm = it->second->getPositionUm();
        return true;
    }
}

bool thorlabsKinesisPlugin::detect()
{
    const QString methodName = "detect()";
    qDebug() << qPrintable(className + "::" + methodName) << "- start";

    // A refresh starts from a clean hardware state. Otherwise unplugged or
    // no-longer-detected controllers would remain open in the maps.
    waitForMotionToFinish();
    stopGamepadJogs(true, true);
    disableAllTriggers();
    closeDevices();
    setInitialized(false);

    m_detectedAxes.clear();
    m_axes.clear();
    clearGamepadJogState();
    refreshAxisCombo();

    const auto devices = detectKinesisDevices();

    QSet<QString> seenM30Base;
    QSet<QString> seenKvs;

    for (const auto& dev : devices)
    {
        const QString serial = QString::fromStdString(dev.serial);

        if (dev.deviceType == 24)
        {
            if (seenKvs.contains(serial)) continue;
            seenKvs.insert(serial);

            AxisEntry z;
            z.baseSerial = serial;
            z.channel = 1;
            z.isM30xy = false;
            z.axisName = QStringLiteral("Z");
            z.display = QString("KVS30 Z (S:%1)").arg(serial);
            m_detectedAxes.push_back(z);

            qDebug() << qPrintable(className + "::" + methodName)
                << "- detected KVS30 serial" << serial;
        }
        else if (dev.deviceType == 101)
        {
            if (!looksLikeBaseM30XYSerial(serial))
            {
                qDebug() << qPrintable(className + "::" + methodName)
                    << "- ignoring non-base BDC entry" << serial;
                continue;
            }

            if (seenM30Base.contains(serial)) continue;
            seenM30Base.insert(serial);

            AxisEntry x;
            x.baseSerial = serial;
            x.channel = 1;
            x.isM30xy = true;
            x.axisName = QStringLiteral("X");
            x.display = QString("M30XY X (Base:%1 ch1)").arg(serial);
            m_detectedAxes.push_back(x);

            AxisEntry y;
            y.baseSerial = serial;
            y.channel = 2;
            y.isM30xy = true;
            y.axisName = QStringLiteral("Y");
            y.display = QString("M30XY Y (Base:%1 ch2)").arg(serial);
            m_detectedAxes.push_back(y);

            qDebug() << qPrintable(className + "::" + methodName)
                << "- detected M30XY base serial" << serial;
        }
    }

    m_axes = m_detectedAxes;
    assignGlobalAxisIds(m_axes);
    refreshAxisCombo();

    setDetected(!m_detectedAxes.isEmpty());
    rebuildAxisFrames();

    if (isDetected() && !m_axes.isEmpty())
    {
        ui.comboBox_devices->setCurrentIndex(0);
        refreshAxisUi();
    }

    qDebug() << qPrintable(className + "::" + methodName)
        << "- done detected=" << isDetected()
        << "detectedAxes=" << m_detectedAxes.size();

    return isDetected();
}

bool thorlabsKinesisPlugin::initialize()
{
    const QString methodName = "initialize()";
    qDebug() << qPrintable(className + "::" + methodName) << "- start";

    if (isInitialized() && !m_axes.isEmpty())
        return true;

    if (!isDetected() || m_detectedAxes.isEmpty())
    {
        qDebug() << qPrintable(className + "::" + methodName) << "- not detected / no axes";
        return false;
    }

    if (!chooseAxesForInitialization())
    {
        qDebug() << qPrintable(className + "::" + methodName) << "- axis selection cancelled or empty";
        setInitialized(false);
        return false;
    }

    bool okAll = true;

    QSet<QString> m30Bases;
    QSet<QString> kvsSerials;

    for (const auto& ax : m_axes)
    {
        if (ax.isM30xy) m30Bases.insert(ax.baseSerial);
        else kvsSerials.insert(ax.baseSerial);
    }

    for (const QString& b : m30Bases)
    {
        short err = 0;
        qDebug() << qPrintable(className + "::" + methodName) << "- opening M30XY base" << b;

        const bool openedNow = m30xyForBase(b)->open(b.toStdString(), true, &err);
        qDebug() << qPrintable(className + "::" + methodName)
            << "- M30XY base" << b << "openResult(openedNow)=" << openedNow << "err=" << err;

        if (!openedNow || err != 0) okAll = false;
    }

    for (const QString& s : kvsSerials)
    {
        short err = 0;
        qDebug() << qPrintable(className + "::" + methodName) << "- opening KVS serial" << s;

        const bool openedNow = kvsForSerial(s)->open(s.toStdString(), false, &err);
        qDebug() << qPrintable(className + "::" + methodName)
            << "- KVS serial" << s << "openResult(openedNow)=" << openedNow << "err=" << err;

        if (!openedNow || err != 0) okAll = false;
    }

    if (!okAll)
        closeDevices();

    setInitialized(okAll);
    if (okAll)
    {
        ensureGamepadDefaults();
        updateGamepadDevice();
    }
    setMotionUiBusy(false);
    refreshAllAxisPositionsUi();

    qDebug() << qPrintable(className + "::" + methodName) << "- done initialized=" << isInitialized();
    return isInitialized();
}

bool thorlabsKinesisPlugin::release()
{
    const QString methodName = "release()";
    qDebug() << qPrintable(className + "::" + methodName) << "- start";

    waitForMotionToFinish();
    stopGamepadJogs(true, true);
    disableAllTriggers();
    closeDevices();
    m_detectedAxes.clear();
    m_axes.clear();
    clearGamepadJogState();
    m_activePositionConfigIndex = -1;
    if (dock)
    {
        refreshAxisCombo();
        rebuildAxisFrames();
    }

    if (m_positionManagerWindow)
        m_positionManagerWindow->refresh();

    setInitialized(false);
    setDetected(false);
    if (dock)
        setMotionUiBusy(false);

    qDebug() << qPrintable(className + "::" + methodName) << "- done";
    return true;
}

bool thorlabsKinesisPlugin::moveTo(double pos, int id)
{
    return moveToAxisIndex(pos, axisIndexFromPublicId(id));
}

bool thorlabsKinesisPlugin::moveToAxisIndex(double pos, int axisIndex, bool waitForOtherAxes)
{
    const QString methodName = "moveTo(pos_um,id)";

    if (!isInitialized() || !std::isfinite(pos) || axisIndex < 0 || axisIndex >= m_axes.size())
        return false;
    if (waitForOtherAxes && !waitUntilAllAxesStopped())
        return false;
    if (m_motionTaskActive.load() && m_stopRequested.load())
        return false;

    AxisEntry& ax = m_axes[axisIndex];

    qDebug() << qPrintable(className + "::" + methodName)
        << "- axisIndex" << axisIndex
        << "globalAxisId" << ax.globalAxisId
        << "baseSerial" << ax.baseSerial
        << "ch" << ax.channel
        << "pos_um" << pos;

    short err = 0;

    if (!disableAllTriggers())
        return false;
    if (m_motionTaskActive.load() && m_stopRequested.load())
        return false;

    if (ax.isM30xy)
    {
        BDCStage* stage = m30xyForBase(ax.baseSerial);
        return stage->isOpen() && stage->moveToUm(pos, ax.channel, &err);
    }

    KVSStage* stage = kvsForSerial(ax.baseSerial);
    return stage->isOpen() && stage->moveToUm(pos, &err);
}

bool thorlabsKinesisPlugin::moveTo(double* pos, const char* axes)
{
    return moveAxesCoordinated(pos, axes, false);
}

bool thorlabsKinesisPlugin::moveTo(const double* positions, const int* axes, int axisCount)
{
    if (!positions || !axes || axisCount < 1)
        return false;

    if (moveAxesByGlobalIds(positions, axes, axisCount, false))
        return true;

    bool allLegacyAxisCodes = true;
    for (int i = 0; i < axisCount; ++i)
    {
        if (axes[i] != 1 && axes[i] != 2 && axes[i] != 4)
        {
            allLegacyAxisCodes = false;
            break;
        }
    }

    return allLegacyAxisCodes
        ? moveToAxisCodes(positions, axes, static_cast<std::size_t>(axisCount))
        : false;
}

bool thorlabsKinesisPlugin::moveToAxisCodes(const double* positions, const int* axes, std::size_t count)
{
    if (!positions || !axes || count == 0)
        return false;

    std::string axisNames;
    axisNames.reserve(count);

    for (std::size_t i = 0; i < count; ++i) {
        switch (axes[i]) {
        case 1:
            axisNames.push_back('x');
            break;
        case 2:
            axisNames.push_back('y');
            break;
        case 4:
            axisNames.push_back('z');
            break;
        default:
            return false;
        }
    }

    return moveAxesCoordinated(positions, axisNames.c_str(), false);
}

ScanCapabilities thorlabsKinesisPlugin::getScanCapabilities() const
{
    ScanCapabilities capabilities;
    capabilities.supportsScanJobs = true;
    capabilities.supportsHardwarePositionTrigger = true;
    capabilities.supportsHorizontalLines = true;
    capabilities.supportsVerticalLines = true;
    capabilities.supportsDiagonalLines = false;
    capabilities.supportsLayeredZ = findAxisIndexByName(QChar('z')) >= 0;
    capabilities.controlsScanVelocity = true;
    capabilities.maximumTriggerFrequencyHz = kMaximumLaserTriggerFrequencyHz;
    return capabilities;
}

ScanValidationResult thorlabsKinesisPlugin::validateScanJob(const ScanJob& job) const
{
    const ScanValidationResult generic = ::validateScanJob(job, kMaximumLaserTriggerFrequencyHz);
    if (!generic.isValid())
        return generic;

    const int xAxisIndex = findAxisIndexByName(QChar('x'));
    const int yAxisIndex = findAxisIndexByName(QChar('y'));
    if (xAxisIndex < 0 || yAxisIndex < 0)
        return scanValidationError(ScanValidationError::MissingScanAxis);

    const AxisEntry& xAxis = m_axes[xAxisIndex];
    const AxisEntry& yAxis = m_axes[yAxisIndex];
    if (!xAxis.isM30xy || !yAxis.isM30xy || xAxis.baseSerial != yAxis.baseSerial)
        return scanValidationError(ScanValidationError::MissingScanAxis);

    if (scanJobNeedsZAxis(job) && findAxisIndexByName(QChar('z')) < 0)
        return scanValidationError(ScanValidationError::MissingScanAxis);

    for (std::size_t layerIndex = 0; layerIndex < job.layers.size(); ++layerIndex)
    {
        const ScanLayer& layer = job.layers[layerIndex];
        for (std::size_t lineIndex = 0; lineIndex < layer.lines.size(); ++lineIndex)
        {
            QChar scanAxis;
            if (!scanAxisForLine(layer.lines[lineIndex], scanAxis))
                return scanValidationError(ScanValidationError::UnsupportedScanGeometry, layerIndex, lineIndex);
        }
    }

    return ScanValidationResult{};
}

bool thorlabsKinesisPlugin::configureLineTrigger(BDCStage* stage, unsigned channel,
    double startUm, double endUm, const ScanJob& job)
{
    if (!stage || !stage->isOpen() || channel < 1 || channel > 2)
        return false;

    short err = 0;
    const int32_t startDev = stage->umToDevice(startUm, channel, &err);
    if (err != 0)
        return false;

    const int32_t intervalDev = stage->umToDevice(job.triggerSpacingUm, channel, &err);
    if (err != 0 || intervalDev <= 0)
        return false;

    const int32_t pulseCount = pulseCountForLine(startUm, endUm, job.triggerSpacingUm);
    if (pulseCount <= 0)
        return false;

    if (!stage->setMaxVelocityMmS(channel, job.velocityMmS, &err))
        return false;

    const double triggerFrequencyHz = job.velocityMmS / (job.triggerSpacingUm / 1000.0);
    if (!std::isfinite(triggerFrequencyHz)
        || triggerFrequencyHz > kMaximumLaserTriggerFrequencyHz + 1e-9)
        return false;

    const double triggerPeriodUs = 1000000.0 / triggerFrequencyHz;
    if (static_cast<double>(job.pulseWidthUs) > triggerPeriodUs + 1e-9)
        return false;

    const bool forward = endUm >= startUm;

    BDCTriggerConfig cfg;
    cfg.enabled = true;
    cfg.trigger1Mode = static_cast<int>(forward
        ? BDCTriggerMode::PositionStepForward
        : BDCTriggerMode::PositionStepReverse);
    cfg.trigger1Polarity = static_cast<int>(BDCTriggerPolarity::High);
    cfg.trigger2Mode = static_cast<int>(BDCTriggerMode::Disabled);
    cfg.trigger2Polarity = static_cast<int>(BDCTriggerPolarity::High);
    cfg.startPosFwd = startDev;
    cfg.intervalFwd = intervalDev;
    cfg.pulseCountFwd = pulseCount;
    cfg.startPosRev = startDev;
    cfg.intervalRev = intervalDev;
    cfg.pulseCountRev = pulseCount;
    cfg.pulseWidthUs = job.pulseWidthUs;
    cfg.cycleCount = 1;

    stage->setTriggerConfig(cfg);
    return stage->applyTriggerConfig(channel, &err);
}

bool thorlabsKinesisPlugin::executeScanLine(const LaserLine& line, const ScanJob& job)
{
    QChar scanAxisName;
    if (!scanAxisForLine(line, scanAxisName))
        return false;

    const int scanAxisIndex = findAxisIndexByName(scanAxisName);
    if (scanAxisIndex < 0)
        return false;

    const AxisEntry& scanAxis = m_axes[scanAxisIndex];
    if (!scanAxis.isM30xy)
        return false;

    BDCStage* stage = m30xyForBase(scanAxis.baseSerial);
    if (!stage || !stage->isOpen())
        return false;

    const unsigned channel = static_cast<unsigned>(scanAxis.channel);
    const double startUm = scanAxisName == QChar('x') ? line.start.xUm : line.start.yUm;
    const double endUm = scanAxisName == QChar('x') ? line.end.xUm : line.end.yUm;

    short err = 0;
    const int32_t targetDev = stage->umToDevice(endUm, channel, &err);
    if (err != 0)
        return false;

    if (!configureLineTrigger(stage, channel, startUm, endUm, job))
        return false;

    if (m_stopRequested.load())
    {
        stage->disableTrigger(channel, &err);
        return false;
    }

    if (!stage->beginMoveTo(targetDev, channel, &err))
    {
        stage->disableTrigger(channel, &err);
        return false;
    }

    const bool reached = stage->waitForPosition(targetDev, channel, 120000, &err);
    const bool disabled = stage->disableTrigger(channel, &err);
    return reached && disabled && !m_stopRequested.load();
}

bool thorlabsKinesisPlugin::executeScanJob(const ScanJob& job)
{
    if (!isInitialized())
        return false;

    bool expectedInactive = false;
    if (!m_motionTaskActive.compare_exchange_strong(expectedInactive, true))
        return false;

    struct ScanMotionGuard
    {
        thorlabsKinesisPlugin* plugin = nullptr;
        ~ScanMotionGuard()
        {
            if (!plugin)
                return;
            plugin->disableAllTriggers();
            plugin->m_motionTaskActive.store(false);
            plugin->m_stopRequested.store(false);
        }
    } guard{ this };

    m_stopRequested.store(false);

    const ScanValidationResult validation = validateScanJob(job);
    if (!validation.isValid())
    {
        qDebug() << className << "::executeScanJob validation failed"
            << static_cast<int>(validation.error)
            << "layer" << static_cast<int>(validation.layerIndex)
            << "line" << static_cast<int>(validation.lineIndex);
        return false;
    }

    if (!isStopped())
        return false;

    const bool useZ = scanJobNeedsZAxis(job);

    for (const ScanLayer& layer : job.layers)
    {
        for (const LaserLine& line : layer.lines)
        {
            if (m_stopRequested.load())
                return false;

            std::array<double, 3> startPositions = {
                line.start.xUm,
                line.start.yUm,
                layer.zUm
            };
            char axes[4] = { 'x', 'y', '\0', '\0' };
            if (useZ)
            {
                axes[2] = 'z';
                axes[3] = '\0';
            }

            if (!moveAxesCoordinated(startPositions.data(), axes, false))
                return false;

            if (m_stopRequested.load())
                return false;

            if (!executeScanLine(line, job))
                return false;
        }
    }

    return true;
}

bool thorlabsKinesisPlugin::abortScanJob()
{
    return stop();
}

bool thorlabsKinesisPlugin::moveAxesCoordinated(const double* values, const char* axes, bool relative)
{
    if (!isInitialized() || !values || !axes)
        return false;

    QVector<int> axisIndices;
    if (!resolveAxisRequest(axes, axisIndices))
        return false;

    if (!m_motionTaskActive.load())
        m_stopRequested.store(false);
    if (!waitUntilAllAxesStopped())
        return false;

    struct PendingMove
    {
        BDCStage* bdc = nullptr;
        KVSStage* kvs = nullptr;
        unsigned channel = 1;
        int32_t targetDeviceUnits = 0;
    };

    std::vector<PendingMove> pending;
    pending.reserve(static_cast<std::size_t>(axisIndices.size()));

    // Resolve and convert every target before the first controller receives a
    // movement command. Relative targets are based on one common snapshot.
    for (int i = 0; i < axisIndices.size(); ++i)
    {
        if (!std::isfinite(values[i]))
            return false;

        const int id = axisIndices[i];
        if (id < 0)
            return false;

        const AxisEntry& axis = m_axes[id];
        PendingMove move;
        move.channel = static_cast<unsigned>(axis.channel);
        short err = 0;

        if (axis.isM30xy)
        {
            move.bdc = m30xyForBase(axis.baseSerial);
            if (!move.bdc || !move.bdc->isOpen())
                return false;

            const int32_t converted = move.bdc->umToDevice(values[i], move.channel, &err);
            if (err != 0)
                return false;

            if (relative)
            {
                const int32_t current = move.bdc->getPosition(move.channel, &err);
                const int64_t target = static_cast<int64_t>(current) + static_cast<int64_t>(converted);
                if (err != 0
                    || target < (std::numeric_limits<int32_t>::min)()
                    || target > (std::numeric_limits<int32_t>::max)())
                    return false;
                move.targetDeviceUnits = static_cast<int32_t>(target);
            }
            else
            {
                move.targetDeviceUnits = converted;
            }
        }
        else
        {
            move.kvs = kvsForSerial(axis.baseSerial);
            if (!move.kvs || !move.kvs->isOpen())
                return false;

            const int32_t converted = move.kvs->umToDevice(values[i], &err);
            if (err != 0)
                return false;

            if (relative)
            {
                const int32_t current = move.kvs->getPosition(&err);
                const int64_t target = static_cast<int64_t>(current) + static_cast<int64_t>(converted);
                if (err != 0
                    || target < (std::numeric_limits<int32_t>::min)()
                    || target > (std::numeric_limits<int32_t>::max)())
                    return false;
                move.targetDeviceUnits = static_cast<int32_t>(target);
            }
            else
            {
                move.targetDeviceUnits = converted;
            }
        }

        pending.push_back(move);
    }

    if (!disableAllTriggers() || m_stopRequested.load())
        return false;

    // Phase 1: enqueue every axis movement without waiting. The controller
    // commands are issued back-to-back with only the API-call latency between
    // them; this is a coordinated software start, not a hardware sync pulse.
    for (PendingMove& move : pending)
    {
        if (m_stopRequested.load())
        {
            stop();
            return false;
        }

        short err = 0;
        const bool started = move.bdc
            ? move.bdc->beginMoveTo(move.targetDeviceUnits, move.channel, &err)
            : move.kvs->beginMoveTo(move.targetDeviceUnits, &err);
        if (!started)
        {
            stop();
            return false;
        }
    }

    // Phase 2: all axes are already moving. Waiting sequentially here does not
    // serialize their motion; it only collects completion and error results.
    for (PendingMove& move : pending)
    {
        if (m_stopRequested.load())
        {
            stop();
            return false;
        }

        short err = 0;
        const bool reached = move.bdc
            ? move.bdc->waitForPosition(move.targetDeviceUnits, move.channel, 120000, &err)
            : move.kvs->waitForPosition(move.targetDeviceUnits, 120000, &err);
        if (!reached)
        {
            stop();
            return false;
        }
    }

    return true;
}

bool thorlabsKinesisPlugin::moveAxesByGlobalIds(const double* values,
    const int* globalAxisIds, int axisCount, bool relative)
{
    if (!values || !globalAxisIds || axisCount < 1 || axisCount > 9)
        return false;

    std::string encodedAxisIds;
    encodedAxisIds.reserve(static_cast<std::size_t>(axisCount) * 2);
    QSet<int> seen;
    for (int i = 0; i < axisCount; ++i)
    {
        const int globalAxisId = globalAxisIds[i];
        if (!std::isfinite(values[i])
            || globalAxisId < 1
            || globalAxisId > 9
            || seen.contains(globalAxisId)
            || axisIndexFromGlobalId(globalAxisId) < 0)
        {
            return false;
        }

        seen.insert(globalAxisId);
        if (!encodedAxisIds.empty())
            encodedAxisIds += '\n';
        encodedAxisIds += static_cast<char>('0' + globalAxisId);
    }

    return moveAxesCoordinated(values, encodedAxisIds.c_str(), relative);
}

bool thorlabsKinesisPlugin::moveSteps(double steps, int id)
{
    return moveStepsAxisIndex(steps, axisIndexFromPublicId(id));
}

bool thorlabsKinesisPlugin::moveStepsAxisIndex(double steps, int axisIndex, bool waitForOtherAxes)
{
    const QString methodName = "moveSteps(delta_um,id)";

    if (!isInitialized() || !std::isfinite(steps) || axisIndex < 0 || axisIndex >= m_axes.size())
        return false;
    if (waitForOtherAxes && !waitUntilAllAxesStopped())
        return false;
    if (m_motionTaskActive.load() && m_stopRequested.load())
        return false;

    AxisEntry& ax = m_axes[axisIndex];

    qDebug() << qPrintable(className + "::" + methodName)
        << "- axisIndex" << axisIndex
        << "globalAxisId" << ax.globalAxisId
        << "baseSerial" << ax.baseSerial
        << "ch" << ax.channel
        << "delta_um" << steps;

    short err = 0;

    if (!disableAllTriggers())
        return false;
    if (m_motionTaskActive.load() && m_stopRequested.load())
        return false;

    if (ax.isM30xy)
    {
        BDCStage* stage = m30xyForBase(ax.baseSerial);
        return stage->isOpen() && stage->moveRelUm(steps, ax.channel, &err);
    }

    KVSStage* stage = kvsForSerial(ax.baseSerial);
    return stage->isOpen() && stage->moveRelUm(steps, &err);
}

// Helper for moving several relative axes at once, e.g. "xyz"
bool thorlabsKinesisPlugin::moveSteps(double* steps, const char* axes)
{
    return moveAxesCoordinated(steps, axes, true);
}

bool thorlabsKinesisPlugin::moveSteps(const double* steps, const int* axes, int axisCount)
{
    if (!steps || !axes || axisCount < 1)
        return false;

    if (moveAxesByGlobalIds(steps, axes, axisCount, true))
        return true;

    bool allLegacyAxisCodes = true;
    for (int i = 0; i < axisCount; ++i)
    {
        if (axes[i] != 1 && axes[i] != 2 && axes[i] != 4)
        {
            allLegacyAxisCodes = false;
            break;
        }
    }
    if (!allLegacyAxisCodes)
        return false;

    std::string axisNames;
    axisNames.reserve(static_cast<std::size_t>(axisCount));

    for (int i = 0; i < axisCount; ++i) {
        switch (axes[i]) {
        case 1:
            axisNames.push_back('x');
            break;
        case 2:
            axisNames.push_back('y');
            break;
        case 4:
            axisNames.push_back('z');
            break;
        default:
            return false;
        }
    }

    return moveAxesCoordinated(steps, axisNames.c_str(), true);
}

double* thorlabsKinesisPlugin::getPosition(const char* axes)
{
    QVector<int> axisIndices;
    if (!resolveAxisRequest(axes, axisIndices))
    {
        m_lastPositionsUm.clear();
        return nullptr;
    }

    m_lastPositionsUm.assign(static_cast<std::size_t>(axisIndices.size()), 0.0);
    for (int i = 0; i < axisIndices.size(); ++i)
    {
        const int id = axisIndices[i];
        if (id < 0)
            continue;

        double valueUm = 0.0;
        if (readAxisPositionUm(id, valueUm))
            m_lastPositionsUm[static_cast<std::size_t>(i)] = valueUm;
    }

    return m_lastPositionsUm.empty() ? nullptr : m_lastPositionsUm.data();
}

double* thorlabsKinesisPlugin::getPosition()
{
    return getPosition(nullptr);
}

QVector<thorlabsKinesisPlugin::AxisInfo> thorlabsKinesisPlugin::getAxisInfo() const
{
    QVector<AxisInfo> axes;
    axes.reserve(m_axes.size());

    for (const AxisEntry& axis : m_axes)
    {
        double minUm = 0.0;
        double maxUm = 0.0;
        const bool limitsValid = axisTravelLimitsUm(axis, minUm, maxUm);
        axes.append({ axis.globalAxisId, axisDisplayText(axis), axis.baseSerial,
            minUm, maxUm, limitsValid });
    }

    return axes;
}

int thorlabsKinesisPlugin::positionConfigIndex(const QString& name) const
{
    const QString normalized = name.trimmed();
    for (int i = 0; i < m_positionConfigs.size(); ++i)
    {
        if (m_positionConfigs[i].name.compare(normalized, Qt::CaseInsensitive) == 0)
            return i;
    }
    return -1;
}

QStringList thorlabsKinesisPlugin::getPositionConfigNames() const
{
    QStringList names;
    for (const PositionConfig& config : m_positionConfigs)
        names.append(config.name);
    return names;
}

QString thorlabsKinesisPlugin::getActivePositionConfigName() const
{
    if (m_activePositionConfigIndex < 0
        || m_activePositionConfigIndex >= m_positionConfigs.size())
        return QString();
    return m_positionConfigs[m_activePositionConfigIndex].name;
}

bool thorlabsKinesisPlugin::getPositionConfig(const QString& name,
    QVector<int>& globalAxisIDs, QVector<double>& positionsUm) const
{
    globalAxisIDs.clear();
    positionsUm.clear();

    const int index = positionConfigIndex(name);
    if (index < 0)
        return false;

    globalAxisIDs = m_positionConfigs[index].globalAxisIDs;
    positionsUm = m_positionConfigs[index].positionsUm;
    return true;
}

bool thorlabsKinesisPlugin::savePositionConfig(const QString& name,
    const QVector<int>& globalAxisIDs, const QVector<double>& positionsUm)
{
    const QString normalized = name.trimmed();
    if (normalized.isEmpty()
        || globalAxisIDs.isEmpty()
        || globalAxisIDs.size() != positionsUm.size())
        return false;

    QSet<int> seen;
    for (int i = 0; i < globalAxisIDs.size(); ++i)
    {
        const int globalAxisId = globalAxisIDs[i];
        if (axisIndexFromGlobalId(globalAxisId) < 0
            || seen.contains(globalAxisId)
            || !std::isfinite(positionsUm[i]))
        {
            return false;
        }
        seen.insert(globalAxisId);
    }

    PositionConfig config{ normalized, globalAxisIDs, positionsUm };
    const int existingIndex = positionConfigIndex(normalized);
    if (existingIndex >= 0)
    {
        m_positionConfigs[existingIndex] = config;
        m_activePositionConfigIndex = existingIndex;
    }
    else
    {
        m_positionConfigs.append(config);
        m_activePositionConfigIndex = m_positionConfigs.size() - 1;
    }
    return true;
}

bool thorlabsKinesisPlugin::removePositionConfig(const QString& name)
{
    const int removedIndex = positionConfigIndex(name);
    if (removedIndex < 0)
        return false;

    m_positionConfigs.removeAt(removedIndex);
    if (m_positionConfigs.isEmpty())
        m_activePositionConfigIndex = -1;
    else if (m_activePositionConfigIndex == removedIndex)
        m_activePositionConfigIndex = qMin(removedIndex, m_positionConfigs.size() - 1);
    else if (m_activePositionConfigIndex > removedIndex)
        --m_activePositionConfigIndex;

    return true;
}

bool thorlabsKinesisPlugin::choosePositionConfig(const QString& name)
{
    const int index = positionConfigIndex(name);
    if (index < 0)
        return false;
    m_activePositionConfigIndex = index;
    return true;
}

bool thorlabsKinesisPlugin::goToPositionConfig()
{
    if (!isInitialized()
        || m_activePositionConfigIndex < 0
        || m_activePositionConfigIndex >= m_positionConfigs.size())
    {
        return false;
    }

    const PositionConfig config = m_positionConfigs[m_activePositionConfigIndex];
    if (config.globalAxisIDs.isEmpty()
        || config.globalAxisIDs.size() != config.positionsUm.size())
        return false;

    return moveAxesByGlobalIds(config.positionsUm.constData(),
        config.globalAxisIDs.constData(),
        config.globalAxisIDs.size(),
        false);
}

bool thorlabsKinesisPlugin::goToPositionConfig(const QString& name)
{
    return choosePositionConfig(name) && goToPositionConfig();
}

bool thorlabsKinesisPlugin::stop()
{
    const QString methodName = "stop()";
    m_stopRequested.store(true);
    qDebug() << qPrintable(className + "::" + methodName) << "- stopping all open axes";

    std::lock_guard<std::mutex> lock(m_deviceMapMutex);
    bool ok = true;
    bool foundOpenStage = false;

    for (auto& entry : m_m30xy)
    {
        BDCStage* stage = entry.second.get();
        if (!stage || !stage->isOpen()) continue;
        foundOpenStage = true;

        short err = 0;
        if (!stage->stopImmediate(1, &err)) ok = false;
        if (!stage->stopImmediate(2, &err)) ok = false;
    }

    for (auto& entry : m_kvs)
    {
        KVSStage* stage = entry.second.get();
        if (!stage || !stage->isOpen()) continue;
        foundOpenStage = true;

        short err = 0;
        if (!stage->stopImmediate(&err)) ok = false;
    }

    return foundOpenStage && ok;
}

bool thorlabsKinesisPlugin::stopAxisIndex(int id)
{
    if (id < 0 || id >= m_axes.size())
        return false;

    const AxisEntry& axis = m_axes[id];
    short err = 0;

    qDebug() << className << "::stopAxisIndex id" << id
        << "serial" << axis.baseSerial
        << "ch" << axis.channel;

    bool ok = false;
    if (axis.isM30xy)
    {
        BDCStage* stage = m30xyForBase(axis.baseSerial);
        ok = stage && stage->isOpen()
            && stage->stopImmediate(static_cast<unsigned>(axis.channel), &err);
    }
    else
    {
        KVSStage* stage = kvsForSerial(axis.baseSerial);
        ok = stage && stage->isOpen() && stage->stopImmediate(&err);
    }

    return ok;
}

bool thorlabsKinesisPlugin::isStopped()
{
    std::lock_guard<std::mutex> lock(m_deviceMapMutex);
    bool foundOpenStage = false;

    for (const auto& entry : m_m30xy)
    {
        const BDCStage* stage = entry.second.get();
        if (!stage || !stage->isOpen()) continue;
        foundOpenStage = true;

        short err = 0;
        if (stage->isMoving(1, &err) || err != 0) return false;
        if (stage->isMoving(2, &err) || err != 0) return false;
    }

    for (const auto& entry : m_kvs)
    {
        const KVSStage* stage = entry.second.get();
        if (!stage || !stage->isOpen()) continue;
        foundOpenStage = true;

        short err = 0;
        if (stage->isMoving(&err) || err != 0) return false;
    }

    return foundOpenStage;
}

// legacy stubs kept
void thorlabsKinesisPlugin::setHWSerialNr(QString* /*serialNr*/) {}
void thorlabsKinesisPlugin::setHWSerialNr(QString /*serialNr*/) {}
void thorlabsKinesisPlugin::setStepSize(QString /*stepSize*/) {}
void thorlabsKinesisPlugin::setVelocityParameters(QString, QString, QString) {}

void thorlabsKinesisPlugin::initGUI()
{
    if (m_guiInitialized)
        return;

    m_guiInitialized = true;
    applyQMotionLikeWidgetStyle(dock ? dock->widget() : nullptr);

    connect(ui.pushButton_detect, &QPushButton::clicked,
        this, &thorlabsKinesisPlugin::slot_detect, Qt::UniqueConnection);
    connect(ui.comboBox_devices, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, &thorlabsKinesisPlugin::slot_deviceChanged, Qt::UniqueConnection);

    connect(ui.pushButton_home, &QPushButton::clicked,
        this, &thorlabsKinesisPlugin::slot_goHome, Qt::UniqueConnection);
    connect(ui.pushButton_stop, &QPushButton::clicked,
        this, &thorlabsKinesisPlugin::slot_stopMotor, Qt::UniqueConnection);

    connect(ui.pushButton_position, &QPushButton::clicked,
        this, &thorlabsKinesisPlugin::slot_moveTo, Qt::UniqueConnection);
    connect(ui.lineEdit_position, &QLineEdit::returnPressed,
        ui.pushButton_position, &QPushButton::click, Qt::UniqueConnection);

    connect(ui.pushButton_stepUp, &QPushButton::clicked,
        this, &thorlabsKinesisPlugin::slot_moveStepForward, Qt::UniqueConnection);
    connect(ui.pushButton_stepDown, &QPushButton::clicked,
        this, &thorlabsKinesisPlugin::slot_moveStepBackward, Qt::UniqueConnection);

    connect(ui.pushButton_positionManager, &QPushButton::clicked,
        this, &thorlabsKinesisPlugin::slot_openPositionManager, Qt::UniqueConnection);
    setupGamepadControls();

    connect(ui.pushButton_applyTrigger, &QPushButton::clicked,
        this, &thorlabsKinesisPlugin::slot_applyTrigger, Qt::UniqueConnection);
    connect(ui.pushButton_disableTrigger, &QPushButton::clicked,
        this, &thorlabsKinesisPlugin::slot_disableTrigger, Qt::UniqueConnection);

    ui.lineEdit_position->setText("0");
    ui.lineEdit_step->setText("100");
    ui.lineEdit_trigStart->setText("0");
    ui.lineEdit_trigInterval->setText("0");
    ui.lineEdit_trigCount->setText("1");
    ui.lineEdit_trigWidth->setText("50000");
    ui.pushButton_stop->setStyleSheet("background-color: red; color: black;");
    ui.pushButton_position->setStyleSheet("background-color: green;");

    rebuildAxisFrames();
    setMotionUiBusy(false);

    m_positionRefreshTimer = new QTimer(this);
    m_positionRefreshTimer->setInterval(kPositionRefreshIntervalMs);
    m_positionRefreshTimer->setTimerType(Qt::CoarseTimer);
    connect(m_positionRefreshTimer, &QTimer::timeout,
        this, &thorlabsKinesisPlugin::refreshAllAxisPositionsUi, Qt::UniqueConnection);
    m_positionRefreshTimer->start();

    m_gamepadPollTimer = new QTimer(this);
    m_gamepadPollTimer->setInterval(kGamepadPollIntervalMs);
    m_gamepadPollTimer->setTimerType(Qt::PreciseTimer);
    connect(m_gamepadPollTimer, &QTimer::timeout,
        this, &thorlabsKinesisPlugin::pollGamepad, Qt::UniqueConnection);
    m_gamepadPollTimer->start();

    updateGamepadDevice();

    qDebug() << className << "::initGUI - signals connected";
}

void thorlabsKinesisPlugin::slot_detect()
{
    qDebug() << className << "::slot_detect";
    if (m_motionThread || !m_axisMotionThreads.isEmpty())
        return;
    detect();
    if (isDetected()) initialize();
}

void thorlabsKinesisPlugin::slot_deviceChanged(int)
{
    qDebug() << className << "::slot_deviceChanged";
    refreshAxisUi();
}

void thorlabsKinesisPlugin::slot_goHome()
{
    const QString methodName = "slot_goHome()";
    const int id = ui.comboBox_devices->currentIndex();
    if (id < 0 || id >= m_axes.size()) return;

    const AxisEntry ax = m_axes[id];
    qDebug() << qPrintable(className + "::" + methodName)
        << "- id" << id << "baseSerial" << ax.baseSerial;

    startAxisMotionTask(id, "Homing", [this, ax]() {
        if (m_stopRequested.load())
            return false;
        if (!disableAllTriggers())
            return false;
        if (m_stopRequested.load())
            return false;

        short err = 0;
        if (ax.isM30xy)
        {
            BDCStage* stage = m30xyForBase(ax.baseSerial);
            return stage->isOpen() && stage->home(static_cast<unsigned>(ax.channel), &err);
        }

        KVSStage* stage = kvsForSerial(ax.baseSerial);
        return stage->isOpen() && stage->home(&err);
    }, true);
}

void thorlabsKinesisPlugin::slot_moveStepForward()
{
    const int id = ui.comboBox_devices->currentIndex();
    double stepUm = 0.0;
    if (!parseFiniteDouble(ui.lineEdit_step->text(), stepUm) || stepUm <= 0.0)
    {
        QMessageBox::warning(dock, "Thorlabs", "Please enter a positive, valid step size.");
        return;
    }

    qDebug() << className << "::slot_moveStepForward id" << id << "step_um" << stepUm;
    startAxisMotionTask(id, "Relative Motion",
        [this, id, stepUm]() { return moveStepsAxisIndex(stepUm, id, false); });
}

void thorlabsKinesisPlugin::slot_moveStepBackward()
{
    const int id = ui.comboBox_devices->currentIndex();
    double stepUm = 0.0;
    if (!parseFiniteDouble(ui.lineEdit_step->text(), stepUm) || stepUm <= 0.0)
    {
        QMessageBox::warning(dock, "Thorlabs", "Please enter a positive, valid step size.");
        return;
    }

    qDebug() << className << "::slot_moveStepBackward id" << id << "step_um" << stepUm;
    startAxisMotionTask(id, "Relative Motion",
        [this, id, stepUm]() { return moveStepsAxisIndex(-stepUm, id, false); });
}

void thorlabsKinesisPlugin::slot_stopMotor()
{
    qDebug() << className << "::slot_stopMotor";
    if (!stop())
        QMessageBox::warning(dock, "Thorlabs", "Not all axes could be stopped.");
    stopGamepadJogs(false);
    m_gamepadSuppressUntilNeutral = true;

    // Close the narrow race in which a worker passed its cancellation check
    // immediately before the stop request and starts the SDK command just after it.
    QTimer::singleShot(100, this, [this]() {
        if (m_motionTaskActive.load()) stop();
    });
}

void thorlabsKinesisPlugin::slot_moveTo()
{
    const int id = ui.comboBox_devices->currentIndex();
    double posUm = 0.0;
    if (!parseFiniteDouble(ui.lineEdit_position->text(), posUm))
    {
        QMessageBox::warning(dock, "Thorlabs", "Please enter a valid target position.");
        return;
    }

    qDebug() << className << "::slot_moveTo id" << id << "pos_um" << posUm;
    startAxisMotionTask(id, "Absolute Motion",
        [this, id, posUm]() { return moveToAxisIndex(posUm, id, false); });
}

void thorlabsKinesisPlugin::slot_getPosition()
{
    const QString methodName = "slot_getPosition()";
    const int id = ui.comboBox_devices->currentIndex();
    if (id < 0 || id >= m_axes.size()) return;

    AxisEntry& ax = m_axes[id];

    qDebug() << qPrintable(className + "::" + methodName)
        << "- id" << id << "baseSerial" << ax.baseSerial << "ch" << ax.channel;

    double pUm = 0.0;
    if (!readAxisPositionUm(id, pUm))
        return;

    const QString text = mmDisplayWithUnitFromUm(pUm);
    ui.label_positionValue->setText(text);
    if (id < m_axisUi.size() && m_axisUi[id].positionLcd)
        m_axisUi[id].positionLcd->display(pUm / 1000.0);
}

void thorlabsKinesisPlugin::openPositionManager()
{
    if (!m_positionManagerWindow)
    {
        m_positionManagerWindow = new ThorlabsPositionManagerDialog(this, dock);
        connect(m_positionManagerWindow, &QObject::destroyed,
            this, [this]() { m_positionManagerWindow = nullptr; });
    }
    else if (!m_positionManagerWindow->isVisible())
    {
        m_positionManagerWindow->refresh();
    }

    m_positionManagerWindow->show();
    m_positionManagerWindow->raise();
    m_positionManagerWindow->activateWindow();
}

void thorlabsKinesisPlugin::slot_openPositionManager()
{
    qDebug() << className << "::slot_openPositionManager";
    openPositionManager();
}

void thorlabsKinesisPlugin::slot_applyTrigger()
{
    const int id = ui.comboBox_devices->currentIndex();
    if (id < 0 || id >= m_axes.size()) return;
    AxisEntry& ax = m_axes[id];

    qDebug() << className << "::slot_applyTrigger id" << id << "serial" << ax.baseSerial;

    double startUm = 0.0;
    double intervalUm = 0.0;
    int32_t count = 0;
    int32_t width = 0;
    const int32_t minimumPulseWidthUs = ax.isM30xy ? 1 : 1000;
    const int32_t maximumPulseWidthUs = ax.isM30xy ? 1000000 : 32767000;
    if (!parseFiniteDouble(ui.lineEdit_trigStart->text(), startUm)
        || !parseFiniteDouble(ui.lineEdit_trigInterval->text(), intervalUm)
        || intervalUm <= 0.0
        || !parseI32(ui.lineEdit_trigCount->text(), count)
        || count <= 0
        || !parseI32(ui.lineEdit_trigWidth->text(), width)
        || width < minimumPulseWidthUs
        || width > maximumPulseWidthUs)
    {
        QMessageBox::warning(dock, "Thorlabs",
            QString("Invalid trigger parameters. Spacing and count must be positive; pulse width must be between %1 and %2 us.")
                .arg(minimumPulseWidthUs)
                .arg(maximumPulseWidthUs));
        return;
    }

    short err = 0;
    double triggerVelocityMmS = 0.0;
    if (id >= 0 && id < m_axisUi.size() && m_axisUi[id].triggerVelocityEdit)
    {
        if (!parseFiniteDouble(m_axisUi[id].triggerVelocityEdit->text(), triggerVelocityMmS)
            || triggerVelocityMmS <= 0.0)
        {
            QMessageBox::warning(dock, "Thorlabs", "Please enter a positive, valid trigger velocity.");
            return;
        }
    }
    else
    {
        triggerVelocityMmS = ax.isM30xy
            ? m30xyForBase(ax.baseSerial)->getMaxVelocityMmS(static_cast<unsigned>(ax.channel), &err)
            : kvsForSerial(ax.baseSerial)->getMaxVelocityMmS(&err);
        if (err != 0 || !std::isfinite(triggerVelocityMmS) || triggerVelocityMmS <= 0.0)
        {
            QMessageBox::warning(dock, "Thorlabs", QString("Velocity read failed, err=%1").arg(err));
            return;
        }
    }

    const double intervalMm = intervalUm / 1000.0;
    const double triggerFrequencyHz = triggerVelocityMmS / intervalMm;
    if (!std::isfinite(triggerFrequencyHz)
        || triggerFrequencyHz > kMaximumLaserTriggerFrequencyHz + 1e-9)
    {
        const double minimumSpacingMm = triggerVelocityMmS / kMaximumLaserTriggerFrequencyHz;
        const double maximumVelocityMmS = kMaximumLaserTriggerFrequencyHz * intervalMm;
        QMessageBox::warning(dock, "Thorlabs",
            QString("The trigger velocity would generate %1 Hz; the maximum allowed is 20 Hz.\n\n"
                    "Increase spacing to at least %2 mm or reduce velocity to at most %3 mm/s.")
                .arg(triggerFrequencyHz, 0, 'f', 3)
                .arg(minimumSpacingMm, 0, 'f', 3)
                .arg(maximumVelocityMmS, 0, 'f', 3));
        return;
    }

    const double triggerPeriodUs = 1000000.0 / triggerFrequencyHz;
    if (static_cast<double>(width) > triggerPeriodUs + 1e-9)
    {
        QMessageBox::warning(dock, "Thorlabs",
            "The pulse width is longer than the time between two trigger pulses.");
        return;
    }

    if (ax.isM30xy)
    {
        BDCStage* st = m30xyForBase(ax.baseSerial);
        if (!st->isOpen())
        {
            QMessageBox::warning(dock, "Thorlabs", "The selected M30XY stage is not open.");
            return;
        }

        const int32_t startDev = st->umToDevice(startUm, static_cast<unsigned>(ax.channel), &err);
        if (err != 0)
        {
            QMessageBox::warning(dock, "Thorlabs", QString("Trigger start conversion failed, err=%1").arg(err));
            return;
        }

        const int32_t intervalDev = st->umToDevice(intervalUm, static_cast<unsigned>(ax.channel), &err);
        if (err != 0 || intervalDev <= 0)
        {
            QMessageBox::warning(dock, "Thorlabs", QString("Trigger interval conversion failed, err=%1").arg(err));
            return;
        }

        if (!st->setMaxVelocityMmS(static_cast<unsigned>(ax.channel), triggerVelocityMmS, &err))
        {
            QMessageBox::warning(dock, "Thorlabs", QString("Velocity setup failed, err=%1").arg(err));
            return;
        }

        BDCTriggerConfig cfg;
        cfg.enabled = true;
        cfg.trigger1Mode = static_cast<int>(BDCTriggerMode::PositionStepBoth);
        cfg.trigger1Polarity = static_cast<int>(BDCTriggerPolarity::High);
        cfg.trigger2Mode = static_cast<int>(BDCTriggerMode::Disabled);
        cfg.trigger2Polarity = static_cast<int>(BDCTriggerPolarity::High);
        cfg.startPosFwd = startDev;
        cfg.intervalFwd = intervalDev;
        cfg.pulseCountFwd = count;
        cfg.startPosRev = startDev;
        cfg.intervalRev = intervalDev;
        cfg.pulseCountRev = count;
        cfg.pulseWidthUs = width;
        cfg.cycleCount = 1;

        st->setTriggerConfig(cfg);
        if (!st->applyTriggerConfig(static_cast<unsigned>(ax.channel), &err))
            QMessageBox::warning(dock, "Thorlabs", QString("Apply trigger failed, err=%1").arg(err));
        return;
    }

    KVSStage* st = kvsForSerial(ax.baseSerial);
    if (!st->isOpen())
    {
        QMessageBox::warning(dock, "Thorlabs", "The selected KVS stage is not open.");
        return;
    }

    const int32_t startDev = st->umToDevice(startUm, &err);
    if (err != 0)
    {
        QMessageBox::warning(dock, "Thorlabs", QString("Trigger start conversion failed, err=%1").arg(err));
        return;
    }

    const int32_t intervalDev = st->umToDevice(intervalUm, &err);
    if (err != 0 || intervalDev <= 0)
    {
        QMessageBox::warning(dock, "Thorlabs", QString("Trigger interval conversion failed, err=%1").arg(err));
        return;
    }

    if (!st->setMaxVelocityMmS(triggerVelocityMmS, &err))
    {
        QMessageBox::warning(dock, "Thorlabs", QString("Velocity setup failed, err=%1").arg(err));
        return;
    }

    KVSTriggerConfig cfg;
    cfg.enabled = true;
    cfg.trigger1Mode = static_cast<int>(KVSTriggerMode::PositionSteps);
    cfg.trigger1Polarity = static_cast<int>(KVSTriggerPolarity::High);
    cfg.trigger2Mode = static_cast<int>(KVSTriggerMode::Disabled);
    cfg.trigger2Polarity = static_cast<int>(KVSTriggerPolarity::High);
    cfg.startPosFwd = startDev;
    cfg.intervalFwd = intervalDev;
    cfg.pulseCountFwd = count;
    cfg.startPosRev = startDev;
    cfg.intervalRev = intervalDev;
    cfg.pulseCountRev = count;
    cfg.pulseWidthUs = width;
    cfg.cycleCount = 1;

    st->setTriggerConfig(cfg);
    if (!st->applyTriggerConfig(&err))
        QMessageBox::warning(dock, "Thorlabs", QString("Apply trigger failed, err=%1").arg(err));
}

void thorlabsKinesisPlugin::slot_disableTrigger()
{
    const int id = ui.comboBox_devices->currentIndex();
    if (id < 0 || id >= m_axes.size()) return;
    AxisEntry& ax = m_axes[id];

    qDebug() << className << "::slot_disableTrigger id" << id << "serial" << ax.baseSerial;

    short err = 0;

    if (ax.isM30xy)
    {
        if (!m30xyForBase(ax.baseSerial)->disableTrigger((unsigned)ax.channel, true, &err))
            QMessageBox::warning(dock, "Thorlabs", QString("Disable trigger failed, err=%1").arg(err));
    }
    else
    {
        if (!kvsForSerial(ax.baseSerial)->disableTrigger(&err))
            QMessageBox::warning(dock, "Thorlabs", QString("Disable trigger failed, err=%1").arg(err));
    }
}
