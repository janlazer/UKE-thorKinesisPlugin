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

#include <QCheckBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFrame>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMenu>
#include <QMessageBox>
#include <QMetaObject>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSize>
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
    constexpr int kStageFrameWidth = 590;
    constexpr int kSerialButtonHeight = 24;

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
            "QLineEdit, QComboBox { min-height: 22px; max-height: 24px; }"));
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
    const double parsed = s.trimmed().toDouble(&ok);
    if (!ok || !std::isfinite(parsed))
        return false;

    value = parsed;
    return true;
}

static QString mmDisplayTextFromUm(double valueUm)
{
    return QString::number(valueUm / 1000.0, 'f', 3);
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

    return QString::number(velocityMmS / spacingMm, 'f', 3);
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

    dock = new QDockWidget();
    ui.setupUi(dock);
    dock->setMinimumWidth(kStageFrameWidth + 8);
    if (dock->widget())
        dock->widget()->setMinimumWidth(kStageFrameWidth);
    ui.scrollArea_axes->setMinimumWidth(kStageFrameWidth);
    ui.scrollAreaWidgetContents_axes->setMinimumWidth(kStageFrameWidth);
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
    return map;
}

void thorlabsKinesisPlugin::setLoadValueInformation(QMap<QString, QString> /*map*/)
{
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
    const bool anyBusy = busy || m_motionTaskActive.load() || !m_axisMotionThreads.isEmpty();

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
    {
        double posMm = 0.0;
        if (parseFiniteDouble(axisUi.positionEdit->text(), posMm))
            ui.lineEdit_position->setText(QString::number(posMm * 1000.0, 'f', 3));
        else
            ui.lineEdit_position->setText(axisUi.positionEdit->text());
    }
    if (axisUi.stepEdit)
    {
        double stepMm = 0.0;
        if (parseFiniteDouble(axisUi.stepEdit->text(), stepMm))
            ui.lineEdit_step->setText(QString::number(stepMm * 1000.0, 'f', 3));
        else
            ui.lineEdit_step->setText(axisUi.stepEdit->text());
    }
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
            QStringLiteral("background-color: %1; color: black; border: 1px solid gray; padding: 2px 6px;")
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
        QStringLiteral("lineEdit_triggerStart"), QStringLiteral("0.000"), 82, panel);
    axisUi.triggerIntervalEdit = makeEdit(
        QStringLiteral("lineEdit_triggerInterval"), QStringLiteral("0.100"), 82, panel);
    axisUi.triggerCountEdit = makeEdit(
        QStringLiteral("lineEdit_triggerCount"), QStringLiteral("1"), 74, panel);
    axisUi.triggerWidthEdit = makeEdit(
        QStringLiteral("lineEdit_triggerWidth"), QStringLiteral("500"), 82, panel);
    axisUi.triggerVelocityEdit = makeEdit(
        QStringLiteral("lineEdit_triggerVelocity"), QStringLiteral("2.000"), 82, panel);
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
            ui.scrollAreaWidgetContents_axes);
        emptyLabel->setAlignment(Qt::AlignCenter);
        emptyLabel->setWordWrap(true);
        emptyLabel->setMinimumHeight(80);
        ui.axisFramesLayout->addWidget(emptyLabel);
        ui.axisFramesLayout->addStretch(1);
        return;
    }

    m_axisUi.resize(m_axes.size());
    QMap<QString, QVBoxLayout*> serialLayouts;

    for (int id = 0; id < m_axes.size(); ++id)
    {
        const AxisEntry& ax = m_axes[id];
        AxisUi axisUi;

        auto* frame = new QFrame(ui.scrollAreaWidgetContents_axes);
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
        axisUi.triggerMenuButton = stageFrameUi.toolButton_triggerMenu;
        setupAxisTriggerMenu(axisUi, frame);

        stageFrameUi.label_globalStageID->setText(QString("%1.").arg(ax.globalAxisId));
        axisUi.title->setText(ax.axisName.isEmpty()
            ? ax.display
            : QString("%1 %2").arg(ax.isM30xy ? QStringLiteral("M30XY") : QStringLiteral("KVS30"), ax.axisName));
        frame->setToolTip(QStringLiteral("Serial: %1").arg(ax.baseSerial));
        axisUi.positionEdit->setText(QStringLiteral("0.000"));
        axisUi.stepEdit->setText(ax.isM30xy ? QStringLiteral("0.100") : QStringLiteral("0.010"));
        axisUi.triggerStartEdit->setText(QStringLiteral("0.000"));
        axisUi.triggerIntervalEdit->setText(QStringLiteral("0.100"));
        axisUi.triggerCountEdit->setText(QStringLiteral("1"));
        axisUi.triggerWidthEdit->setText(ax.isM30xy ? QStringLiteral("500") : QStringLiteral("1000"));
        axisUi.positionLcd->display(0.0);

        {
            QString velocityText = QStringLiteral("2.000");
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
                            velocityText = QString::number(velocityMmS, 'f', 3);
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
                            velocityText = QString::number(velocityMmS, 'f', 3);
                    }
                }
            }

            axisUi.triggerVelocityEdit->setText(velocityText);
        }
        frame->setMinimumWidth(kStageFrameWidth);

        const QString serialKey = ax.baseSerial.isEmpty() ? axisKey(ax) : ax.baseSerial;
        if (!serialLayouts.contains(serialKey))
        {
            const QString serialTitle = QString("%1 %2")
                .arg(ax.isM30xy ? QStringLiteral("M30XY") : QStringLiteral("KVS30"), serialKey);
            auto* serialButton = new QPushButton(serialTitle + QStringLiteral(" >>>"),
                ui.scrollAreaWidgetContents_axes);
            serialButton->setCheckable(true);
            serialButton->setMinimumWidth(kStageFrameWidth);
            serialButton->setFixedHeight(kSerialButtonHeight);
            ui.axisFramesLayout->addWidget(serialButton, 0, Qt::AlignLeft);

            auto* serialWidget = new QWidget(ui.scrollAreaWidgetContents_axes);
            serialWidget->setMinimumWidth(kStageFrameWidth);
            auto* serialLayout = new QVBoxLayout(serialWidget);
            serialLayout->setContentsMargins(0, 0, 0, 0);
            serialLayout->setSpacing(4);
            ui.axisFramesLayout->addWidget(serialWidget, 0, Qt::AlignLeft);

            connect(serialButton, &QPushButton::toggled, this,
                [serialButton, serialWidget, serialTitle](bool collapsed) {
                    serialWidget->setVisible(!collapsed);
                    serialButton->setText(serialTitle + (collapsed
                        ? QStringLiteral(" <<<")
                        : QStringLiteral(" >>>")));
                });
            serialLayouts.insert(serialKey, serialLayout);
        }

        serialLayouts.value(serialKey)->addWidget(frame);

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
            refreshAxisStatusUi(id);
        });
        connect(axisUi.positionEdit, &QLineEdit::returnPressed,
            axisUi.moveButton, &QPushButton::click);
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

    ui.axisFramesLayout->addStretch(1);
    setMotionUiBusy(m_motionTaskActive.load());
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
    disableAllTriggers();
    closeDevices();
    setInitialized(false);

    m_detectedAxes.clear();
    m_axes.clear();
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
    disableAllTriggers();
    closeDevices();
    m_detectedAxes.clear();
    m_axes.clear();
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
