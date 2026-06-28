#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "thorlabsKinesisPlugin.h"

#include <QFont>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSpacerItem>
#include <QTimer>
#include <QVBoxLayout>
#include <array>

namespace
{
    constexpr double kMaximumLaserTriggerFrequencyHz = 20.0;
    constexpr double kScanLineAxisToleranceUm = 1e-6;
    constexpr int kPositionRefreshIntervalMs = 1000;

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
    dock->setWindowTitle(getName());

    dockLogger = new QDockWidget();
    dockLogger->setWindowTitle(getName() + QString(" - Logger"));
    dockLogger->hide();

    connect(dock, &QObject::destroyed, this, [this]() { dock = nullptr; });
    connect(dockLogger, &QObject::destroyed, this, [this]() { dockLogger = nullptr; });

    initGUI();

    qDebug() << className << "::ctor created";
}

thorlabsKinesisPlugin::~thorlabsKinesisPlugin()
{
    release();

    if (dockLogger)
        delete dockLogger;
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
    slot_openLogger();
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

void thorlabsKinesisPlugin::setMotionUiBusy(bool busy)
{
    ui.pushButton_refresh->setEnabled(!busy);
    ui.comboBox_devices->setEnabled(!busy);
    ui.pushButton_home->setEnabled(!busy);
    ui.pushButton_position->setEnabled(!busy);
    ui.pushButton_stepUp->setEnabled(!busy);
    ui.pushButton_stepDown->setEnabled(!busy);
    ui.pushButton_applyTrigger->setEnabled(!busy);
    ui.pushButton_disableTrigger->setEnabled(!busy);
    ui.pushButton_stop->setEnabled(busy || isInitialized());

    const bool axisControlsEnabled = !busy && isInitialized();
    for (int id = 0; id < m_axisUi.size(); ++id)
    {
        AxisUi& axisUi = m_axisUi[id];
        const bool triggerControlsEnabled =
            axisControlsEnabled && id >= 0 && id < m_axes.size() && m_axes[id].isM30xy;

        if (axisUi.homeButton) axisUi.homeButton->setEnabled(axisControlsEnabled);
        if (axisUi.moveButton) axisUi.moveButton->setEnabled(axisControlsEnabled);
        if (axisUi.stepDownButton) axisUi.stepDownButton->setEnabled(axisControlsEnabled);
        if (axisUi.stepUpButton) axisUi.stepUpButton->setEnabled(axisControlsEnabled);
        if (axisUi.applyTriggerButton) axisUi.applyTriggerButton->setEnabled(triggerControlsEnabled);
        if (axisUi.disableTriggerButton) axisUi.disableTriggerButton->setEnabled(triggerControlsEnabled);
    }
}

void thorlabsKinesisPlugin::startMotionTask(const QString& operation, std::function<bool()> task)
{
    if (m_motionThread)
    {
        QMessageBox::warning(dock, "Thorlabs", "Eine Bewegung läuft bereits.");
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
                QMessageBox::warning(dock, "Thorlabs", operation + " fehlgeschlagen.");
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

void thorlabsKinesisPlugin::waitForMotionToFinish()
{
    QThread* thread = m_motionThread;
    if (!thread)
        return;

    stop();
    while (!thread->wait(100))
        stop();
    disconnect(thread, nullptr, this, nullptr);
    if (m_motionThread == thread) m_motionThread = nullptr;
    m_motionTaskActive.store(false);
    delete thread;
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
        ui.lineEdit_position->setText(axisUi.positionEdit->text());
    if (axisUi.stepEdit)
        ui.lineEdit_step->setText(axisUi.stepEdit->text());
}

void thorlabsKinesisPlugin::syncLegacyTriggerInputsFromAxisUi(int id)
{
    if (id < 0 || id >= m_axisUi.size())
        return;

    const AxisUi& axisUi = m_axisUi[id];
    if (axisUi.triggerStartEdit)
        ui.lineEdit_trigStart->setText(axisUi.triggerStartEdit->text());
    if (axisUi.triggerIntervalEdit)
        ui.lineEdit_trigInterval->setText(axisUi.triggerIntervalEdit->text());
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
    const QString text = readAxisPositionUm(id, positionUm)
        ? QString::number(positionUm, 'f', 3) + " um"
        : QStringLiteral("n/a");

    if (m_axisUi[id].positionValue)
        m_axisUi[id].positionValue->setText(text);

    if (ui.comboBox_devices->currentIndex() == id)
        ui.label_positionValue->setText(text);
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
        auto* emptyLabel = new QLabel("Keine Achsen erkannt. Bitte Refresh / Detect ausfuehren.",
            ui.scrollAreaWidgetContents_axes);
        emptyLabel->setAlignment(Qt::AlignCenter);
        emptyLabel->setWordWrap(true);
        emptyLabel->setMinimumHeight(80);
        ui.axisFramesLayout->addWidget(emptyLabel);
        ui.axisFramesLayout->addStretch(1);
        return;
    }

    m_axisUi.resize(m_axes.size());

    for (int id = 0; id < m_axes.size(); ++id)
    {
        const AxisEntry& ax = m_axes[id];
        AxisUi axisUi;

        auto* frame = new QFrame(ui.scrollAreaWidgetContents_axes);
        frame->setObjectName(QString("axisFrame_%1").arg(id));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        frame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);

        auto* frameLayout = new QVBoxLayout(frame);
        frameLayout->setContentsMargins(10, 10, 10, 10);
        frameLayout->setSpacing(8);

        axisUi.title = new QLabel(ax.display, frame);
        QFont titleFont = axisUi.title->font();
        titleFont.setBold(true);
        axisUi.title->setFont(titleFont);
        frameLayout->addWidget(axisUi.title);

        auto* grid = new QGridLayout();
        grid->setColumnStretch(1, 1);
        grid->setColumnStretch(3, 1);

        axisUi.serial = new QLabel(ax.baseSerial, frame);
        axisUi.positionEdit = new QLineEdit("0", frame);
        axisUi.stepEdit = new QLineEdit(ax.isM30xy ? "100" : "10", frame);
        axisUi.positionValue = new QLabel("n/a", frame);
        axisUi.positionValue->setMinimumWidth(90);

        axisUi.homeButton = new QPushButton("Home", frame);
        axisUi.moveButton = new QPushButton("Move", frame);
        axisUi.stepDownButton = new QPushButton("Step-", frame);
        axisUi.stepUpButton = new QPushButton("Step+", frame);

        grid->addWidget(new QLabel("Serial:", frame), 0, 0);
        grid->addWidget(axisUi.serial, 0, 1, 1, 3);

        grid->addWidget(new QLabel("Target [um]:", frame), 1, 0);
        grid->addWidget(axisUi.positionEdit, 1, 1);
        grid->addWidget(axisUi.moveButton, 1, 2);
        grid->addWidget(axisUi.homeButton, 1, 3);

        grid->addWidget(new QLabel("Step [um]:", frame), 2, 0);
        grid->addWidget(axisUi.stepEdit, 2, 1);
        auto* stepButtons = new QWidget(frame);
        auto* stepButtonsLayout = new QHBoxLayout(stepButtons);
        stepButtonsLayout->setContentsMargins(0, 0, 0, 0);
        stepButtonsLayout->setSpacing(4);
        stepButtonsLayout->addWidget(axisUi.stepDownButton);
        stepButtonsLayout->addWidget(axisUi.stepUpButton);
        grid->addWidget(stepButtons, 2, 2, 1, 2);

        grid->addWidget(new QLabel("Current:", frame), 3, 0);
        grid->addWidget(axisUi.positionValue, 3, 1, 1, 3);

        if (ax.isM30xy)
        {
            axisUi.triggerStartEdit = new QLineEdit("0", frame);
            axisUi.triggerIntervalEdit = new QLineEdit("0", frame);
            axisUi.triggerCountEdit = new QLineEdit("1", frame);
            axisUi.triggerWidthEdit = new QLineEdit("50000", frame);
            axisUi.applyTriggerButton = new QPushButton("Apply Trigger", frame);
            axisUi.disableTriggerButton = new QPushButton("Disable Trigger", frame);

            grid->addWidget(new QLabel("Trigger start [um]:", frame), 4, 0);
            grid->addWidget(axisUi.triggerStartEdit, 4, 1);
            grid->addWidget(new QLabel("Spacing [um]:", frame), 4, 2);
            grid->addWidget(axisUi.triggerIntervalEdit, 4, 3);

            grid->addWidget(new QLabel("Pulse count:", frame), 5, 0);
            grid->addWidget(axisUi.triggerCountEdit, 5, 1);
            grid->addWidget(new QLabel("Pulse width [us]:", frame), 5, 2);
            grid->addWidget(axisUi.triggerWidthEdit, 5, 3);

            grid->addWidget(axisUi.applyTriggerButton, 6, 0, 1, 2);
            grid->addWidget(axisUi.disableTriggerButton, 6, 2, 1, 2);
        }

        frameLayout->addLayout(grid);
        ui.axisFramesLayout->addWidget(frame);

        axisUi.frame = frame;
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

    m_axes.clear();
    ui.comboBox_devices->clear();

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
            z.display = QString("KVS30 Z (S:%1)").arg(serial);
            m_axes.push_back(z);

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
            x.display = QString("M30XY X (Base:%1 ch1)").arg(serial);
            m_axes.push_back(x);

            AxisEntry y;
            y.baseSerial = serial;
            y.channel = 2;
            y.isM30xy = true;
            y.display = QString("M30XY Y (Base:%1 ch2)").arg(serial);
            m_axes.push_back(y);

            qDebug() << qPrintable(className + "::" + methodName)
                << "- detected M30XY base serial" << serial;
        }
    }

    for (const auto& ax : m_axes)
        ui.comboBox_devices->addItem(ax.display);

    setDetected(!m_axes.isEmpty());
    rebuildAxisFrames();

    if (isDetected())
    {
        ui.comboBox_devices->setCurrentIndex(0);
        refreshAxisUi();
    }

    qDebug() << qPrintable(className + "::" + methodName)
        << "- done detected=" << isDetected()
        << "axes=" << m_axes.size();

    return isDetected();
}

bool thorlabsKinesisPlugin::initialize()
{
    const QString methodName = "initialize()";
    qDebug() << qPrintable(className + "::" + methodName) << "- start";

    if (!isDetected() || m_axes.isEmpty())
    {
        qDebug() << qPrintable(className + "::" + methodName) << "- not detected / no axes";
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
    m_axes.clear();
    if (dock)
    {
        ui.comboBox_devices->clear();
        rebuildAxisFrames();
    }

    setInitialized(false);
    setDetected(false);
    if (dock)
        setMotionUiBusy(false);

    qDebug() << qPrintable(className + "::" + methodName) << "- done";
    return true;
}

bool thorlabsKinesisPlugin::moveTo(double pos, int id)
{
    const QString methodName = "moveTo(pos_um,id)";

    if (!isInitialized() || !std::isfinite(pos) || id < 0 || id >= m_axes.size())
        return false;
    if (m_motionTaskActive.load() && m_stopRequested.load())
        return false;

    AxisEntry& ax = m_axes[id];

    qDebug() << qPrintable(className + "::" + methodName)
        << "- id" << id
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
    if (axisCount < 1)
        return false;
    return moveToAxisCodes(positions, axes, static_cast<std::size_t>(axisCount));
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
    capabilities.controlsScanVelocity = false;
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

    const double currentVelocityMmS = stage->getMaxVelocityMmS(channel, &err);
    if (err != 0 || !std::isfinite(currentVelocityMmS) || currentVelocityMmS <= 0.0)
        return false;

    const double triggerFrequencyHz = currentVelocityMmS / (job.triggerSpacingUm / 1000.0);
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

    const QString req = QString::fromLatin1(axes).toLower();
    if (req.isEmpty())
        return false;

    if (!m_motionTaskActive.load())
        m_stopRequested.store(false);
    if (!isStopped())
        return false;

    struct PendingMove
    {
        BDCStage* bdc = nullptr;
        KVSStage* kvs = nullptr;
        unsigned channel = 1;
        int32_t targetDeviceUnits = 0;
    };

    std::vector<PendingMove> pending;
    pending.reserve(static_cast<std::size_t>(req.size()));
    QSet<QChar> seenAxes;

    // Resolve and convert every target before the first controller receives a
    // movement command. Relative targets are based on one common snapshot.
    for (int i = 0; i < req.size(); ++i)
    {
        const QChar axisName = req[i];
        if (seenAxes.contains(axisName) || !std::isfinite(values[i]))
            return false;
        seenAxes.insert(axisName);

        const int id = findAxisIndexByName(axisName);
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

bool thorlabsKinesisPlugin::moveSteps(double steps, int id)
{
    const QString methodName = "moveSteps(delta_um,id)";

    if (!isInitialized() || !std::isfinite(steps) || id < 0 || id >= m_axes.size())
        return false;
    if (m_motionTaskActive.load() && m_stopRequested.load())
        return false;

    AxisEntry& ax = m_axes[id];

    qDebug() << qPrintable(className + "::" + methodName)
        << "- id" << id
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

// Zusatzfunktion fuer mehrere relative Achsen gleichzeitig, z.B. "xyz"
bool thorlabsKinesisPlugin::moveSteps(double* steps, const char* axes)
{
    return moveAxesCoordinated(steps, axes, true);
}

bool thorlabsKinesisPlugin::moveSteps(const double* steps, const int* axes, int axisCount)
{
    if (!steps || !axes || axisCount < 1)
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
    m_lastPositionsUm = { 0.0, 0.0, 0.0 };

    QString req = axes ? QString::fromLatin1(axes).toLower() : QString("xyz");
    if (req.isEmpty())
        req = "xyz";

    for (int i = 0; i < req.size() && i < 3; ++i)
    {
        const int id = findAxisIndexByName(req[i]);
        if (id < 0)
            continue;

        double valueUm = 0.0;
        if (readAxisPositionUm(id, valueUm))
            m_lastPositionsUm[i] = valueUm;
    }

    return m_lastPositionsUm.data();
}

double* thorlabsKinesisPlugin::getPosition()
{
    return getPosition("xyz");
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

    connect(ui.pushButton_refresh, &QPushButton::clicked,
        this, &thorlabsKinesisPlugin::slot_refresh, Qt::UniqueConnection);
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

    connect(ui.pushButton_logger, &QPushButton::clicked,
        this, &thorlabsKinesisPlugin::slot_openLogger, Qt::UniqueConnection);

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

void thorlabsKinesisPlugin::slot_refresh()
{
    qDebug() << className << "::slot_refresh";
    if (m_motionThread)
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

    startMotionTask("Homing", [this, ax]() {
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
            return stage->isOpen() && stage->homeAll(&err);
        }

        KVSStage* stage = kvsForSerial(ax.baseSerial);
        return stage->isOpen() && stage->home(&err);
    });
}

void thorlabsKinesisPlugin::slot_moveStepForward()
{
    const int id = ui.comboBox_devices->currentIndex();
    double stepUm = 0.0;
    if (!parseFiniteDouble(ui.lineEdit_step->text(), stepUm) || stepUm <= 0.0)
    {
        QMessageBox::warning(dock, "Thorlabs", "Bitte eine positive, gültige Schrittweite eingeben.");
        return;
    }

    qDebug() << className << "::slot_moveStepForward id" << id << "step_um" << stepUm;
    startMotionTask("Relative Bewegung", [this, id, stepUm]() { return moveSteps(stepUm, id); });
}

void thorlabsKinesisPlugin::slot_moveStepBackward()
{
    const int id = ui.comboBox_devices->currentIndex();
    double stepUm = 0.0;
    if (!parseFiniteDouble(ui.lineEdit_step->text(), stepUm) || stepUm <= 0.0)
    {
        QMessageBox::warning(dock, "Thorlabs", "Bitte eine positive, gültige Schrittweite eingeben.");
        return;
    }

    qDebug() << className << "::slot_moveStepBackward id" << id << "step_um" << stepUm;
    startMotionTask("Relative Bewegung", [this, id, stepUm]() { return moveSteps(-stepUm, id); });
}

void thorlabsKinesisPlugin::slot_stopMotor()
{
    qDebug() << className << "::slot_stopMotor";
    if (!stop())
        QMessageBox::warning(dock, "Thorlabs", "Nicht alle Achsen konnten gestoppt werden.");

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
        QMessageBox::warning(dock, "Thorlabs", "Bitte eine gültige Zielposition eingeben.");
        return;
    }

    qDebug() << className << "::slot_moveTo id" << id << "pos_um" << posUm;
    startMotionTask("Absolute Bewegung", [this, id, posUm]() { return moveTo(posUm, id); });
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

    const QString text = QString::number(pUm, 'f', 3) + " um";
    ui.label_positionValue->setText(text);
    if (id < m_axisUi.size() && m_axisUi[id].positionValue)
        m_axisUi[id].positionValue->setText(text);
}

void thorlabsKinesisPlugin::slot_openLogger()
{
    qDebug() << className << "::slot_openLogger";
    if (!dockLogger) return;
    dockLogger->show();
    dockLogger->raise();
}

void thorlabsKinesisPlugin::slot_applyTrigger()
{
    const int id = ui.comboBox_devices->currentIndex();
    if (id < 0 || id >= m_axes.size()) return;
    AxisEntry& ax = m_axes[id];

    qDebug() << className << "::slot_applyTrigger id" << id << "serial" << ax.baseSerial;

    if (!ax.isM30xy)
    {
        QMessageBox::warning(dock, "Thorlabs", "Lasertrigger werden ausschließlich über die M30XY-Stage ausgegeben.");
        return;
    }

    double startUm = 0.0;
    double intervalUm = 0.0;
    int32_t count = 0;
    int32_t width = 0;
    if (!parseFiniteDouble(ui.lineEdit_trigStart->text(), startUm)
        || !parseFiniteDouble(ui.lineEdit_trigInterval->text(), intervalUm)
        || intervalUm <= 0.0
        || !parseI32(ui.lineEdit_trigCount->text(), count)
        || count <= 0
        || !parseI32(ui.lineEdit_trigWidth->text(), width)
        || width < 1
        || width > 1000000)
    {
        QMessageBox::warning(dock, "Thorlabs",
            "Ungültige Triggerparameter. Intervall und Anzahl müssen positiv sein; die Pulsbreite muss zwischen 1 und 1000000 µs liegen.");
        return;
    }

    short err = 0;

    auto* st = m30xyForBase(ax.baseSerial);
    if (!st->isOpen())
    {
        QMessageBox::warning(dock, "Thorlabs", "Die ausgewählte M30XY-Stage ist nicht geöffnet.");
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

    const double maxVelocityMmS = st->getMaxVelocityMmS(static_cast<unsigned>(ax.channel), &err);
    if (err != 0)
    {
        QMessageBox::warning(dock, "Thorlabs", QString("Velocity read failed, err=%1").arg(err));
        return;
    }

    const double triggerFrequencyHz = maxVelocityMmS / (intervalUm / 1000.0);
    if (!std::isfinite(triggerFrequencyHz)
        || triggerFrequencyHz > kMaximumLaserTriggerFrequencyHz + 1e-9)
    {
        QMessageBox::warning(dock, "Thorlabs",
            QString("Die aktuelle Geschwindigkeit würde %1 Hz erzeugen; erlaubt sind maximal 20 Hz.")
                .arg(triggerFrequencyHz, 0, 'f', 3));
        return;
    }

    const double triggerPeriodUs = 1000000.0 / triggerFrequencyHz;
    if (static_cast<double>(width) > triggerPeriodUs + 1e-9)
    {
        QMessageBox::warning(dock, "Thorlabs",
            "Die Pulsbreite ist größer als die Zeit zwischen zwei Triggerpulsen.");
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
