#include "thorlabsKinesisPlugin.h"

#include <QMessageBox>
#include <mutex>
#include <array>

static int32_t toI32(const QString& s, int32_t fallback = 0)
{
    bool ok = false;
    long long v = s.toLongLong(&ok);
    return ok ? (int32_t)v : fallback;
}

static double toD(const QString& s, double fallback = 0.0)
{
    bool ok = false;
    double v = s.toDouble(&ok);
    return ok ? v : fallback;
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

    qDebug() << className << "::ctor created";
}

thorlabsKinesisPlugin::~thorlabsKinesisPlugin()
{
    release();
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
    static std::mutex mtx;
    std::lock_guard<std::mutex> lock(mtx);
    const std::string key = baseSerial.toStdString();
    auto it = m_m30xy.find(key);
    if (it == m_m30xy.end())
        it = m_m30xy.emplace(key, std::make_unique<BDCStage>(key)).first;
    return it->second.get();
}

KVSStage* thorlabsKinesisPlugin::kvsForSerial(const QString& serial)
{
    static std::mutex mtx;
    std::lock_guard<std::mutex> lock(mtx);
    const std::string key = serial.toStdString();
    auto it = m_kvs.find(key);
    if (it == m_kvs.end())
        it = m_kvs.emplace(key, std::make_unique<KVSStage>(key)).first;
    return it->second.get();
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
        if (it == m_m30xy.end() || !it->second)
            return false;

        valueUm = it->second->getPositionUm(ax.channel);
        return true;
    }
    else
    {
        auto it = m_kvs.find(ax.baseSerial.toStdString());
        if (it == m_kvs.end() || !it->second)
            return false;

        valueUm = it->second->getPositionUm();
        return true;
    }
}

bool thorlabsKinesisPlugin::detect()
{
    const QString methodName = "detect()";
    qDebug() << qPrintable(className + "::" + methodName) << "- start";

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

        if (err != 0) okAll = false;
    }

    for (const QString& s : kvsSerials)
    {
        short err = 0;
        qDebug() << qPrintable(className + "::" + methodName) << "- opening KVS serial" << s;

        const bool openedNow = kvsForSerial(s)->open(s.toStdString(), true, &err);
        qDebug() << qPrintable(className + "::" + methodName)
            << "- KVS serial" << s << "openResult(openedNow)=" << openedNow << "err=" << err;

        if (err != 0) okAll = false;
    }

    initGUI();
    setInitialized(okAll);

    qDebug() << qPrintable(className + "::" + methodName) << "- done initialized=" << isInitialized();
    return isInitialized();
}

bool thorlabsKinesisPlugin::release()
{
    const QString methodName = "release()";
    qDebug() << qPrintable(className + "::" + methodName) << "- start";

    m_m30xy.clear();
    m_kvs.clear();
    m_axes.clear();

    setInitialized(false);
    setDetected(false);

    qDebug() << qPrintable(className + "::" + methodName) << "- done";
    return true;
}

bool thorlabsKinesisPlugin::moveTo(double pos, int id)
{
    const QString methodName = "moveTo(pos_um,id)";

    if (id < 0 || id >= m_axes.size()) return false;

    AxisEntry& ax = m_axes[id];

    qDebug() << qPrintable(className + "::" + methodName)
        << "- id" << id
        << "baseSerial" << ax.baseSerial
        << "ch" << ax.channel
        << "pos_um" << pos;

    short err = 0;

    if (ax.isM30xy)
        return m30xyForBase(ax.baseSerial)->moveToUm(pos, ax.channel, &err);

    return kvsForSerial(ax.baseSerial)->moveToUm(pos, &err);
}

bool thorlabsKinesisPlugin::moveTo(double* pos, const char* axes)
{
    if (!pos || !axes)
        return false;

    const QString req = QString::fromLatin1(axes).toLower();
    if (req.isEmpty())
        return false;

    for (int i = 0; i < req.size(); ++i)
    {
        const int id = findAxisIndexByName(req[i]);
        if (id < 0)
            return false;

        if (!moveTo(pos[i], id))
            return false;
    }

    return true;
}

bool thorlabsKinesisPlugin::moveSteps(double steps, int id)
{
    const QString methodName = "moveSteps(delta_um,id)";

    if (id < 0 || id >= m_axes.size()) return false;

    AxisEntry& ax = m_axes[id];

    qDebug() << qPrintable(className + "::" + methodName)
        << "- id" << id
        << "baseSerial" << ax.baseSerial
        << "ch" << ax.channel
        << "delta_um" << steps;

    short err = 0;

    if (ax.isM30xy)
        return m30xyForBase(ax.baseSerial)->moveRelUm(steps, ax.channel, &err);

    return kvsForSerial(ax.baseSerial)->moveRelUm(steps, &err);
}

// Zusatzfunktion fuer mehrere relative Achsen gleichzeitig, z.B. "xyz"
bool thorlabsKinesisPlugin::moveSteps(double* steps, const char* axes)
{
    if (!steps || !axes)
        return false;

    const QString req = QString::fromLatin1(axes).toLower();
    if (req.isEmpty())
        return false;

    for (int i = 0; i < req.size(); ++i)
    {
        const int id = findAxisIndexByName(req[i]);
        if (id < 0)
            return false;

        if (!moveSteps(steps[i], id))
            return false;
    }

    return true;
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
    const int id = ui.comboBox_devices->currentIndex();

    if (id < 0 || id >= m_axes.size()) return false;

    AxisEntry& ax = m_axes[id];

    qDebug() << qPrintable(className + "::" + methodName)
        << "- id" << id << "baseSerial" << ax.baseSerial << "ch" << ax.channel;

    short err = 0;

    if (ax.isM30xy)
        return m30xyForBase(ax.baseSerial)->stopImmediate(ax.channel, &err);

    return kvsForSerial(ax.baseSerial)->stopImmediate(&err);
}

bool thorlabsKinesisPlugin::isStopped()
{
    return true;
}

// legacy stubs kept
void thorlabsKinesisPlugin::setHWSerialNr(QString* /*serialNr*/) {}
void thorlabsKinesisPlugin::setHWSerialNr(QString /*serialNr*/) {}
void thorlabsKinesisPlugin::setStepSize(QString /*stepSize*/) {}
void thorlabsKinesisPlugin::setVelocityParameters(QString, QString, QString) {}

void thorlabsKinesisPlugin::initGUI()
{
    disconnect(ui.pushButton_refresh, 0, this, 0);

    connect(ui.pushButton_refresh, SIGNAL(clicked()), this, SLOT(slot_refresh()));
    connect(ui.comboBox_devices, SIGNAL(currentIndexChanged(int)), this, SLOT(slot_deviceChanged(int)));

    connect(ui.pushButton_home, SIGNAL(clicked()), this, SLOT(slot_goHome()));
    connect(ui.pushButton_stop, SIGNAL(clicked()), this, SLOT(slot_stopMotor()));

    connect(ui.pushButton_position, SIGNAL(clicked()), this, SLOT(slot_moveTo()));
    connect(ui.lineEdit_position, SIGNAL(returnPressed()), ui.pushButton_position, SLOT(click()));

    connect(ui.pushButton_stepUp, SIGNAL(clicked()), this, SLOT(slot_moveStepForward()));
    connect(ui.pushButton_stepDown, SIGNAL(clicked()), this, SLOT(slot_moveStepBackward()));

    connect(ui.pushButton_getPosition, SIGNAL(clicked()), this, SLOT(slot_getPosition()));
    connect(ui.pushButton_logger, SIGNAL(clicked()), this, SLOT(slot_openLogger()));

    connect(ui.pushButton_applyTrigger, SIGNAL(clicked()), this, SLOT(slot_applyTrigger()));
    connect(ui.pushButton_disableTrigger, SIGNAL(clicked()), this, SLOT(slot_disableTrigger()));

    qDebug() << className << "::initGUI - signals connected";
}

void thorlabsKinesisPlugin::slot_refresh()
{
    qDebug() << className << "::slot_refresh";
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

    AxisEntry& ax = m_axes[id];
    qDebug() << qPrintable(className + "::" + methodName)
        << "- id" << id << "baseSerial" << ax.baseSerial;

    short err = 0;

    if (ax.isM30xy)
    {
        if (!m30xyForBase(ax.baseSerial)->homeAll(&err))
            QMessageBox::warning(dock, "Thorlabs", QString("HomeAll failed, err=%1").arg(err));
    }
    else
    {
        if (!kvsForSerial(ax.baseSerial)->home(&err))
            QMessageBox::warning(dock, "Thorlabs", QString("Home failed, err=%1").arg(err));
    }
}

void thorlabsKinesisPlugin::slot_moveStepForward()
{
    const int id = ui.comboBox_devices->currentIndex();
    const double stepUm = toD(ui.lineEdit_step->text(), 100.0);
    qDebug() << className << "::slot_moveStepForward id" << id << "step_um" << stepUm;
    moveSteps(stepUm, id);
}

void thorlabsKinesisPlugin::slot_moveStepBackward()
{
    const int id = ui.comboBox_devices->currentIndex();
    const double stepUm = toD(ui.lineEdit_step->text(), 100.0);
    qDebug() << className << "::slot_moveStepBackward id" << id << "step_um" << stepUm;
    moveSteps(-stepUm, id);
}

void thorlabsKinesisPlugin::slot_stopMotor()
{
    qDebug() << className << "::slot_stopMotor";
    stop();
}

void thorlabsKinesisPlugin::slot_moveTo()
{
    const int id = ui.comboBox_devices->currentIndex();
    const double posUm = toD(ui.lineEdit_position->text(), 0.0);

    qDebug() << className << "::slot_moveTo id" << id << "pos_um" << posUm;
    moveTo(posUm, id);
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

    ui.label_positionValue->setText(QString::number(pUm, 'f', 3) + " µm");
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

    const double startUm = toD(ui.lineEdit_trigStart->text(), 0.0);
    const double intervalUm = toD(ui.lineEdit_trigInterval->text(), 0.0);
    const int32_t count = toI32(ui.lineEdit_trigCount->text(), 1);
    const int32_t width = toI32(ui.lineEdit_trigWidth->text(), 50000);

    short err = 0;

    if (ax.isM30xy)
    {
        auto* st = m30xyForBase(ax.baseSerial);

        const int32_t startDev = st->umToDevice(startUm, (unsigned)ax.channel, &err);
        if (err != 0)
        {
            QMessageBox::warning(dock, "Thorlabs", QString("Trigger start conversion failed, err=%1").arg(err));
            return;
        }

        const int32_t intervalDev = st->umToDevice(intervalUm, (unsigned)ax.channel, &err);
        if (err != 0)
        {
            QMessageBox::warning(dock, "Thorlabs", QString("Trigger interval conversion failed, err=%1").arg(err));
            return;
        }

        BDCTriggerConfig cfg;
        cfg.enabled = true;
        cfg.trigger1Mode = 0x0F;       // both directions
        cfg.trigger1Polarity = 0x01;   // high
        cfg.trigger2Mode = 0x00;       // disabled
        cfg.trigger2Polarity = 0x01;

        cfg.startPosFwd = startDev;
        cfg.intervalFwd = intervalDev;
        cfg.pulseCountFwd = count;
        cfg.startPosRev = startDev;
        cfg.intervalRev = intervalDev;
        cfg.pulseCountRev = count;
        cfg.pulseWidthUs = width;

        st->setTriggerConfig(cfg);

        if (!st->applyTriggerConfig((unsigned)ax.channel, &err))
            QMessageBox::warning(dock, "Thorlabs", QString("Apply trigger failed, err=%1").arg(err));
    }
    else
    {
        auto* st = kvsForSerial(ax.baseSerial);

        const int32_t startDev = st->umToDevice(startUm, &err);
        if (err != 0)
        {
            QMessageBox::warning(dock, "Thorlabs", QString("Trigger start conversion failed, err=%1").arg(err));
            return;
        }

        const int32_t intervalDev = st->umToDevice(intervalUm, &err);
        if (err != 0)
        {
            QMessageBox::warning(dock, "Thorlabs", QString("Trigger interval conversion failed, err=%1").arg(err));
            return;
        }

        KVSTriggerConfig cfg;
        cfg.enabled = true;
        cfg.trigger1Mode = 0x0D;       // at position steps
        cfg.trigger1Polarity = 0x01;   // high
        cfg.trigger2Mode = 0x00;       // disabled
        cfg.trigger2Polarity = 0x01;

        cfg.startPosFwd = startDev;
        cfg.intervalFwd = intervalDev;
        cfg.pulseCountFwd = count;
        cfg.startPosRev = startDev;
        cfg.intervalRev = intervalDev;
        cfg.pulseCountRev = count;
        cfg.pulseWidthUs = width;

        st->setTriggerConfig(cfg);

        if (!st->applyTriggerConfig(&err))
            QMessageBox::warning(dock, "Thorlabs", QString("Apply trigger failed, err=%1").arg(err));
    }
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
        if (!m30xyForBase(ax.baseSerial)->disableTrigger((unsigned)ax.channel, &err))
            QMessageBox::warning(dock, "Thorlabs", QString("Disable trigger failed, err=%1").arg(err));
    }
    else
    {
        if (!kvsForSerial(ax.baseSerial)->disableTrigger(&err))
            QMessageBox::warning(dock, "Thorlabs", QString("Disable trigger failed, err=%1").arg(err));
    }
}