#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4005)
#endif

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

#include "ThorlabsPositionManagerDialog.h"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

namespace
{
    const QString DraftConfigText = QStringLiteral("New position config...");
}

ThorlabsPositionManagerDialog::ThorlabsPositionManagerDialog(thorlabsKinesisPlugin* plugin, QWidget* parent)
    : QDialog(parent), m_plugin(plugin)
{
    setWindowTitle("Thorlabs Position Manager");
    setModal(false);
    resize(680, 420);

    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(10, 10, 10, 10);
    mainLayout->setSpacing(8);

    auto* configLayout = new QGridLayout();
    configLayout->addWidget(new QLabel("Config:", this), 0, 0);
    m_configCombo = new QComboBox(this);
    configLayout->addWidget(m_configCombo, 0, 1, 1, 4);

    m_newButton = new QPushButton("New", this);
    m_removeButton = new QPushButton("Delete", this);
    configLayout->addWidget(m_newButton, 0, 5);
    configLayout->addWidget(m_removeButton, 0, 6);
    mainLayout->addLayout(configLayout);

    m_axisTable = new QTableWidget(this);
    m_axisTable->setColumnCount(4);
    m_axisTable->setHorizontalHeaderLabels(
        QStringList() << "Axis" << "Serial" << "Name" << "Position [mm]");
    m_axisTable->verticalHeader()->setVisible(false);
    m_axisTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_axisTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    m_axisTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    m_axisTable->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
    mainLayout->addWidget(m_axisTable, 1);

    auto* buttonLayout = new QGridLayout();
    m_getPositionsButton = new QPushButton("Get Current Positions", this);
    m_saveButton = new QPushButton("Save Config", this);
    m_goButton = new QPushButton("Go To Config", this);
    buttonLayout->addWidget(m_getPositionsButton, 0, 0);
    buttonLayout->addWidget(m_saveButton, 0, 1);
    buttonLayout->addWidget(m_goButton, 0, 2);
    mainLayout->addLayout(buttonLayout);

    connect(m_configCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [this](int index) { loadConfig(index); });
    connect(m_newButton, &QPushButton::released, this, [this]() { newConfig(); });
    connect(m_removeButton, &QPushButton::released, this, [this]() { removeConfig(); });
    connect(m_getPositionsButton, &QPushButton::released, this, [this]() { getPositions(); });
    connect(m_saveButton, &QPushButton::released, this, [this]() { saveConfig(); });
    connect(m_goButton, &QPushButton::released, this, [this]() { go(); });

    refresh();
}

void ThorlabsPositionManagerDialog::refresh()
{
    m_axes = m_plugin ? m_plugin->getAxisInfo() : QVector<thorlabsKinesisPlugin::AxisInfo>();
    rebuildAxisTable();
    rebuildConfigList();
}

void ThorlabsPositionManagerDialog::rebuildAxisTable()
{
    m_axisTable->clearContents();
    m_axisTable->setRowCount(m_axes.size());
    m_positionEditors.clear();
    m_positionEditors.reserve(m_axes.size());

    for (int row = 0; row < m_axes.size(); ++row)
    {
        const thorlabsKinesisPlugin::AxisInfo& axis = m_axes[row];

        auto* axisItem = new QTableWidgetItem(QString::number(axis.globalAxisID));
        axisItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsUserCheckable);
        axisItem->setCheckState(Qt::Checked);
        m_axisTable->setItem(row, 0, axisItem);

        auto* serialItem = new QTableWidgetItem(axis.serial);
        serialItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
        m_axisTable->setItem(row, 1, serialItem);

        auto* nameItem = new QTableWidgetItem(axis.axisName);
        nameItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
        m_axisTable->setItem(row, 2, nameItem);

        auto* positionEditor = new QDoubleSpinBox(m_axisTable);
        positionEditor->setButtonSymbols(QAbstractSpinBox::NoButtons);
        positionEditor->setDecimals(3);
        positionEditor->setKeyboardTracking(false);
        positionEditor->setAlignment(Qt::AlignRight);
        if (axis.travelLimitsValid)
            positionEditor->setRange(axis.minimumTravelUm / 1000.0, axis.maximumTravelUm / 1000.0);
        else
        {
            positionEditor->setRange(-1000000.0, 1000000.0);
            positionEditor->setEnabled(false);
            positionEditor->setToolTip("Travel limits unavailable");
        }
        positionEditor->setValue(0.0);
        m_axisTable->setCellWidget(row, 3, positionEditor);
        m_positionEditors.append(positionEditor);
    }
}

void ThorlabsPositionManagerDialog::rebuildConfigList()
{
    QSignalBlocker blocker(m_configCombo);
    m_configCombo->clear();
    m_configCombo->addItem(DraftConfigText);
    if (m_plugin)
        m_configCombo->addItems(m_plugin->getPositionConfigNames());

    int selectedIndex = 0;
    if (m_plugin)
    {
        const QString activeName = m_plugin->getActivePositionConfigName();
        const int activeIndex = m_configCombo->findText(activeName, Qt::MatchFixedString);
        if (activeIndex > 0)
            selectedIndex = activeIndex;
    }

    m_configCombo->setCurrentIndex(selectedIndex);
    blocker.unblock();
    loadConfig(selectedIndex);
}

void ThorlabsPositionManagerDialog::resetDraft()
{
    for (int row = 0; row < m_axes.size(); ++row)
    {
        if (QTableWidgetItem* axisItem = m_axisTable->item(row, 0))
            axisItem->setCheckState(Qt::Checked);
        if (row < m_positionEditors.size())
            m_positionEditors[row]->setValue(0.0);
    }
}

void ThorlabsPositionManagerDialog::loadConfig(int comboIndex)
{
    if (comboIndex <= 0 || !m_plugin)
    {
        resetDraft();
        updateButtons();
        return;
    }

    QVector<int> selectedAxisIDs;
    QVector<double> positionsUm;
    const QString configName = m_configCombo->itemText(comboIndex);
    if (!m_plugin->choosePositionConfig(configName)
        || !m_plugin->getPositionConfig(configName, selectedAxisIDs, positionsUm))
    {
        resetDraft();
        updateButtons();
        return;
    }

    for (int row = 0; row < m_axes.size(); ++row)
    {
        const int configAxisIndex = selectedAxisIDs.indexOf(m_axes[row].globalAxisID);
        if (QTableWidgetItem* axisItem = m_axisTable->item(row, 0))
            axisItem->setCheckState(configAxisIndex >= 0 ? Qt::Checked : Qt::Unchecked);

        if (row < m_positionEditors.size())
        {
            const double positionUm =
                configAxisIndex >= 0 && configAxisIndex < positionsUm.size()
                ? positionsUm[configAxisIndex]
                : 0.0;
            m_positionEditors[row]->setValue(positionUm / 1000.0);
        }
    }

    updateButtons();
}

void ThorlabsPositionManagerDialog::updateButtons()
{
    const bool hasAxes = !m_axes.isEmpty();
    const bool hasConfig = m_configCombo->currentIndex() > 0;
    m_newButton->setEnabled(hasAxes);
    m_getPositionsButton->setEnabled(hasAxes);
    m_removeButton->setEnabled(hasConfig);
    m_saveButton->setEnabled(hasConfig);
    m_goButton->setEnabled(hasConfig);
}

bool ThorlabsPositionManagerDialog::collectSelection(QVector<int>& globalAxisIDs,
    QVector<double>& positionsUm) const
{
    globalAxisIDs.clear();
    positionsUm.clear();

    for (int row = 0; row < m_axes.size(); ++row)
    {
        const QTableWidgetItem* axisItem = m_axisTable->item(row, 0);
        if (!axisItem || axisItem->checkState() != Qt::Checked || row >= m_positionEditors.size())
            continue;

        globalAxisIDs.append(m_axes[row].globalAxisID);
        positionsUm.append(m_positionEditors[row]->value() * 1000.0);
    }

    return !globalAxisIDs.isEmpty();
}

void ThorlabsPositionManagerDialog::newConfig()
{
    if (!m_plugin || m_axes.isEmpty())
        return;

    bool accepted = false;
    const QString name = QInputDialog::getText(this, "New Position Config",
        "Name:", QLineEdit::Normal, QString(), &accepted).trimmed();
    if (!accepted)
        return;
    if (name.isEmpty())
    {
        QMessageBox::warning(this, "New Position Config", "Please enter a name.");
        return;
    }
    if (m_plugin->getPositionConfigNames().contains(name, Qt::CaseInsensitive))
    {
        QMessageBox::warning(this, "New Position Config", "A config with this name already exists.");
        return;
    }

    resetDraft();
    QVector<int> globalAxisIDs;
    QVector<double> positionsUm;
    collectSelection(globalAxisIDs, positionsUm);
    if (!m_plugin->savePositionConfig(name, globalAxisIDs, positionsUm))
    {
        QMessageBox::warning(this, "New Position Config", "The config could not be created.");
        return;
    }

    rebuildConfigList();
}

void ThorlabsPositionManagerDialog::removeConfig()
{
    if (!m_plugin || m_configCombo->currentIndex() <= 0)
        return;

    if (!m_plugin->removePositionConfig(m_configCombo->currentText()))
        QMessageBox::warning(this, "Delete Position Config", "The config could not be deleted.");
    rebuildConfigList();
}

void ThorlabsPositionManagerDialog::getPositions()
{
    if (!m_plugin || m_axes.isEmpty())
        return;

    double* positions = m_plugin->getPosition();
    if (!positions)
    {
        QMessageBox::warning(this, "Get Positions", "The current positions could not be read.");
        return;
    }

    bool allValid = true;
    for (int row = 0; row < m_positionEditors.size(); ++row)
    {
        const double position = positions[row];
        if (std::isfinite(position))
            m_positionEditors[row]->setValue(position / 1000.0);
        else
            allValid = false;
    }

    if (!allValid)
        QMessageBox::warning(this, "Get Positions", "At least one position could not be read.");
}

void ThorlabsPositionManagerDialog::saveConfig()
{
    if (!m_plugin || m_configCombo->currentIndex() <= 0)
        return;

    QVector<int> globalAxisIDs;
    QVector<double> positionsUm;
    if (!collectSelection(globalAxisIDs, positionsUm))
    {
        QMessageBox::warning(this, "Save Position Config", "Please select at least one axis.");
        return;
    }

    if (!m_plugin->savePositionConfig(m_configCombo->currentText(), globalAxisIDs, positionsUm))
        QMessageBox::warning(this, "Save Position Config", "The config could not be saved.");
}

void ThorlabsPositionManagerDialog::go()
{
    if (!m_plugin || m_configCombo->currentIndex() <= 0)
        return;

    m_goButton->setEnabled(false);
    const bool success = m_plugin->goToPositionConfig(m_configCombo->currentText());
    m_goButton->setEnabled(true);
    if (!success)
        QMessageBox::warning(this, "Go To Position Config", "The config could not be reached.");
}
