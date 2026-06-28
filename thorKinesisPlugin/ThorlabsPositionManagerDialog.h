#pragma once

#include "thorlabsKinesisPlugin.h"

#include <QDialog>
#include <QVector>

class QComboBox;
class QDoubleSpinBox;
class QPushButton;
class QTableWidget;

class ThorlabsPositionManagerDialog : public QDialog
{
public:
    explicit ThorlabsPositionManagerDialog(thorlabsKinesisPlugin* plugin, QWidget* parent = nullptr);
    ~ThorlabsPositionManagerDialog() override = default;

    void refresh();

private:
    thorlabsKinesisPlugin* m_plugin = nullptr;
    QVector<thorlabsKinesisPlugin::AxisInfo> m_axes;
    QVector<QDoubleSpinBox*> m_positionEditors;

    QComboBox* m_configCombo = nullptr;
    QTableWidget* m_axisTable = nullptr;
    QPushButton* m_newButton = nullptr;
    QPushButton* m_removeButton = nullptr;
    QPushButton* m_getPositionsButton = nullptr;
    QPushButton* m_saveButton = nullptr;
    QPushButton* m_goButton = nullptr;

    void rebuildAxisTable();
    void rebuildConfigList();
    void resetDraft();
    void loadConfig(int comboIndex);
    void updateButtons();
    bool collectSelection(QVector<int>& globalAxisIDs, QVector<double>& positionsUm) const;
    void newConfig();
    void removeConfig();
    void getPositions();
    void saveConfig();
    void go();
};
