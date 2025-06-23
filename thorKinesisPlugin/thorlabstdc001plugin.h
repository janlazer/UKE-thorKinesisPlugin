#ifndef THORLABSTDC001PLUGIN_H
#define THORLABSTDC001PLUGIN_H

#define CLSID_LOGGER		"{3DA95BB9-9A53-4ED0-B1AA-2D98805C895F}"
#define CLSID_MOTORCONTROL	"{3CE35BF3-1E13-4D2C-8C0B-DEF6314420B3}"	

#include "thorlabstdc001plugin_global.h"
#include "ui_thorlabsTDC001_UI.h"
#include "ui_thorlabsLogger_UI.h"
#include "interfaces.h"
#include <string>
#include <ActiveQt\QAxWidget>

class thorlabsTDC001Plugin : public IStage
{
	Q_OBJECT

	Q_PLUGIN_METADATA(IID IStage_iid)

	//Q_INTERFACES(IStage)
	//Q_INTERFACES(IHardware)

public:
	thorlabsTDC001Plugin();
	~thorlabsTDC001Plugin();


	// hardware interface
	virtual bool detect();
	virtual bool initialize();
	virtual QDockWidget* getView();
	virtual bool release();
	virtual IOutput* getOutput();
	virtual QMap<QString,QString> getSaveValueInformation();
	virtual void setLoadValueInformation( QMap<QString,QString> map);
	virtual void showSettingsWindow();

	// stage specific interface
	bool moveTo(double pos,int id);
	double getPosition();
	bool stop();

	void setHWSerialNr(QString* serialNr);
	void setHWSerialNr(QString serialNr);
	void setStepSize(QString stepSize);
	bool moveSteps(int steps);
	bool moveSteps(int steps, double stepSize);
	void setVelocityParameters(QString minVelocity, QString acceleration, QString maxVelocity);
	void moveTo(QString position);
	void moveTo(double position);
	bool isStopped();

	void initGUI();
	

private:
	Ui::DockWidget ui;
	Ui::DockWidget_logger uiLogger;
	QDockWidget* dock;
	QDockWidget* dockLogger;

	QString		author;
	QString		className;		//for debugging 
	QString		methodName;		//for debugging

	

	QAxWidget* pTDC001;
	QAxWidget* logger;
	
	 


public slots:
	void slot_goHome();
	void slot_openLogger();
	void slot_moveStepForward();
	void slot_moveStepBackward();
	void slot_stopMotor();
	void slot_setStepSize();
	void slot_setMotorSerialNr();
	void slot_setVelocity();
	void slot_moveTo();
	void slot_getPosition();
	void slot_isStopped();


};

#endif // THORLABSTDC001PLUGIN_H

