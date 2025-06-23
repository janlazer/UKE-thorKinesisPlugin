#include "thorlabstdc001plugin.h"
#include <QtGui>



thorlabsTDC001Plugin::thorlabsTDC001Plugin(){
	this->setName("Thorlabs TDC001 Plugin v0.0.0.1");
	this->author = "MZ";
	this->xmlName = "Thorlabs";
	this->className = "thorlabsTDC001Plugin";
	this->setType(STAGE);
	this->setDetectable(false);		
	this->setDetected(false);
	this->setInitialized(false);
	this->setOutput(true);

	dock = new QDockWidget();
	ui.setupUi(dock);
	dock->setWindowTitle(getName()); 

	dockLogger = new QDockWidget();
	uiLogger.setupUi(dockLogger);
}

thorlabsTDC001Plugin::~thorlabsTDC001Plugin()
{

}


bool thorlabsTDC001Plugin::detect(){
	QString methodName = "detect()";
	
	//qDebug("%s::%s - detect ...", qPrintable(className), qPrintable(methodName));
	//
	//if(!isDetected()){
	//	this->setDetected(true); 
	//	qDebug() << qPrintable(className + "::" + methodName) << "- Detected!";
	//	
	//	//this->setDetected(false);
	//	//qDebug() << qPrintable(className + "::" + methodName) << "- Panic";
	//}else{ 
	//	qDebug("%s::%s - Thorlabs APT was already detected!", qPrintable(className), qPrintable(methodName));
	//}

	this->setDetected(true);
	return this->isDetected();
}


bool thorlabsTDC001Plugin::initialize(){
	
	//Initialize Logger
	logger = uiLogger.axWidget;
	logger->setControl(CLSID_LOGGER);

	// ActiveX MotorControl, defined via QAxWidget & Controller-SN in QtDesigner
	pTDC001 = uiLogger.axWidget_2;
	pTDC001->setControl(CLSID_MOTORCONTROL);


	initGUI();	

	setInitialized(true);
	return this->isInitialized();
}


void thorlabsTDC001Plugin::initGUI(){
	//ui.lineEdit_motorSerialNr->setInputMask("999999");	//6 ASCII digits required. 0-9.
	//ui.lineEdit_motorSerialNr->setCursorPosition (0);


	connect(ui.pushButton_home, SIGNAL(clicked()), this, SLOT(slot_goHome()));
	connect(ui.pushButton_logger, SIGNAL(clicked()), this, SLOT(slot_openLogger()));
	connect(ui.pushButton_stepUp, SIGNAL(clicked()), this, SLOT(slot_moveStepForward()));
	connect(ui.pushButton_stepDown, SIGNAL(clicked()), this, SLOT(slot_moveStepBackward()));
	connect(ui.pushButton_setStepSize, SIGNAL(clicked()), this, SLOT(slot_setStepSize()));
	connect(ui.pushButton_motorSerialNr, SIGNAL(clicked()), this, SLOT(slot_setMotorSerialNr()));
	connect(ui.pushButton_speed, SIGNAL(clicked()), this, SLOT(slot_setVelocity()));
	connect(ui.pushButton_stop, SIGNAL(clicked()), this, SLOT(slot_stopMotor()));
	connect(ui.pushButton_position, SIGNAL(clicked()), this, SLOT(slot_moveTo()));
	connect(ui.pushButton_getPosition, SIGNAL(clicked()), this, SLOT(slot_getPosition()));
	connect(ui.pushButton_isStopped, SIGNAL(clicked()), this, SLOT(slot_isStopped()));

	connect(ui.lineEdit_motorSerialNr, SIGNAL(returnPressed()), ui.pushButton_motorSerialNr, SLOT(click()));
	connect(ui.lineEdit_stepSize, SIGNAL(returnPressed()), ui.pushButton_setStepSize, SLOT(click()));
	connect(ui.lineEdit_speed, SIGNAL(returnPressed()), ui.pushButton_speed, SLOT(click()));
	connect(ui.lineEdit_position, SIGNAL(returnPressed()), ui.pushButton_position, SLOT(click()));
}

void thorlabsTDC001Plugin::setHWSerialNr(QString* serialNr){
	
	bool started = false;
	QVariant var1;
	var1.setValue(started);
	QList<QVariant> listVars;
	listVars.clear();
	listVars.append(var1);
	pTDC001->dynamicCall("GetCtrlStarted(bool&)", listVars);
	started = listVars.at(0).toBool();
	
	if (started){
		pTDC001->dynamicCall("StopCtrl()");
	}

	QString input = "setHWSerialNum(83"+(*serialNr)+")";
	pTDC001->dynamicCall(qPrintable(input));	
	pTDC001->dynamicCall("StartCtrl()");
}

void thorlabsTDC001Plugin::setHWSerialNr(QString serialNr){
	pTDC001->dynamicCall("StopCtrl()");
	QString input = "setHWSerialNum(83"+serialNr+")";
	pTDC001->dynamicCall(qPrintable(input));	
	pTDC001->dynamicCall("StartCtrl()");
}

void thorlabsTDC001Plugin::setStepSize(QString stepSize){
	QString input = "SetJogStepSize(0,"+stepSize+")";
	pTDC001->dynamicCall(qPrintable(input));
}


void thorlabsTDC001Plugin::setVelocityParameters(QString minVelocity, QString acceleration, QString maxVelocity){	
	QString input = "SetVelParams(0,"+minVelocity+","+acceleration+","+maxVelocity+")";
	pTDC001->dynamicCall(qPrintable(input));
}


bool thorlabsTDC001Plugin::moveSteps(int steps){					//if "steps" is negative motor will move backward
	for(int i = 0; i < abs(steps); i++){
		bool standing = false;
		steps > 0 ? pTDC001->dynamicCall("MoveJog(0,1)") : pTDC001->dynamicCall("MoveJog(0,2)");
		do
		{
			Sleep(150);									//probably not ideal for all stepsize/speed combinations
			standing=isStopped();						//this needs a termination condition (in case the stage never stops)
		}
		while(!standing);								//wait for each step to finish
	}
	return true;
}

bool thorlabsTDC001Plugin::moveSteps(int steps, double stepSize){
	QString stepSizeString = QString::number(stepSize);
	setStepSize(stepSizeString);
	return moveSteps(steps);
}

void thorlabsTDC001Plugin::moveTo(QString position){
	int channel = 0;
	int moving = 1;
	QString input = "SetAbsMovePos(0,"+position+")";
	//pTDC001->dynamicCall("MoveAbsolute(int,int)",channel,moving);
	pTDC001->dynamicCall(qPrintable(input));
	pTDC001->dynamicCall("MoveAbsolute(int,int)",channel,moving); //while this function is called the gui won't response (until the position is reached)
}

void thorlabsTDC001Plugin::moveTo(double position){
	QString inputParameter = QString::number(position); 
	moveTo(inputParameter);
}

double thorlabsTDC001Plugin::getPosition(){
	return pTDC001->dynamicCall("GetPosition_Position(0)").toDouble();
}

bool thorlabsTDC001Plugin::isStopped(){
	methodName = "isStopped()";
	int fullStatus = pTDC001->dynamicCall("GetStatusBits_Bits(0)").toInt();
	int status = fullStatus % 100;
	//qDebug() << qPrintable(className + "::" + methodName) << "status: \t " << fullStatus; //this line was needed to determine stop values
	return status==-63 || status==-64 || status==-67 || status==-68 || status==-74 || status==-75 || status==-76 || status==-87 || status==-91 || status==-92 || status ==-88;		//not sure about -88	//these values were experimentally determined
}

QDockWidget* thorlabsTDC001Plugin::getView(){return dock;}

bool thorlabsTDC001Plugin::release(){
	pTDC001->dynamicCall("StopProfiled(0)");	//stop motor
	pTDC001->~QAxWidget();						//Shuts down the ActiveX control and destroys the QAxWidget widget, cleaning up all allocated resources.
	return true;
}



IOutput* thorlabsTDC001Plugin::getOutput(){return NULL;}
QMap<QString,QString> thorlabsTDC001Plugin::getSaveValueInformation(){QMap<QString,QString> map; return map;}
void thorlabsTDC001Plugin::setLoadValueInformation( QMap<QString,QString> map){}



void thorlabsTDC001Plugin::showSettingsWindow(){
	slot_openLogger();
}

bool thorlabsTDC001Plugin::moveTo(double pos,int id){
	double eps = 0.0002;
	bool standing = false;

	moveTo(pos); 

	do
	{
		Sleep(150);					//probably not ideal for all move distances
		standing = isStopped();
	}
	while(!standing);	

	if (abs(pos-getPosition()) < eps){
		return true;
	}else{
		return false;
	}
}

bool thorlabsTDC001Plugin::stop(){
	pTDC001->dynamicCall("StopProfiled(0)");
	Sleep(150);	
	return isStopped();
}














//---------- Slots ----------//
void thorlabsTDC001Plugin::slot_goHome(){
		pTDC001->dynamicCall("MoveHome(0,0)");
}

void thorlabsTDC001Plugin::slot_moveStepForward(){
		pTDC001->dynamicCall("MoveJog(0,1)");
		//moveSteps(1);
}

void thorlabsTDC001Plugin::slot_moveStepBackward(){
		pTDC001->dynamicCall("MoveJog(0,2)");
		//moveSteps(-1);
}

void thorlabsTDC001Plugin::slot_moveTo(){
	if (ui.lineEdit_position->text().size()==0){ui.lineEdit_position->setText("0");}
	QString position = ui.lineEdit_position->text();
	moveTo(position);
}

void thorlabsTDC001Plugin::slot_stopMotor(){
		//pTDC001->dynamicCall("StopImmediate(0)");		//Stops a motor move immediately.
		pTDC001->dynamicCall("StopProfiled(0)");		//Stops  a  motor  move  in  a  profiled  (decelleration) manner.	
}

void thorlabsTDC001Plugin::slot_setStepSize(){
	if (ui.lineEdit_stepSize->text().size()==0){ui.lineEdit_stepSize->setText("1");}
	QString stepSize = ui.lineEdit_stepSize->text();
	setStepSize(stepSize);
}

void thorlabsTDC001Plugin::slot_openLogger(){
	dockLogger->show();
	dockLogger->raise();
}

void thorlabsTDC001Plugin::slot_setMotorSerialNr(){
	if (ui.lineEdit_motorSerialNr->text().size()==0){ui.lineEdit_motorSerialNr->setText("839626");}
	QString serialNr = ui.lineEdit_motorSerialNr->text();
	ui.lineEdit_motorSerialNr->setMaxLength(15);
	ui.lineEdit_motorSerialNr->setText(tr("Connecting..."));
	setHWSerialNr(&serialNr);
	bool commsOK = false;
	QVariant var1;
	var1.setValue(commsOK);
	QList<QVariant> listVars;
	listVars.clear();
	listVars.append(var1);
	pTDC001->dynamicCall("GetHWCommsOK(bool&)", listVars);
	commsOK = listVars.at(0).toBool();
	ui.lineEdit_motorSerialNr->setMaxLength(6);
	ui.lineEdit_motorSerialNr->setText(serialNr);
	
	if(commsOK){
		ui.lineEdit_motorSerialNr->setStyleSheet("QLineEdit{background: #80DE68;}");		//QLineEdit Background green if connection successful
	}else{
		ui.lineEdit_motorSerialNr->setStyleSheet("QLineEdit{background: red;}");			//QLineEdit Background red if connection failed
	}
}


void thorlabsTDC001Plugin::slot_setVelocity(){
	if (ui.lineEdit_speed->text().size()==0){ui.lineEdit_speed->setText("1");}
	QString velocity = ui.lineEdit_speed->text();
	int minVel = velocity.toInt()-(velocity.toInt()/5);
	QString minVelocity = QString::number(minVel); 
	setVelocityParameters(minVelocity, "15", velocity);
}


void thorlabsTDC001Plugin::slot_getPosition(){
	ui.label_getPosition->setText(QString::number(getPosition()));
	
}

void thorlabsTDC001Plugin::slot_isStopped(){
	isStopped()==true ? ui.label_isStopped->setText(tr("True")) : ui.label_isStopped->setText(tr("False"));
}



