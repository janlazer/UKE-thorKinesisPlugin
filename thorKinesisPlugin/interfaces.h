/*************************************************************************************
**																					**
**									smartLab										**
**	Copyright 2012 - Laser Zentrum Hannover e.V.  - Biomedical Optics Departement	**
**																					**
**	@author Sebastian Bleeker (BR), Ben Matthias (MB), Jan Hahn (JH)				**
**																					**
**	Class: interfaces.h																**
**																					**
**  This file contains all interafaces which are used in this software. The base	**
**  Class of Hardware Plugins is the IHardware Interface, which will be derived by  **
**  by a hardware specific interface, which is derived by the plugins main class.   **
**																					**
**  The interfaces for the Output Classes are also located in this file. The Main   **
**  Interface is the IOutout interface, from which all output classes have to derive**
**  from.																			**
**																					**
*************************************************************************************/

//!	Interfaces
/*!	This file contains all interfaces which are used in this software. The base
	Class of Hardware Plugins is the IHardware Interface, which will be derived by
	by a hardware specific interface, which is derived by the plugins main class.

	The interfaces for the Output Classes are also located in this file. The Main
	Interface is the IOutout interface, from which all output classes have to derive
	from.
*/
#pragma once

#ifndef INTERFACES_H
#define INTERFACES_H

#ifdef min
#undef min
#endif

#include <QTextEdit>
#include <QDialog>
#include <QDateTime>
#include <QtPlugin>
#include <QMdiSubWindow>
#include <QMap>
#include "qwt_plot_curve.h"

class QDockWidget;

//! IOutput
/*! Base class of the Output windows
	If you want to create a new Output window derive from this class.
	It provides basic methods for creation and window refreshing.
*/
class IOutput
{
public:

	/**
	* This method has to contain the logic for GUI initialization. It will be called automatically after creation.
	**/

	virtual void initGUI() = 0;

	/**
	* This method has to contain the logic for window refreshing and will be called automatically or manually.
	*
	**/

	virtual void updateView() = 0;

	void setMdiSubWindow(QMdiSubWindow* wnd) { this->pWnd = wnd; }
	QMdiSubWindow* getMdiSubWindow() { return this->pWnd; }

protected:
	QMdiSubWindow* pWnd;
};


class IHWNDOutput : public IOutput
{
public:
	virtual HWND getWindowHandle() = 0;

};
//! MuSoCurve
/*! Helper class for realising dynamically multiple Curves
*/

class MuSoCurve
{
public:

	/**
	* Constructor
	*
	* @param name - uniqie identifier for the curve
	**/

	MuSoCurve(QString name) { this->name = name; curve = new QwtPlotCurve(name); }

	/**
	* Returns the QwtPlotCurve
	*
	* @returns a pointer to the QwtPlotCurve
	**/

	QwtPlotCurve* getCurve() { return this->curve; }

	/**
	* Returns the name/identifier
	*
	* @returns the name
	**/

	QString getName() { return this->name; }

	/**
	* Sets the name/identifier
	*
	* @param name - sets the name of the curve
	**/

	void setName(QString name) { this->name = name; this->curve->setTitle(name); }

private:
	QwtPlotCurve* curve; /*!< actual curve */
	QString name; /*!< name/unique identifier */
};

//! IQwtOutput
/*! Output Window preset for displaying functions and graphs with Qwt Framework.
	The Basic class provides a simple QPlot and three QPlotCurves which data can be set by you.
	A Magnifier, a legend and a mouse navigation is alread< implemented
*/
class IQwtOutput : public IOutput
{
public:

	/**
	* Sets the data and scale for the first curve.
	*
	* @param data the data
	* @param scale the scale
	*/

	virtual void setData(QString identifier, QVector<double>* data, QVector<double>* scale) = 0;
	virtual void setData(QString identifer, const double* data, const double* scale, int size) = 0;

	/**
	* adds a curve to the plotter
	*
	* @param identifier - unique name to retrieve and set data
	* @param color - line color. default: blue
	* @param style - curve style. default: line
	**/
	virtual void addCurve(QString identifier, QColor color = QColor(Qt::blue), QwtPlotCurve::CurveStyle style = QwtPlotCurve::Lines) = 0;

	/**
	* removes a curve from the plotter
	*
	* @param identifier - unique name of the curve
	**/
	virtual void removeCurve(QString identifier) = 0;


	/**
	* Update Axis titles
	*
	* @param yAxis - title for y-axis
	* @param xAxis - title for x-axis
	**/

	virtual void setAxisLegend(QString yAxis, QString xAxis) = 0;

	/**
	* Set axis autoscale
	*
	* @param yAxis - autoscale for y-axis
	* @param xAxis - autoscale for x-axis
	**/

	virtual void setAxisAutoScale(bool yAxis, bool xAxis) = 0;

	/**
	* Set ranges of the axes
	*
	* @param yAxisMin - minimum Value of y-axis
	* @param yAxisMax - maximum Value of y-axis
	* @param xAxisMin - minimum Value of x-axis
	* @param xAxisMax - maximum Value of x-axis
	**/

	virtual void setAxesRanges(double yAxisMin, double yAxisMax, double xAxisMin, double xAxisMax) = 0;

	/**
	* Set ranges of the x-axis
	*
	* @param xAxisMin - minimum Value of x-axis
	* @param xAxisMax - maximum Value of x-axis
	**/

	virtual void setAxisXRanges(double xAxisMin, double xAxisMax) = 0;

	/**
	* Set ranges of the y-axis
	*
	* @param yAxisMin - minimum Value of y-axis
	* @param yAxisMax - maximum Value of y-axis
	**/

	virtual void setAxisYRanges(double yAxisMin, double yAxisMax) = 0;

	/**
	* Returns the list with MuSoCurves.
	*
	* @returns the list with MuSoCurves
	**/
	QList<MuSoCurve*> getCurves() { return curves; }

protected:
	/**
	* Returns a specific QwtPlotCurve by its identifier.
	*
	* @param identifier - unique identifier
	*
	* @returns the QwtPlotCurve with the given identifier
	**/

	QwtPlotCurve* getCurveByName(QString identifier) {
		QwtPlotCurve* curve = NULL;
		foreach(MuSoCurve * ms, curves) { if (ms->getName() == identifier) curve = ms->getCurve(); }
		return curve;
	}


	QList<MuSoCurve*> curves; /*!< List with MuSoCurves */

};

class IOGl2DOutput : public IOutput
{

};

//! IOGlOutput
/*!	Output window preset fir displaying fast 2D and 3D image data with the OpenGl framework.
	Is refreshed automatically with approx. 28 frames per second.
*/
class IOGlOutput : public IOutput
{
public:
	//! DataType
	/*! DataType enumeration */

	enum DataType
	{
		CHAR,		/*!< unsigned char data (8bit images) */
		SHORT10,	/*!< unsigned short data (for 10bit camera acquisition) */
		SHORT12,	/*!< unsigned short data (for 12bit camera acquisition) */
		CHAR_RGB	/*!< unsigned char RGB 8 data*/
	};

	/**
	* Sets the data type.
	*
	* @param type DataType used for the output window.
	*/
	virtual void setDataType(DataType type) { this->dataType = type; }

	/**
	* Sets a new unsgined char data array to display image data.
	*
	* @param bScan unsigned char array with 8 bit grayscale values.
	*/
	virtual void setData(unsigned char* bScan) = 0;

	/**
	* Sets a new unsgined short data array to display image data.
	*
	* @param bScan unsigned short array with 16 bit grayscale values. (10 bit or 12 bit from camera)
	*/
	virtual void setData(unsigned short* bScan) = 0;

	/**
	* Sets the width and height of the data array
	*
	* @param w - width of the array
	* @param h - height of the array
	*
	* NOTE: You have to adjust the width and the height, before you pass the data via setData(unsigned char *)
	*/

	virtual void setSize(int w, int h) = 0;

	/**
	* Sets the rotation angles
	*
	* @param xRot - angle in xRot/16.0 degrees around x-axis
	* @param yRot - angle in yyRot16.0 degrees around y-axis
	* @param zRot - angle in zRot degrees around z-axis
	*
	*/

	virtual void setRotation(float xRot, float yRot, float zRot) = 0;

protected:
	DataType dataType;	/*!< dataType data field */
};
//! OutputWindow
/*!	Represents an output window for data visualization.
*/
class OutputWindow : public QObject
{
	Q_OBJECT
public:
	enum OutputType
	{
		OPENGL2D,	/*!< OpenGl 2D */
		OPENGL, /*!< OpenGl */
		QWT, /*!< Qwt */
		SHWNDWINDOW
	};

	/**
	* Constructor
	*
	* @param name - identifier for the window
	* @param type - output type
	**/

	OutputWindow(QString name, OutputType type) { this->name = name; this->type = type; }

	/**
	* Destructor
	**/

	~OutputWindow() {}

	/**
	* Returns the outpu ttype.
	*
	* @returns the output type
	**/

	OutputType getType() { return type; }

	/**
	* Returns the name of the window
	*
	* @returns the name of the window
	**/

	QString getName() { return name; }

	/**
	* Returns a poiter to the actual output Window.
	*
	* @returns a poiter to the actual output Window
	**/

	IOutput* getOutput() { return output; }

	/**
	* Sets the pointer for the acutal output window
	*
	* @param IOutput* pointer to output window
	**/

	void setOutput(IOutput* output) { this->output = output; }

	bool isShownOnStart() { return shownOnStart; }
	void setShownOnStart(bool flag) { this->shownOnStart = flag; }

private:
	bool shownOnStart;
	OutputType type; /*!< output type */
	IOutput* output; /*!< pointer to output window */
	QString name; /*!< identifier */

public slots:
	void slot_visibilityChanged(bool b) { emit visibilityChanged(b); }
signals:
	void visibilityChanged(bool);	/*!< true means shown, false means hidden */
};

//! Output
/*! Manages output windows
*/

class Output
{
public:

	/**
	* Constructor
	**/

	Output() {}

	/**
	* Destructor
	**/

	~Output() {}

	/**
	* Returns a list with all outputwindows
	*
	* @returns QList with pointers to outputwindows
	**/

	QList<OutputWindow*> getOutputWindows() { return list; }

	/**
	* Adds an output window to the list
	*
	* @param name - identifier
	* @param type- output type
	**/

	void addOutputWindow(QString name, OutputWindow::OutputType type) { list.append(new OutputWindow(name, type)); }

	/**
	* Returns the specific IOutput pointer to the given name
	*
	* @param name - identifier
	*
	* NOTE: Remember to use different names to ensure unique acces to the output windows!
	**/

	IOutput* getOutputWindowByName(QString name) {
		foreach(OutputWindow * out, list) { if (out->getName() == name) return out->getOutput(); }
		return NULL;
	}

private:
	QList<OutputWindow*> list; /*!< List with output window pointers */
};

class CPoint3d
{
public:
	/**
	* standard constructor
	**/
	CPoint3d() { m_x = 0.0;	m_y = 0.0; m_z = 0.0; }

	/**
	* destructor
	**/
	~CPoint3d() {}

	/**
	* copy constructor
	*
	* @param pt - point to copy
	**/
	CPoint3d(const CPoint3d& pt) { *this = pt; }

	/**
	* constructor with parameters
	*
	* @param _x - x-coordinate of point
	* @param _y - y-coordinate of point
	* @param _z - z-coordinate of point
	**/
	CPoint3d(float _x, float _y, float _z) { m_x = _x; m_y = _y; m_z = _z; }

	/**
	* Returns a a pointer to CPoint3d
	*
	* @returns CPoint3d* - a point with three coordinates
	**/
	CPoint3d* getPoint3d() { return this; }

	/**
	* Returns the x-coordinate of the point
	*
	* @returns the x-coordinate of the point
	**/
	float getX() { return m_x; }

	/**
	* Returns the y-coordinate of the point
	*
	* @returns the y-coordinate of the point
	**/
	float getY() { return m_y; }

	/**
	* Returns the z-coordinate of the point
	*
	* @returns the z-coordinate of the point
	**/
	float getZ() { return m_z; }

	/**
	* Sets the x-, y- and z-coordinate of the point
	*
	* @param _x - x-coordinate of point
	* @param _y - y-coordinate of point
	* @param _z - z-coordinate of point
	**/
	void setPoint3d(float _x, float _y, float _z) { m_x = _x; m_y = _y; m_z = _z; }

	/**
	* Sets the x-coordinate of the point
	*
	* @param _x - x-coordinate of point
	**/
	void setX(float _x) { m_x = _x; }

	/**
	* Sets the y-coordinate of the point
	*
	* @param _y - y-coordinate of point
	**/
	void setY(float _y) { m_y = _y; }

	/**
	* Sets the z-coordinate of the point
	*
	* @param _z - z-coordinate of point
	**/
	void setZ(float _z) { m_z = _z; }

	/**
	* Returns the euclidian distance from a second point
	*
	* @returns the euclidian distance from a second point
	**/
	float getDistance(CPoint3d pt) {
		return (float)(sqrt(pow((m_x - pt.getX()), 2)
			+ pow((m_y - pt.getY()), 2)
			+ pow((m_z - pt.getZ()), 2)));
	}

private:
	float m_x;/*!< x-coordinate */
	float m_y;/*!< y-coordinate */
	float m_z;/*!< z-coordinate */
};

class GlobalParameter
{
public:
	double Brightness;
	double Contrast;
	double TraverseSteps;
	double ScanWidthX;
	double ScanWidthFactor;
	bool triggered;
};

//! IHardware
/*!	Hardware base interface. Contains all necessary methods, which all Hardware plugins have to provide.
	This includes base methods like naming and typing.
*/

class IHardware :public QObject
{
	Q_OBJECT
public:

	//! DeviceType
	/*! DeviceType enumeration */

	enum DeviceType
	{
		SCANNER, /*!< Scanner */
		CAMERA, /*!< Camera */
		SPECTROMETER, /*!< Spectrometer */
		GALVANOMETER, /*!< Galvanometer */
		STAGE, /*!< Stage */
		SYSTEM, /*!< System */
		DELAYGENERATOR,
		ENERGYSENSOR,
		SHUTTER,
		WAVEFRONT_MODULATOR,
		SERIALLIGHTSOURCE,
		LIQUIDLENS,
		FORCESENSOR,
		NETWORKCOMMUNICATION
	};

	//! OutputType
	/*! OutputType enumeration */

	enum OutputType
	{
		OPENGL2D,	/*!< OpenGl 2D */
		OPENGL, /*!< OpenGl */
		QWT, /*!< Qwt */
		SHWNDWINDOW
	};


	/**
	* Destructor
	**/

	virtual ~IHardware() {}

	/**
	* Detects the the status of the hardware.
	*
	* @returns true if hardware is plugged in, otherwise false
	*
	* NOTE: There are several hardware devices which are not able to be detected. In this case the method can return false though its plugged in and ready to use.
	*		For this case see the variable bIsDetectable and the method isDetectable()
	*
	* @see bIsDetectable
	* @see isDetectable()
	*
	**/

	virtual bool detect() = 0;

	/**
	* Initializes the hardware. After the call of this method the Hardware Plugins has to be ready to use.
	* Make sure you set the bIsInitialized flag to true, after a succesfull initialization.
	*
	* @returns ture if the process was succesful, false otherwise.
	*
	**/

	virtual bool initialize() = 0;

	/**
	* Returns the GUI of the Plugin in due form.
	*
	* @returns a QDockWidget with user control elemts to control the plugin
	*
	**/
	virtual QDockWidget* getView() = 0;

	/**
	* Releases the Plugin and cleans up the memory. This Method is called automatically.
	*
	* @returns true if succesfull, false otherwise
	**/

	virtual bool release() = 0;

	/**
	* Returns the name of the plugin.
	*
	* @returns the name of the plugin
	**/

	QString getName() { return name; }

	/**
	* Sets the name of the Plugin
	*
	* @param QString name - the name of the plugin
	*
	**/

	void setName(QString name) { this->name = name; }

	/**
	* Returns the id of the plugin
	*
	* @returns the id
	*
	**/

	int getId() { return id; }

	/**
	* Sets the id of the plugin
	*
	* @param int id - the id of the plugin
	*
	**/

	void setId(int id) { this->id = id; }

	/**
	* Returns the device typ.
	*
	* @returns the device type
	*
	* @see DeviceType
	*
	**/

	DeviceType getType() { return deviceType; }

	/**
	* Sets the device type of the plugin.
	*
	* @param the device type
	*
	* @see DeviceType
	*
	**/

	void setType(DeviceType deviceType) { this->deviceType = deviceType; }

	/**
	* Returns wheather the plugin is detectable or not.
	*
	* @returns true, if detectable, otherwise false.
	*
	**/

	bool isDetected() { return bIsDetected; }

	/**
	* Sets the flag, wheather the hardware is detectable or not.
	*
	* @param bool detectabily
	*
	**/

	void setDetected(bool bFlag) { bIsDetected = bFlag; }

	/**
	* Returns wheather the plugin is initialized or not
	*
	* @returns true if initialized, false otherwise
	*
	**/

	bool isInitialized() { return bIsInitialized; }

	/**
	* Sets the flag, wheather the plugin is initialized or not
	*
	* @param bool initialization status
	*
	**/

	void setInitialized(bool bFlag) { bIsInitialized = bFlag; }

	/**
	* Returns the has output flag
	*
	* @returns true if the plugin has an ouput, false otherwise
	*
	**/

	bool hasOutput() { return bHasOutput; }

	/**
	* Sets the has ouput flag.
	*
	* @param true, if has output otherwise false
	*
	**/

	void setOutput(bool bFlag) { bHasOutput = bFlag; }

	/**
	* Returns wheather the hardware is detectable or not
	*
	* @returns true if detectable otherwise false
	*
	**/

	bool isDetectable() { return bIsDetectable; }

	/**
	* Sets wheather the hardware is detectable or not.
	*
	* @param bool - true if detectable, false otherwise
	*
	*/

	void setDetectable(bool bFlag) { bIsDetectable = bFlag; }

	/**
	* Returns a pointer to the output window
	*
	* @returns IOutput - the output window
	*
	**/

	virtual IOutput* getOutput() = 0;

	/**
	* Sets the output window
	*
	* @param IOutput - pointer to output window.
	*
	*/

	void setOutput(IOutput* pOut) { this->mdi = pOut; }

	/**
	* Returns the output type
	*
	* @returns the output type
	*
	**/

	OutputType getOutputType() { return outputType; }

	/**
	* Sets the output type.
	*
	* @param OutputType - the output type
	*
	**/

	void setOutputType(OutputType outputType) { this->outputType = outputType; }

	/**
	* Returns a map with relevant data for saving process
	*
	* @returns a map with data for saving process
	*
	**/

	virtual QMap<QString, QString> getSaveValueInformation() = 0;

	/**
	* Sets a map with loaded data.
	*
	* @param a map with loaded data.
	*
	**/

	virtual void setLoadValueInformation(QMap<QString, QString> map) = 0;

	/**
	* Returns a xml valid name for the plugin
	*
	* @returns xml valid name
	*
	**/

	QString getXmlName() { return xmlName; }

	/**
	* Shows a settings Window with preferences for the plugin
	**/

	virtual void showSettingsWindow() = 0;

	GlobalParameter* getGlobalParameter() { return globalParam; }

	Output output() { return pOutput; }

	/**
	* Set device as master device if true,
	* as slave device if false
	* @param master - master(true), slave(false)
	**/
	void setMasterDevice(bool master) { this->bMasterDevice = master; }

	/**
	* Returns if device is master device.
	* @returns state of the variable masterDevice
	**/
	bool isMasterDevice() { return this->bMasterDevice; }


protected:
	GlobalParameter* globalParam;
	Output pOutput;
	OutputType outputType; /*!< outputType data field */
	IOutput* mdi; /*!< Output window */
	QString name; /*!< Plugin name */
	QString xmlName; /*!< xml valid plugin name */
	int id; /*!< unique id */
	DeviceType deviceType; /*!< the device type */
	bool bIsDetected; /*!< flag for detection status */
	bool bIsInitialized; /*!< inititialized flag */
	bool bHasOutput; /*!< output flag */
	bool bIsDetectable; /*!< flag for detectability */
	bool bMasterDevice;	/*!< master device for syncronization */

signals:
	void openOutputWindow(QMdiSubWindow*); /*!< signal to open a Subwindow in main software */
	void update(); /*!< update signal */
	void updateQwtOutput(QString, QVector<qreal>*, QVector<qreal>*);
};

//! IStage
/*! IStage interface for stage implementations
*/

class IStage : public IHardware
{
	Q_OBJECT
public:

	/**
	* Moves a stage to a position absolutely
	*
	* @param pos - absolute position
	*
	* @returns true if succesful, false otherwise
	*
	**/
	virtual bool moveTo(double pos, int id = 0) = 0;

	/**
	* Moves a stage to a position absolutely
	*
	* @param pos - absolute position
	*
	* @returns true if succesful, false otherwise
	*
	**/
	virtual bool moveTo(double* pos, const char* axes) = 0;

	/**
	* Moves to currentPosition + step*stepSize
	*
	* @param step - number of steps
	*
	* @param stepSize - distance to move per step
	*
	* @returns true if succesful, false otherwise
	*
	**/
	virtual bool moveSteps(double steps, int id = 0) = 0;


	/**
	* Returns the current position
	*
	* @returns the current position
	*
	**/
	virtual double* getPosition(const char* axes) = 0;

	/**
	* Returns the current position
	*
	* @returns the current position
	*
	**/
	virtual double* getPosition() = 0;


	/**
	* Stops all movement immediately
	*
	* @returns true if succeeded, false otherwise
	*
	* NOTE: This feature isn't available for all stages
	*
	**/
	virtual bool stop() = 0;

signals:
	void newPositionAvailable(QVector<qreal>* position, int id);
};

//! ISystem
/*! ISystem interface for system implementations
*/

class ISystem : public IHardware
{
	Q_OBJECT
public:
	//! State
	/*! State enumeration */
	enum State {
		READY,				/*!< system is in ready state */
		ACTIVE,				/*!< system is in active state */
		FAILURE,			/*!< system is in failure state */
		UNDEFINED			/*!< system is in undefined state */
	};

	//! ScanShape
	/*! ScanShape enumeration */
	enum ScanShape
	{
		LINE,		/*!< line scan */
		CIRCLE,		/*!< circle scan */
		CROSS,		/*!< cross scan */
		STAR,		/*!< star scan */
		VOLUME,		/*!< volume scan */
		UNDEF		/*!< undefined */
	};

	typedef struct {
		int width;
		int height;
		int bScansPerVolume;
		ISystem::ScanShape scanShape;
	} SystemParams;

	SystemParams* params;

	/**
	* Starts the system
	**/

	virtual void startSystem() = 0;

	/**
	* Stops the system
	**/

	virtual void stopSystem() = 0;

	/**
	* Returns an array of data
	*
	* @returns an unsigned char* array with plugin specific data
	*
	* NOTE: Check the plugins Documentation for more information
	*
	**/
	virtual unsigned char* getData() = 0;

	/**
	* Returns an integer
	*
	* @return an integer value with plugin specific data
	*
	* NOTE: Check the plugins Documentation for further information
	*
	**/

	virtual int getParam() = 0;
	virtual void setParam(double) = 0;

	/**
	* Returns the state of the system.
	* @returns the state of the system
	* @see State
	**/
	State getState() { return state; }

	/**
	* Sets the state of the system.
	* @param the state of the system
	* @see State
	**/
	void setState(State st) { if (getState() != st) { state = st; emit stateChanged(); } }

protected:
	State		state;	/*!< state of the system */
	ScanShape	scanShape;		/*!< the scan shape */

signals:
	/** signal that new raw data was acquired **/
	void dataAcquired(unsigned short*);
	/** signal that state was changed **/
	void stateChanged();
};

//! ICamera
/*! ICamera interface for camera implementations
*/

class ICamera : public IHardware
{
	Q_OBJECT
public:

	//! BitDepth
	/*! BitDepth enumeration */

	enum BitDepth
	{
		BD_8BITS,
		BD_10BITS,
		BD_12BITS,
		BD_16BITS
	};

	/**
	* Starts grabbing frames continuously
	**/
	virtual void startGrabbing() = 0;

	/**
	* Stops grabbing frames continuously
	**/
	virtual void stopGrabbing() = 0;

	/**
	* Snaps a single frame
	**/
	virtual void snapFrame() = 0;

	/**
	* Snaps a single frame and sets its file name to fileName
	*
	* @param fileName is the file name of the captured image
	**/
	virtual void snapFrame(QString fileName) = 0;

	/**
	* Returns an array of data
	*
	* @returns an unsigned short* array with plugin specific data
	*
	* NOTE: for 10-bit, 12-bit (or 16-bit) acquisition
	**/
	virtual unsigned short* getData_ushort() = 0;

	/**
	* Returns an array of data
	*
	* @returns an unsigned char* array with plugin specific data
	*
	* NOTE: for 8-bit acquisition

	**/
	virtual unsigned char* getData_uchar() = 0;

	virtual void setSavePath(QString) = 0;

	/**
	* Returns current exposure time in us
	*
	* @returns current exposure time in us
	*
	**/
	virtual double getExposureTime() = 0;

	/**
	* Returns current exposure time in us
	*
	* @returns current exposure time in us
	*
	**/
	virtual void setExposureTime(double) = 0;


signals:
	void snapTaken(QString);
public slots:
	virtual void changeSize(int, int) = 0;


};

//! ISpectrometer
/*! ISpectrometer interface for spectrometer implementations
*/

class ISpectrometer : public IHardware
{
	Q_OBJECT
public:
	/** BitDepth enumeration **/
	enum BitDepth {
		BD_8BITS = 0,
		BD_10BITS = 1,
		BD_12BITS = 2
	};

	/** TriggerOutput enumeration **/
	enum TriggerOutput {
		INT_TRIGGERED = 0,
		EXT_TRIGGERED = 1
	};

	/** TriggerMode enumeration **/
	enum TriggerMode {
		TTL_PROGRAMMABLE = 0,
		LEVEL_CONTROLLED = 1,

	};

	/** TriggerEvent enumeration **/
	enum TriggerEvent {
		ACTIVE_HIGH = 0,
		ACTIVE_LOW = 1,
		EDGE_RISING = 2,
		EDGE_FALLING = 3
	};

	//! State
	/*! State enumeration */
	enum State {
		READY = 0,		/*!< spectrometer is ready for acquisition */
		ACTIVE = 1,		/*!< spectrometer is in active state */
		FAILURE = 2,		/*!< spectrometer is in failure state */
		UNDEFINED = 3			/*!< spectrometer is in undefined state */
	};

	typedef struct {


		QString cameraModel; //TODO: to int plus cameraName read out from xml
		QString comPort;
		int bitDepth;
		int triggerInput;
		int	triggerOutput;
		int triggerMode;
		int triggerEvent;
		int triggerPort;
		int triggerFrequency;
		int triggerSource;
		int	lineWidth;
		int	lineStart;
		int linesPerFrame; //mz: nur aus legacy gründen. kann gelöscht werden sobald alle plugins/jobs nicht mehr auf linesPerFrame zugreifen, sondern auf frameWidth
		int frameWidth;
		int frameFrequency;
		int gain;
		int offset;
		int integrationTime;
		int lineTime;
		int isLineScanning;
		double dechirpLineCoeffs[4];
		int	lineAcquisitionMode;				/*	0:"Single Line [Max 70KHz]", 1:"Dual Line [Max 140KHz] - Line A First",
													2:"Vertical Binning", 3:"Line Sum - A Delayed",
													4:"Line Sum - B Delayed", 5:"Line Averaging"
													6:"Line Avg - A Delayed", 7:"Line Avg - B Delayed"	*/
		int	horizontalBinning;					/*	0:"Off", 1:"On"	*/
	} SpectrometerParams;

	SpectrometerParams* params;

	/**
	* Prepares the spectrometer for the acquisition
	* usage of GlobalParameter
	**/
	virtual void prepare() = 0;

	/**
	* Starts grabbing frames continuously
	**/
	virtual void startGrabbing() = 0;

	/**
	* Stops grabbing frames continuously
	**/
	virtual void stopGrabbing() = 0;

	/**
	* Snaps a single frame
	**/
	virtual void snapFrame() = 0;

	/**
	* Returns an array of data
	*
	* @returns an unsigned short* array with plugin specific data
	*
	* NOTE: for 10-bit, 12-bit (or 16-bit) acquisition
	**/
	virtual unsigned short* getData_ushort() = 0;

	/**
	* Returns an array of data
	*
	* @returns an unsigned char* array with plugin specific data
	*
	* NOTE: for 8-bit acquisition

	**/
	virtual unsigned char* getData_uchar() = 0;

	/**
	* Accepts current parameters
	**/
	virtual void accept() = 0;

	/**
	* Setter and getter of the spectrometer
	*
	**/

	// TODO: add descriptions or delete all!!!
	//virtual IFramegrabberAttachedCameraInterface* getCamera() = 0; ???
	//virtual void loadCameraConfigFile(QString fileLocation) = 0; ???
	//virtual void setCamera(IFramegrabberAttachedCameraInterface*) = 0; ???
	// TODO: add IFramegrabberAttachedCameraInterface and IFramegrabberInterface to interfaces
	virtual int getBitDepth() = 0;
	virtual int setBitDepth(int bd) = 0;
	virtual int getTriggerOutput() = 0;
	virtual int setTriggerOutput(int to) = 0;
	virtual int getTriggerMode() = 0;
	virtual int setTriggerMode(int tm) = 0;
	virtual int getTriggerEvent() = 0;
	virtual int setTriggerEvent(int te) = 0;
	//virtual int getTriggerFrequency() = 0;
	//virtual int setTriggerFrequency(int tf) = 0;
	virtual int getTriggerPort() = 0;
	virtual int setTriggerPort(int tp) = 0;
	//virtual int getTriggerSource() = 0;
	//virtual int setTriggerSource(int ts) = 0;
	virtual int getLineStart() = 0;
	virtual int setLineStart(int sp) = 0;
	virtual int getLineWidth() = 0;
	virtual int setLineWidth(int lw) = 0;
	virtual int getFrameWidth() = 0;
	virtual int setFrameWidth(int fw) = 0;
	virtual int getFrameFrequency() = 0;
	virtual int setFrameFrequency(int ff) = 0;
	virtual int getParameter(int paramID) = 0;
	virtual int setParameter(int paramID, int value) = 0;

	/**
	* Returns the state of the spectrometer.
	*
	* @returns the state of the spectrometer
	*
	* @see State
	*
	**/
	State getState() { return state; }

	/**
	* Sets the state of the spectrometer.
	*
	* @param the state of the spectrometer
	*
	* @see State
	*
	**/
	void setState(State st) { if (getState() != st) { state = st; emit stateChanged(); } }

protected:
	State state;	/*!< state of the spectrometer */

signals:
	void paramsChanged();
	void stateChanged();
	void frameAcquired(int);
	void frameAcquired(unsigned short*);
	void frameAcquired(unsigned char*);

};

//! IDelayGenerator
/*! IDelayGenerator interface for delay generator implementations
*/

class IDelayGenerator : public IHardware
{
public:
	/**
	* Sets the delay for a given card
	*
	* @param t - flag for channel t
	* @param channelT - delay for channel T
	* @param a - flag for channel A
	* @param channelA - delay for channel A
	* @param B - flag for channel B
	* @param channelB - delay for channel B
	* @param dgNumber - Card identifier, when more then one cards are used
	*
	**/

	virtual void setDelay(bool t, double channelT, bool a, double channelA, bool b, double channelB, bool gate, long dgNumber) = 0;

	/**
	* Returns the number of cards plugged in.
	*
	* @returns the number of cards
	**/

	virtual int getCardCount() = 0;

	/**
	* Sets the flag for Singleshot activation
	*
	* @param flag - singleshot flag
	* @param dgNumber - Card identifier, when more then one cards are used
	*
	**/

	virtual void setSingleShot(bool flag, long dgNumber) = 0;

	/**
	* Sets the Outputlevel
	*
	* @param val - outputlevel in volt
	* @param dgNumber - Card identifier, when more then one cards are used
	*
	**/

	virtual void setOutputLevel(unsigned long val, long dgNumber) = 0;

	/**
	* Sets the Triggelevel
	*
	* @param val - triggerlevel in volt
	* @param dgNumber - Card identifier, when more then one cards are used
	*
	**/

	virtual void setTriggerLevel(double val, long dgNumber) = 0;

	virtual void stopDelay(long dgNumber) = 0;

};

//! IScanner
/*! IScanner interface for scanner implementations
*/

class IScanner : public IHardware
{
	Q_OBJECT
public:

	//! State
	/*! State enumeration */
	enum State
	{
		READY,				/*!< scanner is in ready for marking state */
		ACTIVE,				/*!< scanner is in active state */
		FAILURE,			/*!< scanner is in failure state */
		UNDEFINED			/*!< scanner is in undefined state */
	};


	/**
	* Start job on scanner
	**/
	virtual void startJob() = 0;


	/**
	* Abort active job
	**/
	virtual void abortJob() = 0;


	/**
	* Send data stream to scanner
	* @param name - scan job name
	* @param xData - analog output data for x axis
	* @param yData - analog output data for y axis
	* @param mData - modulation data for digital output
	* @param numOfPts - number of sample points per analog/digital output
	* @param repeat - repetitions of scanjob, -1: until stopped
	* @returns an error code
	**/
	virtual int sendJob(QString name, float* xData, float* yData, unsigned char* mData, int numOfPts, int repeat) = 0;


	/**
	* load job from file and send it to scanner scanner
	* @param fileName - path of file
	**/
	virtual void loadJob(QString fileName) = 0;

	/**
	* save current job to file
	* @param fileName - path of file
	**/
	virtual void saveJob(QString fileName) = 0;

	/**
	* Set scanner externally triggered
	* @param triggered - true or false
	**/
	virtual void setExtTriggered(bool triggered) = 0;


	/**
	* Is scanner externally triggered?
	**/
	virtual bool isExtTriggered() = 0;


	/**
	* Set sample rate (in free run mode)
	* @param sps - samples per second
	**/
	virtual void setSampleRate(float sps) = 0;


	/**
	* Get sample rate (in free run mode) in samples per second
	**/
	virtual float getSampleRate() = 0;


	/**
	* Returns the state of the scanner.
	* @returns the state of the scanner
	* @see State
	**/
	State getState() { return state; }

	/**
	* Sets the state of the scanner.
	* @param the state of the scanner
	* @see State
	**/
	void setState(State st) { if (getState() != st) { this->state = st; emit stateChanged(); } }

protected:
	State state;	/*!< state of the scanner */

signals:
	void stateChanged();
	void scanAborted();
};

//! IEnergySensor
/*! IEnergySensor interface for energy sensor implementation
*/

class IEnergySensor : public IHardware
{
	Q_OBJECT
public:
	/**
	* Starts measurement and ends it as soon as "int measurements" (or more) values were measured
	*
	* @param deviceNr		number of usb interface
	* @param channel		every energy sensor head has its own channel. (i.e. channel defines the sensor you want to use) If you want to use both sensors simultaneously, use for "channel" any value greater than 1.
	* @param measurements	number of values to be recorded. Measurement stops automatically. If you want to stop measurement manually set measurements = 0
	*
	**/
	virtual void startConditionedMeasurement(int deviceNr, int channel, int measurements) = 0;


	/**
	* Returns pointer to QVector with measured data
	*
	* @param deviceNr	number of usb interface
	* @param channel	every energy sensor head has its own channel. (i.e. channel defines the sensor you want to interact with)
	*
	**/
	virtual QVector<qreal>* getDataPointer(int deviceNr, int channel) = 0;


	/**
	* Returns pointer to QVector with time values that matches to measured data values
	*
	* @param deviceNr	number of usb interface
	* @param channel	every energy sensor head has its own channel. (i.e. channel defines the sensor you want to interact with)
	*
	**/
	virtual QVector<qreal>* getTimePointer(int deviceNr, int channel) = 0;


	/**
	* Returns pointer to QVector with status values that matches to measured data values
	*
	* @param deviceNr	number of usb interface
	* @param channel	every energy sensor head has its own channel. (i.e. channel defines the sensor you want to interact with)
	*
	**/
	virtual QVector<qreal>* getStatusPointer(int deviceNr, int channel) = 0;


	/**
	* Returns true while measuring, false otherwise
	*
	* @param deviceNr	number of usb interface
	* @param channel	every energy sensor head has its own channel. (i.e. channel defines the sensor you want to interact with)
	*
	**/
	virtual bool isMeasuring(int deviceNr, int channel) = 0;


	/**
	* Stops current measurement
	*
	**/
	virtual void stopCurrentMeasurement() = 0;

	/**
	* Gets range  (0: 20uJ -- 1: 2uJ -- 2: 200nJ -- 3: 20nJ)
	*
	* @param deviceNr	number of usb interface
	* @param channel	every energy sensor head has its own channel. (i.e. channel defines the sensor you want to interact with)
	*
	**/
	virtual int getRange(int deviceNr, int channel) = 0;


	/**
	* Sets range
	*
	* @param deviceNr	number of usb interface
	* @param channel	every energy sensor head has its own channel. (i.e. channel defines the sensor you want to interact with)
	* @param range		0: 20uJ -- 1: 2uJ -- 2: 200nJ -- 3: 20nJ
	*
	**/
	virtual void setRange(int deviceNr, int channel, int rangeIndex) = 0;


	/**
	* Gets the measurement condition. The condition determines when and if the measurement will stop automatically. The condition value is the number of measurement values that have to be recorded before the measurement stops automatically.
	*
	**/
	virtual int getCondition() = 0;


	/**
	* Sets the measurement condition. The condition determines when and if the measurement will stop automatically. The condition value is the number of measurement values that have to be recorded before the measurement stops automatically.
	*
	* @param condition	if value of condition is 0, measurement will not stop automatically.
	*
	**/
	virtual void setCondition(int condition) = 0;


signals:
	void measurementFinished();

};

//! IShutterController
/*! IShutterController interface for shutter controller implementation
*/
class IShutterController : public IHardware
{
	Q_OBJECT
public:
	/**
	* Returns true if shutter is open, otherwise false
	*
	**/
	virtual bool isOpen() = 0;

	/**
	* Opens shutter and returns true if shutter is opened, otherwise false
	*
	**/
	virtual bool open() = 0;

	/**
	* Closes shutter and returns true if shutter is closed, otherwise false
	*
	**/
	virtual bool close() = 0;

};

//! IWavefrontModulator
/*! IWavefrontModulator interface for wavefront modulator (e.g. deformable mirror) implementation
*/
class IWavefrontModulator : public IHardware
{
	Q_OBJECT
public:
	typedef struct {

	} WavefrontModulatorParams;

	WavefrontModulatorParams* params;

	/**
	* Set wavefront modulation command
	*
	* @param data	command data (e.g. actuator voltages of a deformable mirror)
	*
	**/
	virtual void setCommand(QVector<qreal>* data) = 0;


	/**
	* Get current set wavefront modulation command
	*
	* \attention	last set command may not be the currently applied command.
	*
	**/
	virtual QVector<qreal> getCommand() = 0;


	/**
	* Get zero wavefront modulator state. (e.g. flat command for deformable mirror)
	*
	*  @returns wavefront modulator state which has no modulation effects
	*
	**/
	virtual QVector<qreal> getZeroCommand() = 0;


	/**
	* Apply wavefront modulation command that was already set
	*
	**/
	virtual bool applySetCommand() = 0;

	/**
	* Apply wavefront modulation command
	*
	* @param data	command data (e.g. Actuator voltages of a deformable mirror)
	*
	**/
	virtual bool applyCommand(QVector<qreal>* data) = 0;


public slots:


signals:
	void applied();
};

//! ISerialLightsource
/*! ISerialLightsource interface for lightsource (SLD) via serial port implementation
*/

class ISerialLightsource : public IHardware
{
	Q_OBJECT
public:

	enum SerialBaudrate
	{
		BAUDRATE110 = 110,
		BAUDRATE300 = 300,
		BAUDRATE600 = 600,
		BAUDRATE1200 = 1200,
		BAUDRATE2400 = 2400,
		BAUDRATE4800 = 4800,
		BAUDRATE9600 = 9600,
		BAUDRATE19200 = 19200,
		BAUDRATE38400 = 38400,
		BAUDRATE57600 = 57600,
		BAUDRATE115200 = 115200
	};

	/**
	* Check all detected serial ports and deliver list pointer with vectors of strings of:
	* port name, friendly name, physical name and enumartor name
	*
	**/
	virtual QList<QVector<QString>>* checkAllSerialPort() = 0;

	/**
	* Check SLD status and set it to the defined status: SLD off in low power mode
	*
	* @param portname	name of serial port of SLD from the detected ones (i.e. COM4)
	*
	**/
	virtual void initSLD(QString portName) = 0;
	virtual void deinitSLD() = 0;

	/**
	* Switch SLD on or off
	*
	**/
	virtual void setOnSLD() = 0;
	virtual void setOffSLD() = 0;

	/**
	* Switch to high or low power mode
	*
	**/
	virtual void setToHighPowerMode() = 0;
	virtual void setToLowPowerMode() = 0;

	/**
	* Establish port connection for communication with SLD via serial port (QextserialPort
	*
	**/
	virtual bool openSerialPortConnection(QString portname, SerialBaudrate sb) = 0;

	/**
	* Gets SLD status pointer of bitarray with actual configuration and failure/error bit
	*
	**/
	virtual int getSLDStatusCode() = 0;

protected:
	SerialBaudrate		serialBaudrate;

signals:
	void signalReadBuffer();
	void signalSLDFailure();

};


//! ILiquidLens
/*! ILiquidLens interface for tunable liquid lens implementation
*/
class ILiquidLens : public IHardware
{
	Q_OBJECT
public:
	/**
	* Set liquid lens to relative position between 0..1
	**/
	virtual void setPosition(double pos) = 0;

	/**
	* Get liquid lens relative position between 0..1
	**/
	virtual double getPosition() = 0;

	/**
	* Set liquid lens externally controlled by e.g. analog signal
	**/
	virtual void setExternallyControlled(bool) = 0;

	/**
	* Checks if the liquid lens is externally controlled
	**/
	virtual bool isExternallyControlled() = 0;
};

//! IForceSensor
/*! IForceSensor interface for force sensor implementation
*/
class IForceSensor : public IHardware
{
	Q_OBJECT
public:
	/**
	* Add new sensor with index
	**/
	virtual void addSensor(int id) = 0;

	/**
	* Add new sensor with index
	**/
	virtual void removeSensor(int id) = 0;

	/**
	* Get force value for specific sensors in mN
	**/
	virtual double getForce(int id) = 0;

	/**
	* Get force values for all added sensors in mN
	**/
	virtual double* getForce() = 0;

	/**
	* Get force values for all added sensors in mN
	**/
	virtual bool start() = 0;

	/**
	* Get force values for all added sensors in mN
	**/
	virtual bool stop() = 0;

signals:
	void newDataAvailable(QVector<qreal>* data, QVector<qreal>* scale, int id);
};

class INetworkCommunication : public IHardware
{
	Q_OBJECT
public:
	/**
	* something
	**/
	//virtual void start() = 0;
public slots:
	virtual bool isConnectionEstablished() = 0;
	virtual void sendMessage(QString msg) = 0;

signals:
	void showMessage(QString msg);
};

#define INetworkCommunication_iid "interfaces.lzh.bo.INetworkCommunication/1.0"
#define IForceSensor_iid "interfaces.lzh.bo.IForceSensor/1.0"
#define ILiquidLens_iid "interfaces.lzh.bo.ILiquidLens/1.0"
#define ISerialLightsource_iid "interfaces.lzh.bo.ISerialLightsource/1.0"
#define IWaveFrontModulator_iid "interfaces.lzh.bo.IWavefrontModulator/1.0"
#define IShutterController_iid "interfaces.lzh.bo.IShutterController/1.0"
#define IEnergySensor_iid "interfaces.lzh.bo.IEnergySensor/1.0"
#define IScanner_iid "interfaces.lzh.bo.IScanner/1.0"
#define IDelayGenerator_iid "interfaces.lzh.bo.IDelayGenerator/1.0"
#define ISpectrometer_iid "interfaces.lzh.bo.ISpectrometer/1.0"
#define ICamera_iid	"interfaces.lzh.bo.ICamera/1.0"
#define IHardware_iid "interfaces.lzh.bo.IHardware/1.0"
#define IStage_iid "interfaces.lzh.bo.IStage/1.0"
#define ISystem_iid	"interfaces.lzh.bo.ISystem/1.0"
#define IOutput_iid	"interfaces.lzh.bo.IOutput/1.0"
#define IQwtOutput_iid "interfaces.lzh.bo.IQwtOutput/1.0"
#define IOGlOutput_iid "interfaces.lzh.bo.IOGlOutput/1.0"
#define IOGl2DOutput_iid "interfaces.lzh.bo.IOGl2DOutput/1.0"

Q_DECLARE_INTERFACE(INetworkCommunication, INetworkCommunication_iid)
Q_DECLARE_INTERFACE(IForceSensor, IForceSensor_iid)
Q_DECLARE_INTERFACE(ILiquidLens, ILiquidLens_iid)
Q_DECLARE_INTERFACE(ISerialLightsource, ISerialLightsource_iid)
Q_DECLARE_INTERFACE(IWavefrontModulator, IWaveFrontModulator_iid)
Q_DECLARE_INTERFACE(IShutterController, IShutterController_iid)
Q_DECLARE_INTERFACE(IEnergySensor, IEnergySensor_iid)
Q_DECLARE_INTERFACE(IScanner, IScanner_iid)
Q_DECLARE_INTERFACE(IDelayGenerator, IDelayGenerator_iid)
Q_DECLARE_INTERFACE(ISpectrometer, ISpectrometer_iid)
Q_DECLARE_INTERFACE(ICamera, ICamera_iid)
Q_DECLARE_INTERFACE(IHardware, IHardware_iid)
Q_DECLARE_INTERFACE(IStage, IStage_iid)
Q_DECLARE_INTERFACE(ISystem, ISystem_iid)
Q_DECLARE_INTERFACE(IOutput, IOutput_iid)
Q_DECLARE_INTERFACE(IQwtOutput, IQwtOutput_iid)
Q_DECLARE_INTERFACE(IOGlOutput, IOGlOutput_iid)
Q_DECLARE_INTERFACE(IOGl2DOutput, IOGl2DOutput_iid)

#endif