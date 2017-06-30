#include "WPILib.h"
#include <Joystick.h>
#include <LiveWindow/LiveWindow.h>
#include <RobotDrive.h>
#include <WPILib.h>
#include <Timer.h>
#include <AHRS.h>
#include <CANSpeedController.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <iostream>
#include <iomanip>
#include <unistd.h>

#define POT_MAX_VALUE 40
#define POT_MIN_VALUE 10
#define ANGLE 45
#define WIGGLEROOM .5
#define MXP_IO_VOLTAGE (double)3.3f /* Alternately, 5.0f   */
#define MIN_AN_TRIGGER_VOLTAGE (double)0.76f
#define MAX_AN_TRIGGER_VOLTAGE MXP_IO_VOLTAGE - (double)2.0f
#define AUTO_ONE 100
#define AUTO_TWO 100
#define AUTO_THREE 100
#define AUTO_FOUR 100
#define AUTO_FIVE 100

static const int MAX_NAVX_MXP_DIGIO_PIN_NUMBER = 9;
static const int MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER = 3;
static const int MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER = 1;
static const int NUM_ROBORIO_ONBOARD_DIGIO_PINS = 10;
static const int NUM_ROBORIO_ONBOARD_PWM_PINS = 10;
static const int NUM_ROBORIO_ONBOARD_ANALOGIN_PINS = 4;

enum PinType {
	DigitalIO, PWMs, AnalogIn, AnalogOut
};
int GetChannelFromPin(PinType type, int io_pin_number) {
	int roborio_channel = 0;
	if (io_pin_number < 0) {
		throw std::runtime_error("Error:  navX MXP I/O Pin #");
	}
	switch (type) {
	case DigitalIO:
		if (io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER) {
			throw std::runtime_error(
					"Error:  Invalid navX MXP Digital I/O Pin #");
		}
		roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_DIGIO_PINS
				+ (io_pin_number > 3 ? 4 : 0);
		break;
	case PWMs:
		if (io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER) {
			throw std::runtime_error(
					"Error:  Invalid navX MXP Digital I/O Pin #");
		}
		roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_PWM_PINS;
		break;
	case AnalogIn:
		if (io_pin_number > MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER) {
			throw new std::runtime_error(
					"Error:  Invalid navX MXP Analog Input Pin #");
		}
		roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_ANALOGIN_PINS;
		break;
	case AnalogOut:
		if (io_pin_number > MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER) {
			throw new std::runtime_error(
					"Error:  Invalid navX MXP Analog Output Pin #");
		}
		roborio_channel = io_pin_number;
		break;
	}
	return roborio_channel;
}

//When deploying Code: Turn off Wifi

//If there is an Error With Microsoft Visual Studios: Go to Project->Properties->
//C/C++ General-> Preprocessors Include Paths...->Go the the Providers Tab->
//Hit Apply-> Uncheck CDT Cross GCC Built-in Compiler Settings-> Hit Apply
//Reckeck CDT Cross GCC Built-in Complier Settings-> Hit Apply-> close and build
class Robot: public frc::IterativeRobot {

public:
	Robot() {
		//class Victor *LeftMotors, *RightMotors; //*ConveyorBelt.//
		//Inverts the left side
		//robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		//robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		timer.Start();  //Initializes the timer
		// Define joystick being used at USB port #0 on the Drivers Station
		autocounter = 0;

		//This try catch block checks to see if an error instantiating the navx is thrown
		try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 *
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 *
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
			ahrs = new AHRS(SPI::Port::kMXP);
		} catch (std::exception& ex) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
		if (ahrs) {
			LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
		}

	}

private:

	//x axis=4
	//y axis=-5
	frc::Joystick stick { 0 };         // Only joystick

	AHRS *ahrs;

	frc::RobotDrive robotDrive { 8, 9, 2, 3 };

	Victor *RightMotors =new Victor(GetChannelFromPin(PinType::PWMs, 8));
	Victor *LeftMotors =new Victor(GetChannelFromPin(PinType::PWMs, 9));
	Victor *ConveyorBelt =new Victor(GetChannelFromPin(PinType::PWMs, 2));
	Victor *Horn = new Victor(GetChannelFromPin(PinType::PWMs, 3));

	//Values for determining a deadband for control
	float joystickDeadBandX = 0;
	float joystickDeadBandY = 0;
	float joystickDeadBandZ = 0;
	float X = 1;
	float Y = 0;
	//Variable for horn
	bool hornBlast = false;

	frc::Timer timer;
	frc::LiveWindow* lw = frc::LiveWindow::GetInstance();

	void AutonomousInit() override {
		timer.Reset();
		timer.Start();
	}
	int autocounter;
	void AutonomousPeriodic() override{

	}
	;

	void TeleopInit() override {
	}
	;

	void TeleopPeriodic() override {
		do {
			robotDrive.ArcadeDrive(stick);
		} while(IsOperatorControl() && IsEnabled());

		if(stick.GetRawButton(2)){
			Horn->SetSpeed(0.5);
		}
		X = stick.GetRawAxis(4);
		Y = -stick.GetRawAxis(5);

		//Conveyer Belt
		if(stick.GetRawButton(1)){
			ConveyorBelt->SetSpeed(0.5);
		}
		//if()




		/*//right= left/\ right\/
		if(stick.GetRawAxis(4)>0){
			RightMotors->Set(-.5);
			LeftMotors->Set(.5);
		}
		//left= right/\ left\/
		if(stick.GetRawAxis(4)<0){
			RightMotors->Set(.5);
			LeftMotors->Set(-.5);
	    }
		//forward= right/\ left/\     //
		if(-stick.GetRawAxis(5)>0){
			RightMotors->Set(-.5);
			LeftMotors->Set(-.5);
		}
		//backwards= right\/ left\/   //
		if(-stick.GetRawAxis(5)<0){
			RightMotors->Set(.5);
			LeftMotors->Set(.5);
		}
		 */
		frc::SmartDashboard::PutString("New Code", "This is new code");
		frc::SmartDashboard::PutNumber("X",stick.GetRawAxis(4));
		frc::SmartDashboard::PutNumber("Y",stick.GetRawAxis(5));
		frc::SmartDashboard::PutNumber("Right Motors", RightMotors->Get());
		frc::SmartDashboard::PutNumber("Left Motors", LeftMotors->Get());
	}

	void TestPeriodic() override {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
