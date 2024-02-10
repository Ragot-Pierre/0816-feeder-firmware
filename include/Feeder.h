#ifndef _FEEDER_h
#define _FEEDER_h

#include "arduino.h"
#include "config.h"
#include "shield.h"
// #include <Servo.h>
#include <PCA9685.h>
#include <EEPROMex.h>



class FeederClass {
	protected:

			//on initialize it gets a number. Off feederNo the location EEPROM settings are stored is derived. Nothing else so: TODO: make it obsolete
			int feederNo=-1;


			enum tFeederErrorState {
				sOK=0,
				sOK_NOFEEDBACKLINE=1,
				sERROR_IGNORED=2,
				sERROR=-1,
			} ;
			tFeederErrorState getFeederErrorState();

	public:


	//used to transfer settings between different objects
	struct sFeederSettings {
		uint8_t full_advanced_angle;
		uint8_t half_advanced_angle;
		uint8_t retract_angle;
		uint8_t feed_length;
		int time_to_settle;
		uint16_t advance_angle_speed;					// degree per ms in 1/256 degree resolution, 0 disable
		uint16_t retract_angle_speed;					// degree per ms in 1/256 degree resolution, 0 disable
		int motor_min_pulsewidth;
		int motor_max_pulsewidth;
#ifdef HAS_FEEDBACKLINES  
		uint8_t ignore_feedback;
#endif
		//sFeederState lastFeederState;       //save last position to stay there on poweron? needs something not to wear out the eeprom. until now just go to retract pos.
	};

	uint8_t remainingFeedLength=0;

	//operational status of the feeder
	enum sFeederState {
		sDISABLED,
		sIDLE,
		sSETTLE,
		sMOVING,
	} feederState = sDISABLED;

	//store the position of the advancing lever
	//last state is stored to enable half advance moves (2mm tapes)
	enum sFeederPosition {
		sAT_UNKNOWN,
		sAT_FULL_ADVANCED_POSITION,
		sAT_HALF_ADVANCED_POSITION,
		sAT_RETRACT_POSITION,
		sAT_UNLOAD_POSITION,
	} feederPosition = sAT_UNKNOWN;

	//store last tinestamp position changed to respect a settle time
	unsigned long lastTimePositionChange;
	uint16_t position = FEEDER_DEFAULT_FULL_ADVANCED_ANGLE * 256;			// 1/256 degree
	uint16_t targetPosition = 0;											// 1/256 degree
	bool advanceInProgress = false;
	
	//some variables for utilizing the feedbackline to feed for setup the feeder...
	uint8_t feedbackLineTickCounter=0;
	unsigned long lastTimeFeedbacklineCheck;
	int lastButtonState;

	//permanently in eeprom stored settings
	sFeederSettings feederSettings = {
		FEEDER_DEFAULT_FULL_ADVANCED_ANGLE,
		FEEDER_DEFAULT_HALF_ADVANCED_ANGLE,
		FEEDER_DEFAULT_RETRACT_ANGLE,
		FEEDER_DEFAULT_FEED_LENGTH,
		FEEDER_DEFAULT_TIME_TO_SETTLE,
		FEEDER_DEFAULT_ADVANCE_ANGLE_SPEED,
		FEEDER_DEFAULT_RETRACT_ANGLE_SPEED,
		FEEDER_DEFAULT_MOTOR_MIN_PULSEWIDTH,
		FEEDER_DEFAULT_MOTOR_MAX_PULSEWITH,
#ifdef HAS_FEEDBACKLINES
		FEEDER_DEFAULT_IGNORE_FEEDBACK,
#endif
	};

	PCA9685 *servoController;

	void initialize(uint8_t _feederNo);
	bool isInitialized();
	bool hasFeedbackLine();
	void outputCurrentSettings();
	void setup(PCA9685 *controller);
	sFeederSettings getSettings();
	void setSettings(sFeederSettings UpdatedFeederSettings);
	void loadFeederSettings();
	void saveFeederSettings();
	void factoryReset();

	void gotoPostPickPosition();
	void gotoRetractPosition();
	void gotoHalfAdvancedPosition();
	void gotoFullAdvancedPosition();
	void gotoUnloadPosition();
	void gotoAngle(uint8_t angle);
	bool advance(uint8_t feedLength, bool overrideError);
	void advanceNext();
	void startMove(uint8_t angle, sFeederPosition pos);
	bool moveServoToTarget(uint8_t ms);

	String reportFeederErrorState();
	bool feederIsOk();

	void enable();
	void disable();

	void update();
};

extern FeederClass Feeder;



#endif
