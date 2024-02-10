/*
* Author: mgrl, Dabi
* (c)2023-05-14
*
* This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
* http://creativecommons.org/licenses/by-nc-sa/4.0/
*
* current version: v0.4
*
* CHANGELOG:
* v0.2
*   - added support for sensor shield (experimental, no feedbackline supported)
* v0.3
*   - better checking for manual feed (enhanced reliability not to conflict with g-code issued feeds)
*   - improved setup sequence
*   - default angle for 2mm feeds corrected according to math
* v0.4
*   - New commands: M604, M621, M622, M630
*   - Speed control for advance and retract (for real position tracking)
*   - Unload mode if "0816 Feeder Redesigned" used
*
*/

#define DEBUG

#include "config.h"
#include "shield.h"

// ------------------  I N C  L I B R A R I E S ---------------
#include <HardwareSerial.h>
#include <EEPROMex.h>
#include "Feeder.h"

// ------------------  V A R  S E T U P -----------------------

// ------ Feeder
FeederClass feeders[NUMBER_OF_FEEDER];
enum eFeederEnabledState {
  DISABLED,
  ENABLED,
} feederEnabled=DISABLED;

// ------ Settings-Struct (saved in EEPROM)
struct sCommonSettings {

	//add further settings here

	char version[4];   // This is for detection if settings suit to struct, if not, eeprom is reset to defaults
#ifdef HAS_ANALOG_IN
	float adc_scaling_values[8][2];
#endif
};
sCommonSettings commonSettings_default = {

	//add further settings here

	CONFIG_VERSION,
 
#ifdef HAS_ANALOG_IN
	{
		{ANALOG_A0_SCALING_FACTOR,ANALOG_A0_OFFSET},
		{ANALOG_A1_SCALING_FACTOR,ANALOG_A1_OFFSET},
		{ANALOG_A2_SCALING_FACTOR,ANALOG_A2_OFFSET},
		{ANALOG_A3_SCALING_FACTOR,ANALOG_A3_OFFSET},
		{ANALOG_A4_SCALING_FACTOR,ANALOG_A4_OFFSET},
		{ANALOG_A5_SCALING_FACTOR,ANALOG_A5_OFFSET},
		{ANALOG_A6_SCALING_FACTOR,ANALOG_A6_OFFSET},
		{ANALOG_A7_SCALING_FACTOR,ANALOG_A7_OFFSET},
	},
#endif
};
sCommonSettings commonSettings;

PCA9685 servoController;

// ------ ADC readout
unsigned long lastTimeADCread;
uint16_t adcRawValues[8];
float adcScaledValues[8];

// ------------------  U T I L I T I E S ---------------

// ------ Operate command on all feeder
enum eFeederCommands {
	cmdSetup,
	cmdUpdate,

	cmdEnable,
	cmdDisable,

	cmdOutputCurrentSettings,
	cmdInitializeFeederWithId,
	cmdFactoryReset,

};
void executeCommandOnAllFeeder(eFeederCommands command);
void executeCommandOnAllFeeder(eFeederCommands command) {
	for (uint8_t i=0;i<NUMBER_OF_FEEDER;i++) {
		switch(command) {
			case cmdSetup:
				feeders[i].setup(&servoController);
			break;
			case cmdUpdate:
				feeders[i].update();
			break;
			case cmdEnable:
				feeders[i].enable();
			break;
			case cmdDisable:
				feeders[i].disable();
			break;
			case cmdOutputCurrentSettings:
				feeders[i].outputCurrentSettings();
			break;
			case cmdInitializeFeederWithId:
				feeders[i].initialize(i);
			break;
			case cmdFactoryReset:
				feeders[i].factoryReset();
			break;
			default:
				{}
			break;
		}
	}
}

#ifdef HAS_ANALOG_IN
void updateADCvalues() {

	for(uint8_t i=0; i<=7; i++) {
		adcRawValues[i]=analogRead(i);
		adcScaledValues[i]=(adcRawValues[i]*commonSettings.adc_scaling_values[i][0])+commonSettings.adc_scaling_values[i][1];
	}
}
#endif

void printCommonSettings() {

	//ADC-scaling values
#ifdef HAS_ANALOG_IN
	Serial.println("Analog Scaling Settings:");
	for(uint8_t i=0; i<=7; i++) {
		Serial.print("M");
		Serial.print(MCODE_SET_SCALING);
		Serial.print(" A");
		Serial.print(i);
		Serial.print(" S");
		Serial.print(commonSettings.adc_scaling_values[i][0]);
		Serial.print(" O");
		Serial.print(commonSettings.adc_scaling_values[i][1]);
		Serial.println();
	}
#endif
}


// ----- GCode functions -----

String inputBuffer = "";         // Buffer for incoming G-Code lines


/**
* Look for character /code/ in the inputBuffer and read the float that immediately follows it.
* @return the value found.  If nothing is found, /defaultVal/ is returned.
* @input code the character to look for.
* @input defaultVal the return value if /code/ is not found.
**/
float parseParameter(char code,float defaultVal) {
	int codePosition = inputBuffer.indexOf(code);
	if(codePosition!=-1) {
		//code found in buffer

		//find end of number (separated by " " (space))
		int delimiterPosition = inputBuffer.indexOf(" ",codePosition+1);

		float parsedNumber = inputBuffer.substring(codePosition+1,delimiterPosition).toFloat();

		return parsedNumber;
		} else {
		return defaultVal;
	}

}

uint16_t parseSpeedParameter(char code,uint16_t oldValue) {
	float newValue = parseParameter(code, -1);
	if (newValue<0)
		return oldValue;
	if (newValue==0)
		return 0;
	if (newValue>=256)
		return 65535;
	uint16_t newParam = round(newValue*256);
	if (newParam<1)
		return 1;
	return newParam;
}

void setupGCodeProc() {
	inputBuffer.reserve(MAX_BUFFFER_MCODE_LINE);
}

void sendAnswer(uint8_t error, String message) {
	if(error==0)
	Serial.print(F("ok "));
	else
	Serial.print(F("error "));

	Serial.println(message);
}

bool validFeederNo(int8_t signedFeederNo) {
	if(signedFeederNo<0 || signedFeederNo>(NUMBER_OF_FEEDER-1)) {
		//error, number not in a valid range
		return false;
	}
	//perfectly fine number
	return true;
}

bool validFeederNoError(int8_t signedFeederNo) {
	bool ret = !validFeederNo(signedFeederNo);
	if (ret)
		sendAnswer(1,F("feederNo missing or invalid"));
	return ret;
}

bool checkEnabledFeedersError() {
	if(feederEnabled!=ENABLED) {
		sendAnswer(1,String(String("Enable feeder first! M") + String(MCODE_SET_FEEDER_ENABLE) + String(" S1")));
		return true;
	}
	return false;
}

/**
* Read the input buffer and find any recognized commands.  One G or M command per line.
*/
void processCommand() {

	//get the command, default -1 if no command found
	int cmd = parseParameter('M',-1);

	#ifdef DEBUG
	Serial.print("command found: M");
	Serial.println(cmd);
	#endif


	switch(cmd) {

		/*
		FEEDER-CODES
		*/


		case MCODE_SET_FEEDER_ENABLE: {

			int8_t _feederEnabled=parseParameter('S',-1);
			if( (_feederEnabled==0 || _feederEnabled==1) ) {

				if((uint8_t)_feederEnabled==1) {
					digitalWrite(FEEDER_ENABLE_PIN, HIGH);
					feederEnabled=ENABLED;

					executeCommandOnAllFeeder(cmdEnable);

					sendAnswer(0,F("Feeder set enabled and operational"));
				} else {
					digitalWrite(FEEDER_ENABLE_PIN, LOW);
					feederEnabled=DISABLED;

					executeCommandOnAllFeeder(cmdDisable);

					sendAnswer(0,F("Feeder set disabled"));
				}
			} else if(_feederEnabled==-1) {
				sendAnswer(0,("current powerState: ") + String(feederEnabled));
			} else {
				sendAnswer(1,F("Invalid parameters"));
			}


			break;
		}


		case MCODE_ADVANCE: {
			//1st to check: are feeder enabled?
			if(checkEnabledFeedersError()) {
				break;
			}

			int8_t signedFeederNo = (int)parseParameter('N',-1);
			int8_t overrideErrorRaw = (int)parseParameter('X',-1);
			bool overrideError = false;
			if(overrideErrorRaw >= 1) {
				overrideError = true;
				#ifdef DEBUG
				Serial.println("Argument X1 found, feedbackline/error will be ignored");
				#endif
			}

			//check for presence of a mandatory FeederNo
			if(validFeederNoError(signedFeederNo)) {
				break;
			}

			//determine feedLength
			uint8_t feedLength;
			//get feedLength if given, otherwise go for default configured feed_length
			feedLength = (uint8_t)parseParameter('F',feeders[(uint8_t)signedFeederNo].feederSettings.feed_length);


			if ( ((feedLength%2) != 0) || feedLength>24 ) {
				//advancing is only possible for multiples of 2mm and 24mm max
				sendAnswer(1,F("Invalid feedLength"));
				break;
			}
			#ifdef DEBUG
			Serial.print("Determined feedLength ");
			Serial.print(feedLength);
			Serial.println();
			#endif

			//start feeding
			bool triggerFeedOK=feeders[(uint8_t)signedFeederNo].advance(feedLength,overrideError);
			if(!triggerFeedOK) {
				//report error to host at once, tape was not advanced...
				sendAnswer(1,F("feeder not OK (not activated, no tape or tension of cover tape not OK)"));
			} else {
				//answer OK to host in case there was no error -> NO, no answer now:
				//wait to send OK, until feed process finished. otherwise the pickup is started immediately, thus too early.
				//message is fired off in feeder.cpp
			}

			
			break;
		}

		case MCODE_RETRACT_POST_PICK: {
			//1st to check: are feeder enabled?
			if(checkEnabledFeedersError()) {
				break;
			}


			int8_t signedFeederNo = (int)parseParameter('N',-1);

			//check for presence of a mandatory FeederNo
			if(validFeederNoError(signedFeederNo)) {
				break;
			}

			feeders[(uint8_t)signedFeederNo].gotoPostPickPosition();

			sendAnswer(0,F("feeder postPickRetract done if needed"));

			break;
		}

		case MCODE_FEEDER_IS_OK: {
			int8_t signedFeederNo = (int)parseParameter('N',-1);

			//check for presence of a mandatory FeederNo
			if(validFeederNoError(signedFeederNo)) {
				break;
			}

			sendAnswer(0,feeders[(uint8_t)signedFeederNo].reportFeederErrorState());

			break;
		}

		case MCODE_SERVO_SET_ANGLE: {
			//1st to check: are feeder enabled?
			if(checkEnabledFeedersError()) {
				break;
			}


			int8_t signedFeederNo = (int)parseParameter('N',-1);
			uint8_t angle = (int)parseParameter('A',90);

			//check for presence of a mandatory FeederNo
			if(validFeederNoError(signedFeederNo)) {
				break;
			}
			//check for valid angle
			if( angle>180 ) {
				sendAnswer(1,F("illegal angle"));
				break;
			}

			feeders[(uint8_t)signedFeederNo].gotoAngle(angle);

			sendAnswer(0,F("angle set"));

			break;
		}

		case MCODE_UNLOAD: {
			//1st to check: are feeder enabled?
			if(checkEnabledFeedersError()) {
				break;
			}


			int8_t signedFeederNo = (int)parseParameter('N',-1);

			//check for presence of a mandatory FeederNo
			if(validFeederNoError(signedFeederNo)) {
				break;
			}

			feeders[(uint8_t)signedFeederNo].gotoUnloadPosition();

			sendAnswer(0,F("feeder unload started if needed"));

			break;
		}

		case MCODE_UPDATE_FEEDER_CONFIG:
		case MCODE_UPDATE_ALL_FEEDER_CONFIG: {
			uint8_t feederStart = 0;
			uint8_t feederEnd = NUMBER_OF_FEEDER-1;
			if (cmd == MCODE_UPDATE_FEEDER_CONFIG) {
				int8_t signedFeederNo = (int)parseParameter('N',-1);

				//check for presence of a mandatory FeederNo
				if(validFeederNoError(signedFeederNo)) {
					break;
				}
				feederStart = signedFeederNo;
				feederEnd = signedFeederNo;
			}

			for (uint8_t i=feederStart;i<=feederEnd;i++) {

				//merge given parameters to old settings
				FeederClass::sFeederSettings oldFeederSettings=feeders[i].getSettings();
				FeederClass::sFeederSettings updatedFeederSettings;
				updatedFeederSettings.full_advanced_angle=parseParameter('A',oldFeederSettings.full_advanced_angle);
				updatedFeederSettings.half_advanced_angle=parseParameter('B',oldFeederSettings.half_advanced_angle);
				updatedFeederSettings.retract_angle=parseParameter('C',oldFeederSettings.retract_angle);
				updatedFeederSettings.feed_length=parseParameter('F',oldFeederSettings.feed_length);
				updatedFeederSettings.advance_angle_speed=parseSpeedParameter('S',oldFeederSettings.advance_angle_speed);
				updatedFeederSettings.retract_angle_speed=parseSpeedParameter('R',oldFeederSettings.retract_angle_speed);
				updatedFeederSettings.time_to_settle=parseParameter('U',oldFeederSettings.time_to_settle);
				updatedFeederSettings.motor_min_pulsewidth=parseParameter('V',oldFeederSettings.motor_min_pulsewidth);
				updatedFeederSettings.motor_max_pulsewidth=parseParameter('W',oldFeederSettings.motor_max_pulsewidth);
#ifdef HAS_FEEDBACKLINES
				updatedFeederSettings.ignore_feedback=parseParameter('X',oldFeederSettings.ignore_feedback);
#endif
			
				//set to feeder
				feeders[i].setSettings(updatedFeederSettings);

				//save to eeprom
				feeders[i].saveFeederSettings();

				//reattach servo with new settings
				feeders[i].setup(&servoController);
			}

			//confirm
			sendAnswer(0,F("Feeders config updated."));

			break;
		}

		case MCODE_UPDATE_ALL_FEEDERS_RD: {
			for (uint8_t i=0;i<=NUMBER_OF_FEEDER-1;i++) {
				//merge given parameters to old settings
				FeederClass::sFeederSettings settings=feeders[i].getSettings();
				settings.full_advanced_angle=FEEDER_DEFAULT_RD_FULL_ADVANCED_ANGLE;
				settings.half_advanced_angle=FEEDER_DEFAULT_RD_HALF_ADVANCED_ANGLE;
				settings.retract_angle=FEEDER_DEFAULT_RD_RETRACT_ANGLE;

				//set to feeder
				feeders[i].setSettings(settings);

				//save to eeprom
				feeders[i].saveFeederSettings();

				//reattach servo with new settings
				feeders[i].setup(&servoController);
			}
			//confirm
			sendAnswer(0,F("Feeders config updated."));

			break;
		}

		case MCODE_PRINT_FEEDER_CONFIG: {
			int8_t signedFeederNo = (int)parseParameter('N', -1);

			if (signedFeederNo == -1) {
				for (uint8_t i=0;i<NUMBER_OF_FEEDER;i++) {
					feeders[i].outputCurrentSettings();
				}
			} else if (validFeederNo(signedFeederNo)) {
				feeders[(uint8_t)signedFeederNo].outputCurrentSettings();
			} else {
				sendAnswer(1, F("feederNo invalid"));
			}
			break;
		}

		/*
		CODES to Control ADC
		*/
#ifdef HAS_ANALOG_IN
		case MCODE_GET_ADC_RAW: {
			//answer to host
			int8_t channel=parseParameter('A',-1);
			if( channel>=0 && channel<8 ) {

				//send value in first line of answer, so it can be parsed by OpenPnP correctly
				Serial.println(String("value:")+String(adcRawValues[(uint8_t)channel]));

				//common answer
				sendAnswer(0,"value sent");
			} else {
				sendAnswer(1,F("invalid adc channel (0...7)"));
			}

			break;
		}
		case MCODE_GET_ADC_SCALED: {
			//answer to host
			int8_t channel=parseParameter('A',-1);
			if( channel>=0 && channel<8 ) {

				//send value in first line of answer, so it can be parsed by OpenPnP correctly
				Serial.println(String("value:")+String(adcScaledValues[(uint8_t)channel],4));

				//common answer
				sendAnswer(0,"value sent");
			} else {
				sendAnswer(1,F("invalid adc channel (0...7)"));
			}

			break;
		}
		case MCODE_SET_SCALING: {

			int8_t channel=parseParameter('A',-1);

			//check for valid parameters
			if( channel>=0 && channel<8 ) {
				commonSettings.adc_scaling_values[(uint8_t)channel][0]=parseParameter('S',commonSettings.adc_scaling_values[(uint8_t)channel][0]);
				commonSettings.adc_scaling_values[(uint8_t)channel][1]=parseParameter('O',commonSettings.adc_scaling_values[(uint8_t)channel][1]);

				EEPROM.writeBlock(EEPROM_COMMON_SETTINGS_ADDRESS_OFFSET, commonSettings);

				sendAnswer(0,(F("scaling set and stored to eeprom")));
			} else {
				sendAnswer(1,F("invalid adc channel (0...7)"));
			}


			break;
		}
#endif

#ifdef HAS_POWER_OUTPUTS
		case MCODE_SET_POWER_OUTPUT: {
			//answer to host
			int8_t powerPin=parseParameter('D',-1);
			int8_t powerState=parseParameter('S',-1);
			if( (powerPin>=0 && powerPin<NUMBER_OF_POWER_OUTPUT) && (powerState==0 || powerState==1) ) {
				digitalWrite(pwrOutputPinMap[(uint8_t)powerPin], (uint8_t)powerState);
				sendAnswer(0,F("Output set"));
			} else {
				sendAnswer(1,F("Invalid Parameters"));
			}


			break;
		}
   
#endif

		case MCODE_FACTORY_RESET: {
			commonSettings.version[0]=commonSettings.version[0]+1;

			EEPROM.writeBlock(EEPROM_COMMON_SETTINGS_ADDRESS_OFFSET, commonSettings);

			sendAnswer(0,F("EEPROM invalidated, defaults will be loaded on next restart. Please restart now."));


			break;
		}

		default:
		sendAnswer(0,F("unknown or empty command ignored"));

		break;

	}

}

void listenToSerialStream() {

	while (Serial.available()) {

		// get the received byte, convert to char for adding to buffer
		char receivedChar = (char)Serial.read();

		// print back for debugging
		//#ifdef DEBUG
		Serial.print(receivedChar);
		//#endif

		// add to buffer
		inputBuffer += receivedChar;

		// if the received character is a newline, processCommand
		if (receivedChar == '\n') {

			//remove comments
			inputBuffer.remove(inputBuffer.indexOf(";"));
			inputBuffer.trim();


			processCommand();

			//clear buffer
			inputBuffer="";

		}
	}
}



// ------------------  S E T U P -----------------------
void setup() {
	Serial.begin(SERIAL_BAUD);
	while (!Serial);
	Serial.println(F("Controller starting...")); Serial.flush();
	Serial.println(F("Here is some stuff saved in EEPROM. Paste in a textfile to backup these settings...")); Serial.flush();

  servoController.resetDevices();
  servoController.init(PCA9685_PhaseBalancer_Linear, PCA9685_OutputDriverMode_TotemPole, PCA9685_OutputEnabledMode_Normal, PCA9685_OutputDisabledMode_Low, PCA9685_ChannelUpdateMode_AfterAck);

	//feeder enable output
	pinMode(FEEDER_ENABLE_PIN,OUTPUT);
	digitalWrite(FEEDER_ENABLE_PIN,LOW);

	//power output init
	for(uint8_t i=0;i<NUMBER_OF_POWER_OUTPUT;i++) {
		pinMode(pwrOutputPinMap[i],OUTPUT);
		digitalWrite(pwrOutputPinMap[i],LOW);
	}
	
	// setup listener to serial stream
	setupGCodeProc();

	//initialize active feeders, this is giving them an valid ID
	//needs to be done before factory reset to have a valid ID (eeprom-settings location is derived off the ID)
	executeCommandOnAllFeeder(cmdInitializeFeederWithId);

	//load commonSettings from eeprom
	EEPROM.readBlock(EEPROM_COMMON_SETTINGS_ADDRESS_OFFSET, commonSettings);

	//factory reset on first start or version changing
	if(strcmp(commonSettings.version,CONFIG_VERSION) != 0) {
		Serial.println(F("First start/Config version changed"));

		//reset needed
		executeCommandOnAllFeeder(cmdFactoryReset);

		//update commonSettings in EEPROM to have no factory reset on next start
		EEPROM.writeBlock(EEPROM_COMMON_SETTINGS_ADDRESS_OFFSET, commonSettings_default);
	}

	//print all settings to console
	printCommonSettings();

	//setup feeder objects
  digitalWrite(FEEDER_ENABLE_PIN, HIGH);  //power feeder first, because while setup feeder might retract.
	executeCommandOnAllFeeder(cmdSetup);	//setup everything first, then power on short. made it this way to prevent servos from driving to an undefined angle while being initialized
	delay(1000);		//have the last feeder's servo settled before disabling
  executeCommandOnAllFeeder(cmdDisable); //while setup ran, the feeder were moved and remain in sIDLE-state -> it shall be disabled
	digitalWrite(FEEDER_ENABLE_PIN, LOW);	//disable power afterwards
	
	//print all settings of every feeder to console
	executeCommandOnAllFeeder(cmdOutputCurrentSettings);

	//init adc-values
#ifdef HAS_ANALOG_IN
	updateADCvalues();
	lastTimeADCread=millis();
#endif

	Serial.println(F("Controller up and ready! Have fun."));
}



// ------------------  L O O P -----------------------

void loop() {

	//debouncedButton.update();

	// Process incoming serial data and perform callbacks
	listenToSerialStream();

	// Process servo control
	executeCommandOnAllFeeder(cmdUpdate);

	// Process ADC inputs
#ifdef HAS_ANALOG_IN
	if (millis() - lastTimeADCread >= ADC_READ_EVERY_MS) {
		lastTimeADCread=millis();

		updateADCvalues();
	}
#endif
}
