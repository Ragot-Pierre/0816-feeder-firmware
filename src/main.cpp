/*
* Author: mgrl, Dabi, P.Ragot
* (c)2023-05-14
*
* This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
* http://creativecommons.org/licenses/by-nc-sa/4.0/
*
* current version: v0.5
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
* v0.5
*   - Switched from direct servo control to I²C PCA9685 PWM controller
*   - Cleaned up a bit by removing unused code
*
*/

#include "config.h"
#include "shield.h"

// ------------------  I N C  L I B R A R I E S ---------------
#include <HardwareSerial.h>
#include <EEPROMex.h>
#include "Feeder.h"

// ------------------  V A R  S E T U P -----------------------

// ------ Feeders
FeederClass feeders[NUMBER_OF_FEEDER];

enum eFeederEnabledState
{
  DISABLED,
  ENABLED,
} feederEnabled;



// ------ Settings-Struct (saved in EEPROM)
struct sCommonSettings {

	//add further settings here

	char version[4];   // This is for detection if settings suit to struct, if not, eeprom is reset to defaults
};

sCommonSettings commonSettings_default = {

	//add further settings here

	CONFIG_VERSION,
};

sCommonSettings commonSettings;



// ------ I²C controllers
PCA9685 servoControllers[NUMBER_OF_CONTROLLERS];



// ------------------  U T I L I T I E S ---------------

// ------ Operate command on all feeder
enum eFeederCommands
{
	cmdSetup,
	cmdUpdate,

	cmdEnable,
	cmdDisable,

	cmdOutputCurrentSettings,
	cmdInitializeFeederWithId,
	cmdFactoryReset,
};

// void executeCommandOnAllFeeder(eFeederCommands command);
void executeCommandOnAllFeeder(eFeederCommands command)
{
	for (uint16_t i = 0; i < NUMBER_OF_FEEDER; i++)
	{
		switch(command)
		{
			case cmdSetup:
				feeders[i].setup(servoControllers);
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

void printCommonSettings() {}


// ----- GCode functions -----

String inputBuffer = "";         // Buffer for incoming G-Code lines


/**
* Look for character /code/ in the inputBuffer and read the float that immediately follows it.
* @return the value found.  If nothing is found, /defaultVal/ is returned.
* @input code the character to look for.
* @input defaultVal the return value if /code/ is not found.
**/
float parseParameter(char code,float defaultVal)
{
	int codePosition = inputBuffer.indexOf(code);

	if(codePosition != -1) {
		//code found in buffer

		//find end of number (separated by " " (space))
		int delimiterPosition = inputBuffer.indexOf(" ", codePosition+1);

		float parsedNumber = inputBuffer.substring(codePosition + 1, delimiterPosition).toFloat();

		return parsedNumber;
	}
	else
	{
		return defaultVal;
	}
}

uint16_t parseSpeedParameter(char code,uint16_t oldValue)
{
	float newValue = parseParameter(code, -1);

	if (newValue < 0)
		return oldValue;
	if (newValue == 0)
		return 0;
	if (newValue >= 256)
		return 65535;

	uint16_t newParam = round(newValue * 256);

	if (newParam < 1)
		return 1;

	return newParam;
}

void setupGCodeProc()
{
	inputBuffer.reserve(MAX_BUFFFER_MCODE_LINE);
}

void sendAnswer(uint8_t error, String message)
{
	if(error==0)
		Serial.print(F("ok "));
	else
		Serial.print(F("error "));

	Serial.println(message);
}

bool validFeederNo(int16_t signedFeederNo)
{
	if(signedFeederNo < 0 || signedFeederNo > (NUMBER_OF_FEEDER - 1))
	{
		//error, number not in a valid range
		return false;
	}
	//perfectly fine number
	return true;
}

bool validFeederNoError(int16_t signedFeederNo)
{
	bool ret = !validFeederNo(signedFeederNo);
	if (ret)
		sendAnswer(1, F("feederNo missing or invalid"));
	return ret;
}

bool checkEnabledFeedersError()
{
	if(feederEnabled!=ENABLED)
	{
		sendAnswer(1, String(String("Enable feeder first! M") + String(MCODE_SET_FEEDER_ENABLE) + String(" S1")));
		return true;
	}
	return false;
}

/**
* Read the input buffer and find any recognized commands.  One G or M command per line.
*/
void processCommand()
{
	//get the command, default -1 if no command found
	int cmd = parseParameter('M', -1);

	#ifdef DEBUG
	Serial.print("command found: M");
	Serial.println(cmd);
	#endif

	switch(cmd)
	{
		/*
		FEEDER-CODES
		*/

		case MCODE_SET_FEEDER_ENABLE:
		{
			int8_t _feederEnabled = parseParameter('S', -1);

			if((_feederEnabled == 0 || _feederEnabled == 1))
			{
				if((uint8_t)_feederEnabled == 1)
				{
					// for (uint8_t i = 0; i < NUMBER_OF_CONTROLLERS; i++)
					// {
					// 	for (uint8_t j = 0; j < 16; j++)
					// 	{
					// 		servoControllers[i].setChannelOn(j);
					// 		servoControllers[i].setChannelPWM(j, 310);
					// 	}						
					// }
					
					feederEnabled = ENABLED;

					executeCommandOnAllFeeder(cmdEnable);

					sendAnswer(0, F("Feeder set enabled and operational"));
				}
				else
				{
					// for (uint8_t i = 0; i < NUMBER_OF_CONTROLLERS; i++)
					// {
					// 	for (uint8_t j = 0; j < 16; j++)
					// 	{
					// 		servoControllers[i].setChannelOff(j);
					// 	}						
					// }

					feederEnabled = DISABLED;

					executeCommandOnAllFeeder(cmdDisable);

					sendAnswer(0, F("Feeder set disabled"));
				}

			}
			else if(_feederEnabled == -1)
			{
				sendAnswer(0, ("current powerState: ") + String(feederEnabled));
			}
			else
			{
				sendAnswer(1, F("Invalid parameters"));
			}

			break;
		}

		case MCODE_ADVANCE:
		{
			//1st to check: are feeder enabled?
			if(checkEnabledFeedersError()) { break; }

			int16_t signedFeederNo = (int)parseParameter('N', -1);
			int8_t overrideErrorRaw = (int)parseParameter('X', -1);

			bool overrideError = false;

			if(overrideErrorRaw >= 1)
			{
				overrideError = true;
				#ifdef DEBUG
				Serial.println("Argument X1 found, feedbackline/error will be ignored");
				#endif
			}

			//check for presence of a mandatory FeederNo
			if(validFeederNoError(signedFeederNo)) { break; }

			//determine feedLength
			uint8_t feedLength;
			//get feedLength if given, otherwise go for default configured feed_length
			feedLength = (uint8_t)parseParameter('F', feeders[(uint16_t)signedFeederNo].feederSettings.feed_length);


			if ( ((feedLength%2) != 0) || feedLength > 24 )
			{
				//advancing is only possible for multiples of 2mm and 24mm max
				sendAnswer(1, F("Invalid feedLength"));
				break;
			}

			#ifdef DEBUG
			Serial.print("Determined feedLength ");
			Serial.print(feedLength);
			Serial.println();
			#endif

			//start feeding
			bool triggerFeedOK = feeders[(uint16_t)signedFeederNo].advance(feedLength, overrideError);
			if(!triggerFeedOK)
			{
				//report error to host at once, tape was not advanced...
				sendAnswer(1,F("feeder not OK (not activated, no tape or tension of cover tape not OK)"));
			}
			else
			{
				//answer OK to host in case there was no error -> NO, no answer now:
				//wait to send OK, until feed process finished. otherwise the pickup is started immediately, thus too early.
				//message is fired off in feeder.cpp
			}
			
			break;
		}

		case MCODE_RETRACT_POST_PICK:
		{
			//1st to check: are feeder enabled?
			if(checkEnabledFeedersError()) { break; }

			int16_t signedFeederNo = (int)parseParameter('N', -1);

			//check for presence of a mandatory FeederNo
			if(validFeederNoError(signedFeederNo)) { break; }

			feeders[(uint16_t)signedFeederNo].gotoPostPickPosition();

			sendAnswer(0, F("feeder postPickRetract done if needed"));

			break;
		}

		case MCODE_FEEDER_IS_OK:
		{
			int16_t signedFeederNo = (int)parseParameter('N', -1);

			//check for presence of a mandatory FeederNo
			if(validFeederNoError(signedFeederNo)) { break; }

			sendAnswer(0, feeders[(uint16_t)signedFeederNo].reportFeederErrorState());

			break;
		}

		case MCODE_SERVO_SET_ANGLE:
		{
			//1st to check: are feeder enabled?
			if(checkEnabledFeedersError()) { break; }

			int16_t signedFeederNo = (int)parseParameter('N', -1);
			uint8_t angle = (int)parseParameter('A', 90);

			//check for presence of a mandatory FeederNo
			if(validFeederNoError(signedFeederNo)) { break; }

			//check for valid angle
			if( angle > 180 )
			{
				sendAnswer(1, F("illegal angle"));
				break;
			}

			feeders[(uint16_t)signedFeederNo].gotoAngle(angle);

			sendAnswer(0, F("angle set"));

			break;
		}

		case MCODE_UNLOAD:
		{
			//1st to check: are feeder enabled?
			if(checkEnabledFeedersError()) { break; }

			int16_t signedFeederNo = (int)parseParameter('N',-1);

			//check for presence of a mandatory FeederNo
			if(validFeederNoError(signedFeederNo)) {
				break;
			}

			feeders[(uint16_t)signedFeederNo].gotoUnloadPosition();

			sendAnswer(0,F("feeder unload started if needed"));

			break;
		}

		case MCODE_UPDATE_FEEDER_CONFIG:
			case MCODE_UPDATE_ALL_FEEDER_CONFIG:
			{
				uint8_t feederStart = 0;
				uint16_t feederEnd = NUMBER_OF_FEEDER - 1;

				if (cmd == MCODE_UPDATE_FEEDER_CONFIG)
				{
					int16_t signedFeederNo = (int)parseParameter('N', -1);

					//check for presence of a mandatory FeederNo
					if(validFeederNoError(signedFeederNo)) { break; }

					feederStart = signedFeederNo;
					feederEnd = signedFeederNo;
				}

				for (uint16_t i=feederStart;i<=feederEnd;i++)
				{
					//merge given parameters to old settings
					FeederClass::sFeederSettings oldFeederSettings = feeders[i].getSettings();
					FeederClass::sFeederSettings updatedFeederSettings;

					updatedFeederSettings.full_advanced_angle = parseParameter('A', oldFeederSettings.full_advanced_angle);
					updatedFeederSettings.half_advanced_angle = parseParameter('B', oldFeederSettings.half_advanced_angle);
					updatedFeederSettings.retract_angle = parseParameter('C', oldFeederSettings.retract_angle);
					updatedFeederSettings.feed_length = parseParameter('F', oldFeederSettings.feed_length);
					updatedFeederSettings.advance_angle_speed = parseSpeedParameter('S', oldFeederSettings.advance_angle_speed);
					updatedFeederSettings.retract_angle_speed = parseSpeedParameter('R', oldFeederSettings.retract_angle_speed);
					updatedFeederSettings.time_to_settle = parseParameter('U', oldFeederSettings.time_to_settle);
					updatedFeederSettings.motor_min_pulsewidth = parseParameter('V', oldFeederSettings.motor_min_pulsewidth);
					updatedFeederSettings.motor_max_pulsewidth = parseParameter('W', oldFeederSettings.motor_max_pulsewidth);
				
					//set to feeder
					feeders[i].setSettings(updatedFeederSettings);

					//save to eeprom
					feeders[i].saveFeederSettings();

					//reattach servo with new settings
					feeders[i].setup(servoControllers);
				}

				//confirm
				sendAnswer(0, F("Feeders config updated."));

				break;
			}

			case MCODE_UPDATE_ALL_FEEDERS_RD:
			{
				for (uint16_t i = 0; i <= NUMBER_OF_FEEDER - 1; i++)
				{
					//merge given parameters to old settings
					FeederClass::sFeederSettings settings = feeders[i].getSettings();
					settings.full_advanced_angle = FEEDER_DEFAULT_RD_FULL_ADVANCED_ANGLE;
					settings.half_advanced_angle = FEEDER_DEFAULT_RD_HALF_ADVANCED_ANGLE;
					settings.retract_angle = FEEDER_DEFAULT_RD_RETRACT_ANGLE;

					//set to feeder
					feeders[i].setSettings(settings);

					//save to eeprom
					feeders[i].saveFeederSettings();

					//reattach servo with new settings
					feeders[i].setup(servoControllers);
				}

				//confirm
				sendAnswer(0, F("Feeders config updated."));

				break;
			}

		case MCODE_PRINT_FEEDER_CONFIG:
		{
			int16_t signedFeederNo = (int)parseParameter('N', -1);

			if (signedFeederNo == -1)
			{
				for (uint16_t i = 0; i < NUMBER_OF_FEEDER; i++)
				{
					feeders[i].outputCurrentSettings();
				}
			}
			else if (validFeederNo(signedFeederNo))
			{
				feeders[(uint16_t)signedFeederNo].outputCurrentSettings();
			}
			else
			{
				sendAnswer(1, F("feederNo invalid"));
			}
			break;
		}

		case MCODE_FACTORY_RESET:
		{
			commonSettings.version[0] = commonSettings.version[0] + 1;

			EEPROM.writeBlock(EEPROM_COMMON_SETTINGS_ADDRESS_OFFSET, commonSettings);

			sendAnswer(0, F("EEPROM invalidated, defaults will be loaded on next restart. Please restart now."));

			break;
		}

		default:
			sendAnswer(0, F("unknown or empty command ignored"));

			break;
	}
}

void listenToSerialStream()
{
	while (Serial.available())
	{
		// get the received byte, convert to char for adding to buffer
		char receivedChar = (char)Serial.read();

		// print back for debugging
		#ifdef DEBUG
		Serial.print(receivedChar);
		#endif

		// add to buffer
		inputBuffer += receivedChar;

		// if the received character is a newline, processCommand
		if (receivedChar == '\n')
		{
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
void setup()
{
	Serial.begin(SERIAL_BAUD);

	while (!Serial);

	Serial.println(F("Controller starting...")); Serial.flush();
	Serial.println(F("Here is some stuff saved in EEPROM. Paste in a textfile to backup these settings...")); Serial.flush();

	Serial.println("Controller has " + String(NUMBER_OF_FEEDER) + " feeders with " + String(NUMBER_OF_CONTROLLERS) + " PCA9685."); Serial.flush();

	/** Create instances of PCA9685 with incremental addresses
	*	PCA #1 manage feeders 1 - 16, #2 17 - 32, etc.
	*/
	for (uint8_t i = 0; i < NUMBER_OF_CONTROLLERS; i++)
	{
		Serial.print(F("Initializing PCA9685 n° "));
		Serial.println(i); Serial.flush();

		// servoControllers[i] = PCA9685();

		servoControllers[i].resetDevices();
		// delay(10);
		servoControllers[i].init(PCA9685_PhaseBalancer_Linear, PCA9685_OutputDriverMode_TotemPole, PCA9685_OutputEnabledMode_Normal, PCA9685_OutputDisabledMode_Low, PCA9685_ChannelUpdateMode_AfterAck);
		// delay(10);
		servoControllers[i].setPWMFreqServo();
		// delay(10);
		servoControllers[i].setAllChannelsPWM(310);
		// delay(10);

		// for (uint8_t j = 0; j < 16; j++)
		// {
		// 	Serial.println("Turning off channel " + String(j) + " of PCA n°" + String(i));
		// 	servoControllers[i].setChannelOff(j);
		// 	// delay(10);
		// }	
	}
	
	// setup listener to serial stream
	setupGCodeProc();

	//initialize active feeders, this is giving them an valid ID
	//needs to be done before factory reset to have a valid ID (eeprom-settings location is derived off the ID)
	executeCommandOnAllFeeder(cmdInitializeFeederWithId);

	//load commonSettings from eeprom
	EEPROM.readBlock(EEPROM_COMMON_SETTINGS_ADDRESS_OFFSET, commonSettings);

	//factory reset on first start or version changing
	if(strcmp(commonSettings.version,CONFIG_VERSION) != 0)
	{
		Serial.println(F("First start/Config version changed"));

		//reset needed
		executeCommandOnAllFeeder(cmdFactoryReset);

		//update commonSettings in EEPROM to have no factory reset on next start
		EEPROM.writeBlock(EEPROM_COMMON_SETTINGS_ADDRESS_OFFSET, commonSettings_default);
	}

	//print all settings to console
	// printCommonSettings();

	//setup feeder objects
	executeCommandOnAllFeeder(cmdSetup);	//setup everything first, then power on short. made it this way to prevent servos from driving to an undefined angle while being initialized
	delay(1000);		//have the last feeder's servo settled before disabling
	// executeCommandOnAllFeeder(cmdDisable); //while setup ran, the feeder were moved and remain in sIDLE-state -> it shall be disabled
	
	//print all settings of every feeder to console
	executeCommandOnAllFeeder(cmdOutputCurrentSettings);

	Serial.println(F("Controller up and ready! Have fun."));
}



// ------------------  L O O P -----------------------
void loop()
{
	// Process incoming serial data and perform callbacks
	listenToSerialStream();

	// Process servo control
	executeCommandOnAllFeeder(cmdUpdate);

	// delay(5);
}
