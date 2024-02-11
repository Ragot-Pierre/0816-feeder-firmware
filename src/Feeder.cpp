#include "Feeder.h"
#include "config.h"

bool FeederClass::isInitialized() {
	if(this->feederNo == -1)
	  return false;
	else
		return true;
}

void FeederClass::initialize(uint16_t _feederNo) {
	this->feederNo = _feederNo;
}

#ifdef HAS_FEEDBACKLINES
bool FeederClass::hasFeedbackLine() {
	if(feederFeedbackPinMap[this->feederNo] != -1) {
		return true;
	} else {
		return false;
	}
}
#endif

void FeederClass::outputCurrentSettings() {
	Serial.print("M");
	Serial.print(MCODE_UPDATE_FEEDER_CONFIG);
	Serial.print(" N");
	Serial.print(this->feederNo);
	Serial.print(" A");
	Serial.print(this->feederSettings.full_advanced_angle);
	Serial.print(" B");
	Serial.print(this->feederSettings.half_advanced_angle);
	Serial.print(" C");
	Serial.print(this->feederSettings.retract_angle);
	Serial.print(" F");
	Serial.print(this->feederSettings.feed_length);
	Serial.print(" S");
	Serial.print((float)this->feederSettings.advance_angle_speed/256, 3);
	Serial.print(" R");
	Serial.print((float)this->feederSettings.retract_angle_speed/256, 3);
	Serial.print(" U");
	Serial.print(this->feederSettings.time_to_settle);
	Serial.print(" V");
	Serial.print(this->feederSettings.motor_min_pulsewidth);
	Serial.print(" W");
	Serial.print(this->feederSettings.motor_max_pulsewidth);
	Serial.println();
}

void FeederClass::setup(PCA9685 *controllerList) {
	//load settings from eeprom
	this->loadFeederSettings();

	//attach servo to pin, after settings are loaded
	Serial.println("Feeder " + String(this->feederNo) + " assigned to controller " + String((uint8_t) trunc((double) this->feederNo / 16.0)));

	this->servoController = &controllerList[(uint8_t) trunc((double) this->feederNo / 16.0)];

	//put on defined position
	this->gotoRetractPosition();
}

FeederClass::sFeederSettings FeederClass::getSettings() {
	return this->feederSettings;
}

void FeederClass::setSettings(sFeederSettings UpdatedFeederSettings) {
	this->feederSettings=UpdatedFeederSettings;


	#ifdef DEBUG
		Serial.println(F("updated feeder settings"));
		this->outputCurrentSettings();
	#endif
}

void FeederClass::loadFeederSettings() {
	uint16_t adressOfFeederSettingsInEEPROM = EEPROM_FEEDER_SETTINGS_ADDRESS_OFFSET + this->feederNo * sizeof(this->feederSettings);
	EEPROM.readBlock(adressOfFeederSettingsInEEPROM, this->feederSettings);

	#ifdef DEBUG
		Serial.println(F("loaded settings from eeprom:"));
		this->outputCurrentSettings();
	#endif
}

void FeederClass::saveFeederSettings() {
	uint16_t adressOfFeederSettingsInEEPROM = EEPROM_FEEDER_SETTINGS_ADDRESS_OFFSET + this->feederNo * sizeof(this->feederSettings);
	EEPROM.writeBlock(adressOfFeederSettingsInEEPROM, this->feederSettings);


	#ifdef DEBUG
		Serial.println(F("stored settings to eeprom:"));
		this->outputCurrentSettings();
	#endif
}

void FeederClass::factoryReset() {
	//just save the defaults to eeprom...

	this->saveFeederSettings();
}


void FeederClass::gotoPostPickPosition() {
  if ((this->feederPosition==sAT_FULL_ADVANCED_POSITION) || 
  	  (this->feederPosition==sAT_UNLOAD_POSITION)) {
    this->gotoRetractPosition();
    #ifdef DEBUG
      Serial.println("gotoPostPickPosition retracted feeder");
    #endif
  } else {
    #ifdef DEBUG
      Serial.println("gotoPostPickPosition didn't need to retract feeder");
    #endif

  }
}

void FeederClass::gotoRetractPosition() {
	this->startMove(this->feederSettings.retract_angle,sAT_RETRACT_POSITION);
	#ifdef DEBUG
		Serial.println("going to retract now");
	#endif
}

void FeederClass::gotoHalfAdvancedPosition() {
	this->startMove(this->feederSettings.half_advanced_angle,sAT_HALF_ADVANCED_POSITION);
	#ifdef DEBUG
		Serial.println("going to half adv now");
	#endif
}

void FeederClass::gotoFullAdvancedPosition() {
	this->startMove(this->feederSettings.full_advanced_angle,sAT_FULL_ADVANCED_POSITION);
	#ifdef DEBUG
		Serial.println("going to full adv now");
	#endif
}

void FeederClass::gotoUnloadPosition() {
	this->startMove(0,sAT_UNLOAD_POSITION);
	#ifdef DEBUG
		Serial.println("going to unload now");
	#endif
}

void FeederClass::gotoAngle(uint8_t angle) {
	
	this->position = (uint16_t)angle << 8;
	this->targetPosition = this->position;
	#ifdef DEBUG
	Serial.println("Moving feeder " + String(this->feederNo) + " to angle " + String(angle));	
	#endif // DEBUG
	this->servoController->setChannelPWM(this->feederNo, map(angle, 0, 180, this->feederSettings.motor_min_pulsewidth, this->feederSettings.motor_max_pulsewidth));
	
	#ifdef DEBUG
		Serial.print("going to ");
		Serial.print(angle);
		Serial.println("deg");
	#endif
}

bool FeederClass::advance(uint8_t feedLength, bool overrideError = false) {

	#ifdef DEBUG
		Serial.println(F("advance triggered"));
		Serial.println(this->reportFeederErrorState());
	#endif
	
	
	#ifdef DEBUG
		Serial.print(F("feederIsOk: "));
		Serial.println(this->feederIsOk());
		Serial.print(F("overrideError: "));
		Serial.println(overrideError);
	#endif
	
	//check whether feeder is OK before every advance command
	if( !this->feederIsOk() ) {
		//feeder is in error state, usually this would lead to exit advance with false and no advancing cycle started
		 
		 if(!overrideError) {
			//return with false means an error, that is not ignored/overridden
			//error, and error was not overridden -> return false, advance not successful
			return false;
		 } else {
			#ifdef DEBUG
				Serial.println(F("overridden error temporarily"));
			#endif
			 
		 }
	}

	//check, what to do? if not, return quickly
	if(feedLength==0) {
		//nothing to do, just return
		#ifdef DEBUG
			Serial.println(F("advance ignored, 0 feedlength was given"));
		#endif
	} else if ( feedLength>0 && this->feederState!=sIDLE ) {
		//last advancing not completed! ignore newly received command
		//TODO: one could use a queue
		#ifdef DEBUG
		
			Serial.print(F("advance ignored, feedlength>0 given, but feederState!=sIDLE"));
			Serial.print(F(" (feederState="));
			Serial.print(this->feederState);
			Serial.println(F(")"));
		#endif
	} else {
		//OK, start new advance-proc
		//feed multiples of 2 possible: 2/4/6/8/10/12,...
		#ifdef DEBUG
			Serial.print(F("advance initialized, remainingFeedLength="));
			Serial.println(feedLength);
		#endif
		this->remainingFeedLength=feedLength;
		this->advanceNext();
	}

	//return true: advance started okay
	return true;
}

void FeederClass::advanceNext() {
	#ifdef DEBUG
		Serial.print("remainingFeedLength before working: ");
		Serial.println(this->remainingFeedLength);
	#endif
	switch (this->feederPosition) {
		/* ------------------------------------- UNLOAD AND RETRACT POS ---------------------- */
		case sAT_UNLOAD_POSITION:
		case sAT_RETRACT_POSITION: {
			if(this->remainingFeedLength>=FEEDER_MECHANICAL_ADVANCE_LENGTH) {
				//goto full advance-pos
				this->gotoFullAdvancedPosition();
				this->remainingFeedLength-=FEEDER_MECHANICAL_ADVANCE_LENGTH;
			} else if(this->remainingFeedLength>=FEEDER_MECHANICAL_ADVANCE_LENGTH/2) {
				//goto half advance-pos
				this->gotoHalfAdvancedPosition();
				this->remainingFeedLength-=FEEDER_MECHANICAL_ADVANCE_LENGTH/2;
			}

		}
		break;

		/* ------------------------------------- HALF-ADVANCED POS ---------------------- */
		case sAT_HALF_ADVANCED_POSITION: {
			if(this->remainingFeedLength>=FEEDER_MECHANICAL_ADVANCE_LENGTH/2) {
				//goto full advance-pos
				this->gotoFullAdvancedPosition();
				this->remainingFeedLength-=FEEDER_MECHANICAL_ADVANCE_LENGTH/2;
			}
		}
		break;

		/* ------------------------------------- FULL-ADVANCED POS ---------------------- */
		case sAT_FULL_ADVANCED_POSITION: {
	// if coming here and remainingFeedLength==0, then the function is aborted above already, thus no retract after pick
	// if coming here and remainingFeedLength >0, then the feeder goes to retract for next advance move
			this->gotoRetractPosition();
		}
		break;

		default: {
			//state not relevant for advancing...
			//return error, should not occur?
		}
		break;
	}

	#ifdef DEBUG
		Serial.print("remainingFeedLength after working: ");
		Serial.println(this->remainingFeedLength);
	#endif
	//just finished advancing? set flag to send ok in next run after settle-time to let the pnp go on
	if(this->remainingFeedLength==0) {
		this->advanceInProgress = true;
	}
}

void FeederClass::startMove(uint8_t angle, sFeederPosition pos) {
	this->targetPosition = (uint16_t)angle << 8;
	this->feederPosition = pos;
	this->feederState = sMOVING;
	this->lastTimePositionChange = millis();
	this->moveServoToTarget(1);
}

bool FeederClass::moveServoToTarget(uint8_t ms) {
	uint8_t posOld = this->position >> 8;
	if (posOld == 0)	// Force move at angle=0
		posOld = 255;
	while (ms--) {
		if (this->position < this->targetPosition) {
			uint16_t delta = this->targetPosition - this->position;
			if ((this->feederSettings.advance_angle_speed > 0) && (delta > this->feederSettings.advance_angle_speed))
				delta=this->feederSettings.advance_angle_speed;
			this->position += delta;
		} else if (this->position > this->targetPosition) {
			uint16_t delta = this->position - this->targetPosition;
			if ((this->feederSettings.retract_angle_speed > 0) && (delta > this->feederSettings.retract_angle_speed))
				delta=this->feederSettings.retract_angle_speed;
			this->position -= delta;
		} else {
			break;
		}
	}
	uint8_t posNow = this->position >> 8;
	if (posNow != posOld)
	#ifdef DEBUG
		Serial.println("Moving feeder " + String(this->feederNo) + " to position " + String(posNow));
	#endif // DEBUG
		this->servoController->setChannelPWM(this->feederNo, map(posNow, 0, 180, this->feederSettings.motor_min_pulsewidth, this->feederSettings.motor_max_pulsewidth));
		// delay(1);
	return this->position != this->targetPosition;
}

bool FeederClass::feederIsOk() {
	if(this->getFeederErrorState() == sERROR) {
		return false;
	} else {
		return true;
	}
}

FeederClass::tFeederErrorState FeederClass::getFeederErrorState() {
#ifdef HAS_FEEDBACKLINES
	if(!this->hasFeedbackLine()) {
		//no feedback-line, return always OK
		//no feedback pin defined or feedback shall be ignored
		return sOK_NOFEEDBACKLINE;
	}

	if( digitalRead((uint8_t)feederFeedbackPinMap[this->feederNo]) == LOW ) {
		//the microswitch pulls feedback-pin LOW if tension of cover tape is OK. motor to pull tape is off then
		//no error
		return sOK;
	} else {
		//microswitch is not pushed down, this is considered as an error

		if(this->feederSettings.ignore_feedback==1) {
			//error present, but ignore
			return sERROR_IGNORED;
		} else {
			//error present, report fail
			return sERROR;
		}

	}
#else
  return sOK_NOFEEDBACKLINE;
#endif
}

String FeederClass::reportFeederErrorState() {
	switch(this->getFeederErrorState()) {
		case sOK_NOFEEDBACKLINE:
			return "getFeederErrorState: sOK_NOFEEDBACKLINE (no feedback line for feeder, impliciting feeder OK)";
		break;
		case sOK:
			return "getFeederErrorState: sOK (feedbackline checked, explicit feeder OK)";
		break;
		case sERROR_IGNORED:
			return "getFeederErrorState: sERROR_IGNORED (error, but ignored per feeder setting X1)";
		break;
		case sERROR:
			return "getFeederErrorState: sERROR (error signaled on feedbackline)";
		break;
		
		default:
			return "illegal state in reportFeederErrorState";
	}
}

//called when M-Code to enable feeder is issued
void FeederClass::enable() {
	
	this->feederState=sIDLE;
	this->advanceInProgress = false;
	
	this->servoController->setChannelOn(this->feederNo % 16);
	this->servoController->setChannelPWM(this->feederNo % 16, 310);
}

//called when M-Code to disable feeder is issued
void FeederClass::disable() {
  
	this->feederState=sDISABLED;
	
	this->servoController->setChannelOff(this->feederNo % 16);
}

void FeederClass::update() {

#ifdef HAS_FEEDBACKLINES
	//routine for detecting manual feed via tensioner microswitch.
	//useful for setup a feeder. press tensioner short to advance by feeder's default feed length
	//feeder have to be enabled for this, otherwise this feature doesn't work and pressing the tensioner can't be detected due to open mosfet on controller pcb.
	if(this->feederState==sIDLE) {		//only check feedback line if feeder is idle. this shall not interfere with the feedbackline-checking to detect the error state of the feeder
		
		if (millis() - this->lastTimeFeedbacklineCheck >= 10UL) {	//to debounce, check every 10ms the feedbackline.
			
			this->lastTimeFeedbacklineCheck=millis();		//update last time checked
		
			int buttonState = digitalRead(feederFeedbackPinMap[this->feederNo]);	//read level of feedbackline (active low)
		
			if ( this->feedbackLineTickCounter > 0 ) {		//to debounce there is a timer
				this->feedbackLineTickCounter++;
			}
		
			
			if ( (buttonState != this->lastButtonState) && (buttonState == LOW)) {		//event: button state changed to low (tensioner pressed)
				this->lastButtonState=buttonState;		//update state
				this->feedbackLineTickCounter=1;		//start counter
				#ifdef DEBUG
					Serial.println(F("buttonState changed to low"));
				#endif
			} else if (buttonState != this->lastButtonState) {
				this->lastButtonState=buttonState;	//update in case button went high again
			}
			
			if ( (this->feedbackLineTickCounter > 5) ) {
				//after 50ms the microswitch is expected to be debounced, so a potential manual feed can be issued, when going to high level again.
				if(buttonState==HIGH) {
					//button released, we have a valid feed command now
					#ifdef DEBUG
						Serial.print(F("Manual feed triggered for feeder N"));
						Serial.print(this->feederNo);
						Serial.print(F(", advancing feeders default length "));
						Serial.print(this->feederSettings.feed_length);
						Serial.println(F("mm."));
					#endif
					
					//trigger feed with default feeder length, errors are overridden.
					this->advance(this->feederSettings.feed_length,true);
					
					//reset
					this->feedbackLineTickCounter=0;
				}
				
				//check for invalidity
				if (this->feedbackLineTickCounter > 50) {	//button pressed too long (this is the case, too, if the cover tape was inserted and properly tensioned)
					
					#ifdef DEBUG
						Serial.println(F("Potential manual feed rejected (button pressed too long, probably cover tape was inserted properly)"));
					#endif
					
					//reset counter to reject potential feed
					this->feedbackLineTickCounter=0;
				}
			}
		}
		return;
	} else {
		//permanently reset vars to don't do anything if not idle...
		this->lastButtonState = digitalRead(feederFeedbackPinMap[this->feederNo]);	//read level of feedbackline (active low)
		feedbackLineTickCounter=0;
	}
#else
	if (this->feederState==sIDLE)
		return;
#endif
  
	if (this->feederState==sMOVING) {	// Move in progress
		uint8_t dt = millis() - this->lastTimePositionChange;
		if (dt == 0)
			return;
		this->lastTimePositionChange += dt;
		if (this->moveServoToTarget(dt))
			return;
		this->feederState=sSETTLE;
	}

	//time to change the position?
	if (millis() - this->lastTimePositionChange >= (unsigned long)this->feederSettings.time_to_settle) {

		//now servo is expected to have settled at its designated position, so do some stuff
		if(this->advanceInProgress) {
			this->advanceInProgress = false;
			Serial.println("ok, advancing cycle completed");
		}

		//if no need for feeding exit fast.
		if(this->remainingFeedLength==0) {
			if(this->feederState!=sDISABLED)
				//if feeder are not disabled:
				//make sure sIDLE is entered always again (needed if gotoXXXPosition functions are called directly instead by advance() which would set a remainingFeedLength)
				this->feederState=sIDLE;
				
			return;
		}
		this->feederState=sMOVING;
		this->advanceNext();
	}
}
