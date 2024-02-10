#ifndef _SHIELD_h
#define _SHIELD_h
#include "shields_def.h"

#if SHIELD(NATIVE)
  #include "shield_native.h"
#elif SHIELD(SENSOR)
  #include "shield_sensor.h"
#elif SHIELD(I2C)
  #include "shield_i2c.h"
#else
  #error "invalid board, please select a proper shield"
#endif

//DEFINE _SHIELD_h-ENDIF!!!
#endif
