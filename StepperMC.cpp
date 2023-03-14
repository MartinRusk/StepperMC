#include <Arduino.h>
#include "StepperMC.h"

// stepping scheme for the motor
const uint8_t phase_scheme[8][4] = 
{
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1},
  {1,0,0,0}
};

// constructor
StepperMC::StepperMC(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, uint16_t steps)
{
  // Initialize variables
  _init(steps);
  // Set Stepper interface
  _interface = stp4Wire;
  // Arduino pins for the motor control connection:
  _pin1 = pin1;
  _pin2 = pin2;
  _pin3 = pin3;
  _pin4 = pin4;
  // setup the pins on the microcontroller:
  pinMode(_pin1, OUTPUT);
  pinMode(_pin2, OUTPUT);
  pinMode(_pin3, OUTPUT);
  pinMode(_pin4, OUTPUT);
  // and start in idle mode
  _powerOff();
}

// constructor
StepperMC::StepperMC(uint8_t pinDir, uint8_t pinStep, uint16_t steps)
{
  // Initialize variables
  _init(steps);
  // Set Stepper interface
  _interface = stp2Wire;
  // Arduino pins for the motor control connection:
  _pin1 = pinDir;
  _pin2 = pinStep;
  _pin3 = 255;
  _pin4 = 255;
  // setup the pins on the microcontroller:
  pinMode(_pin1, OUTPUT);
  pinMode(_pin2, OUTPUT);
}

void StepperMC::_init(uint16_t steps)
{
  _stepAct = 0;
  _stepTarget = 0;
  _direction = dirStop;
  _backlash = 0;
  _backlashAct = 0;
  _stepMotor = 0;
  _isModulo = false;
  _isLimited = false;
  _stepsTurn = steps;
  _stepsModulo = 0;
  _feedConst = _stepsTurn / 360.0;
  _gearRatio = 1.0;
  _upperLimit = 0x7fffffff;
  _lowerLimit = 0x80000001;
  _delayStep = 1250;
  _cycle = 0;
  _cycleMin = 0;
  _cycleMax = 0;
  _rampConst = 0;
  _rampStep = 0;
  _stepsStop = 0;
  _delayPowersave = 1000000;
  _timeLastStep = micros() + _delayStep;
}

// cyclic handle of motion (call in loop)
void StepperMC::handle()
{
  // check if next step can be executed
  unsigned long now = micros();
  if (now > _timeLastStep + _delayStep)
  {
    // get new direction and step delay
    _calcDelay();    
    // do one step in the right direction
    if (_direction == dirPos)
    {
      // count step only when backlash fully compensated
      if (_stepUp())
      {
        _stepAct = _trimModulo(_stepAct + 1);
      }
      _timeLastStep = now;
    }
    else if (_direction == dirNeg)
    {      
      // count step only when backlash fully compensated
      if(_stepDown())
      {
        _stepAct = _trimModulo(_stepAct - 1);
      }
      _timeLastStep = now;
    }
    // activate powersave on standstill
    if ((_delayPowersave > 0) && (now > _timeLastStep + _delayPowersave))
    {
      _powerOff();
    }
  }
}

// update ramp and calculate new step delay 
void StepperMC::_calcDelay()
{
  // get distance to target
  int32_t diff = _diffModulo(_stepTarget - _stepAct);
  // no ramp? 
  if (_rampConst == 0)
  {
    // set direction and exit
    _direction = (diff > 0) ? dirPos : (diff < 0) ? dirNeg : dirStop;
    return;
  }
  // Stop when in Target
  if ((diff == 0) && (_stepsStop <= 5))
  {
    _direction = dirStop;
    _cycle = _cycleMax;
    _delayStep = 0;
    _rampStep = 0;
    return;
  }
  // detect necessary switch between acceleration and deceleration
  if (diff > 0) // positive turn needed?
  {
    if (_rampStep > 0) // accelerating or constant speed?
    {
      if ((_stepsStop >= diff) || (_direction == dirNeg))
      { 
        // start deceleration
        _rampStep = -_stepsStop;
      }
    }
    else if (_rampStep < 0) // decelerating?
    {
      if ((_stepsStop < diff) && (_direction == dirPos))
      {
        // accelerate again
        _rampStep = -_rampStep;
      }
    }
  }
  else if (diff < 0) // negative turn needed?
  {
    if (_rampStep > 0) // accelerating or constant speed?
    {
      if ((_stepsStop >= -diff) || (_direction == dirPos))
      {
        // start deceleration
        _rampStep = -_stepsStop;
      }
    }
    else if (_rampStep < 0) // decelerating?
    {
      if ((_stepsStop < -diff) && (_direction == dirNeg))
      {
        // accelerate again
        _rampStep = -_rampStep;
      }
    }
  }
  // on zero crossing 
  if (_rampStep == 0) 
  {
    // set required direction to target and reinitialize cycle time
    _direction = (diff > 0) ? dirPos : dirNeg;
    _cycle = _cycleMax;
    // get new stopping distance
    _stepsStop = _rampConst / (_cycle * _cycle);
    // next rampStep
    _rampStep++;
  }
  else
  {
    // update cycle time when not final speed reached
    if (_cycle > _cycleMin || _rampStep < 0)
    {
      _cycle = _cycle - ((2.0 * _cycle) / ((4 * _rampStep) + 1));
      // get new stopping distance
      _stepsStop = _rampConst / (_cycle * _cycle); 
      // next rampStep
      _rampStep++;
    }
  }
  // set current delay
  _delayStep = (unsigned long)_cycle;
}

// set new target position
void StepperMC::setIncrements(int32_t pos)
{
  if (_isLimited)
  {
    pos = min(max(pos, _lowerLimit), _upperLimit);
  }
  // enforce modulo
  _stepTarget = _trimModulo(pos);
}

// set relative target position
void StepperMC::setIncrementsRelative(int32_t steps)
{
  setIncrements(_stepTarget + steps);
}

// set new target position
void StepperMC::setPosition(float pos)
{
  setIncrements((int32_t)(pos * _feedConst));
}

// set new target position relative
void StepperMC::setPositionRelative(float pos)
{
  setIncrementsRelative((int32_t)(pos * _feedConst));
}

// automatic trim position in modulo range
int32_t StepperMC::_trimModulo(int32_t pos)
{
  if (_isModulo) 
  {
    if (pos >= _stepsModulo)
    {
      pos -= _stepsModulo;
    }
    if (pos < 0)
    {
      pos += _stepsModulo;
    }
  }
  return pos;
}

// automatic trim position difference in modulo range
int32_t StepperMC::_diffModulo(int32_t diff)
{
  if (_isModulo)
  {
    if (diff > (_stepsModulo >> 1))
    {
      diff -= _stepsModulo;
    }
    if (diff < -(_stepsModulo >> 1))
    {
      diff += _stepsModulo;
    }
  }
  return diff;
}

// return actual position
int32_t StepperMC::getIncrements()
{
  return (_stepAct);
}

// get new cuurrent position
float StepperMC::getPosition()
{
  return (float) _stepAct / _feedConst;
}

// check if target position reached
bool StepperMC::inTarget()
{
  return (_stepTarget == _stepAct);
}

void StepperMC::stop()
{
  if (_direction == dirPos)
  {
    _stepTarget = _trimModulo(_stepAct + _stepsStop);
  }
  else if (_direction == dirNeg)
  {
    _stepTarget = _trimModulo(_stepAct - _stepsStop);
  }
}

// wait and handle steps until target position reached
void StepperMC::moveTarget()
{
  while (!inTarget())
  {
    handle();
  }
}

// set actual and target position to zero
void StepperMC::setZero()
{
  _stepAct = 0;
  _stepTarget = 0;
}

// adjust zero position by some steps
void StepperMC::adjustZero(int32_t steps)
{
  _stepAct -= steps;
}

// set backlash compensation
void StepperMC::setBacklash(int32_t steps)
{
  _backlash = steps;
}

// set constant speed
void StepperMC::setSpeed(uint16_t freq)
{
  if (freq > 0)
  {
    _delayStep = 1000000UL / freq;
    _rampConst = 0;
  }
}

// set dynamic speed ramping
void StepperMC::setSpeed(uint16_t freq, uint16_t acc)
{
  if ((freq > 0) && (acc > 0))
  {
    _cycleMin = 1e6 / (float)freq;
    _cycleMax = 676e3 * sqrt(2.0 / ((float)acc));
    _cycle = _cycleMax;
    _rampConst = (5e11 / (float)acc);
  }
}

// make this a modulo axis
void StepperMC::setModulo(uint16_t steps)
{
  _isModulo = true;
  _isLimited = false;
  _stepsModulo = steps;
}

// remove limits and modulo
void StepperMC::setUnlimited()
{
  _isLimited = false;
  _isModulo = false;
  _lowerLimit = 0x80000001;
  _upperLimit = 0x7fffffff;
  _stepsModulo = 0;
}

// set software limits
void StepperMC::setPositionLimit(float lower, float upper)
{
  _isLimited = true;
  _isModulo = false;
  _lowerLimit = lower * _feedConst;
  _upperLimit = upper * _feedConst; 
}

// Feedrate per turn (default 360)
void StepperMC::setFeedConst(float feed)
{
  _feedConst = _stepsTurn / feed;
}

// invert direction
void StepperMC::reverseDir(bool neg)
{
  _negDir = neg;
}

void StepperMC::setPowersaveTime(uint16_t seconds)
{
  _delayPowersave = 1000000UL * seconds;
}

bool StepperMC::_stepUp()
{
  _stepMotor++;
  _step();
  if (_backlashAct < _backlash - 1)
  { 
    _backlashAct++;
    return false;
  }
  return true;
}

bool StepperMC::_stepDown()
{
  _stepMotor--;
  _step();
  if (_backlashAct > 0)
  { 
    _backlashAct--;
    return false;
  }
  return true;
}

// execute one step
void StepperMC::_step()
{
  if (_interface == stp4Wire)
  {
    int phase = (int)(_stepMotor & 0x07);
    if (_negDir)
    {
      // invert direction
      phase = 7 - phase;
    }
    digitalWrite(_pin1, phase_scheme[phase][0]);
    digitalWrite(_pin2, phase_scheme[phase][1]);
    digitalWrite(_pin3, phase_scheme[phase][2]);
    digitalWrite(_pin4, phase_scheme[phase][3]);
  }
  else if (_interface == stp2Wire)
  {
    digitalWrite(_pin1, _direction == dirPos);
    digitalWrite(_pin2, true);
    delayMicroseconds(1);
    digitalWrite(_pin2, false);
  }
}

// switch power off
void StepperMC::_powerOff()
{
  // powerOff is sensible only with 4 wire interface
  if (_interface == stp4Wire)
  {
    digitalWrite(_pin1, 0);
    digitalWrite(_pin2, 0);
    digitalWrite(_pin3, 0);
    digitalWrite(_pin4, 0);
  }
}
