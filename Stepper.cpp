#include <Arduino.h>
#include "Stepper.h"

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
Stepper::Stepper(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
{
  // Initialize variables
  _stepAct = 0;
  _stepTarget = 0;
  _backlash = 0;
  _backlashAct = 0;
  _stepMotor = 0;
  _isModulo = false;
  _isLimited = false;
  _stepsTurn = 4096;
  _stepsModulo = 0;
  _feedConst = _stepsTurn / 360.0;
  _gearRatio = 1.0;
  _upperLimit = 0x7fffffff;
  _lowerLimit = 0x80000001;
  _delayMin = 1250;
  _delayMax = _delayMin;
  _delayStep = _delayMin;
  _rampConst = 0;
  _delayPowersave = 1000000;
  _timeLastStep = micros() + _delayStep;
  
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

// variable steps (e.g. with gear)
Stepper::Stepper(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, uint16_t steps) : Stepper(pin1, pin2, pin3, pin4) 
{
  _stepsTurn = steps;
}

// cyclic handle of motion (call in loop)
void Stepper::handle()
{
  // check if next step can be executed (rate limitation)
  unsigned long now = micros();
  if (now > _timeLastStep + _delayStep)
  {
    // do one step in the right direction
    int32_t diff = _diffModulo(_stepTarget - _stepAct);
    if (diff > 0)
    {
      // count step only when backlash fully compensated
      if (_stepUp())
      {
        _stepAct = _trimModulo(_stepAct + 1);
        _calcDelay();
      }
      _timeLastStep = now;
    }
    else if (diff < 0)
    {      
      // count step only when backlash fully compensated
      if(_stepDown())
      {
        _stepAct = _trimModulo(_stepAct - 1);
        _calcDelay();
      }
      _timeLastStep = now;
    }
    if ((_delayPowersave > 0) && (now > _timeLastStep + _delayPowersave))
    {
      _powerOff();
    }
  }
}

void Stepper::_calcDelay()
{
  int32_t diff = _diffModulo(_stepTarget - _stepAct);
  // constant speed?
  if (_rampConst == 0)
  {
    _direction = (diff > 0) ? dirPos : (diff < 0) ? dirNeg : dirStop;
    _delayStep = _delayMax;
    return;
  }

  // Stop when in Target
  int32_t stepsStop = _rampConst / (_delayStep * _delayStep);
  if ((diff == 0) && (stepsStop <= 1))
  {
    _direction = dirStop;
    _delayStep = 0;
    _rampStep = 0;
    return;
  }

  // Positive turn needed
  if (diff > 0)
  {
    if (_rampStep > 0) // accelerating or constant speed?
    {
      if ((stepsStop >= diff) || (_direction == dirNeg))
      {
        _rampStep = -stepsStop;
      }
    }
    else if (_rampStep < 0) // decelerating?
    {
      if ((stepsStop < diff) && (_direction == dirPos))
      {
        _rampStep = -_rampStep;
      }
    }
  }
  else if (diff < 0)
  {
    if (_rampStep > 0) // accelerating?
    {
      if ((stepsStop >= -diff) || (_direction == dirPos))
      {
        _rampStep = -stepsStop;
      }
    }
    else if (_rampStep < 0) // decelerating?
    {
      if ((stepsStop < -diff) && (_direction == dirNeg))
      {
        _rampStep = -_rampStep;
      }
    }
  }

  if (_rampStep == 0)
  {
    _direction = (diff > 0) ? dirPos : dirNeg;
    _delayStep = _delayMax;
  }
  else
  {
    _delayStep = max(_delayStep - ((2 * _delayStep) / ((4 * _rampStep) + 1)), _delayMin);
  }
  _rampStep++;
}

// set new target position
void Stepper::setIncrements(int32_t pos)
{
  if (_isLimited)
  {
    pos = min(max(pos, _lowerLimit), _upperLimit);
  }
  // enforce modulo
  _stepTarget = _trimModulo(pos);
}

// set relative target position
void Stepper::setIncrementsRelative(int32_t steps)
{
  setIncrements(_stepTarget + steps);
}

// set new target position
void Stepper::setPosition(float pos)
{
  setIncrements((int32_t)(pos * _feedConst));
}

// set new target position relative
void Stepper::setPositionRelative(float pos)
{
  setIncrementsRelative((int32_t)(pos * _feedConst));
}

// automatic trim position in modulo range
int32_t Stepper::_trimModulo(int32_t pos)
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
int32_t Stepper::_diffModulo(int32_t diff)
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
int32_t Stepper::getIncrements()
{
  return (_stepAct);
}

// get new cuurrent position
float Stepper::getPosition()
{
  return (float) _stepAct / _feedConst;
}

// check if target position reached
bool Stepper::inTarget()
{
  return (_stepTarget == _stepAct);
}

// wait and handle steps until target position reached
void Stepper::moveTarget()
{
  while (!inTarget())
  {
    handle();
  }
}

// set actual and target position to zero
void Stepper::setZero()
{
  _stepAct = 0;
  _stepTarget = 0;
}

// adjust position by some steps
void Stepper::adjustZero(int32_t steps)
{
  _stepAct -= steps;
}

// adjust position by some steps
void Stepper::setBacklash(int32_t steps)
{
  _backlash = steps;
}

// override stepper frequency
void Stepper::setFrequency(uint16_t freqMax, uint16_t acc)
{
  if (freqMax > 0)
  {
    _delayMin = 1000000UL / freqMax;
  }
  if (acc > 0)
  {
//    _delayMax = (long)(676000.0 * sqrt((float)accTime / ((float)freqMax * 500.0)));
    _delayMax = (long)(676000.0 * sqrt(2.0 / ((float)acc)));
    _rampConst = 250000UL * _delayMin;
  }
  else
  {
    _delayMax = _delayMin;
    _rampConst = 0;
  }
  _delayStep = _delayMax;
}

// make this a modulo axis
void Stepper::setModulo(uint16_t steps)
{
  _isModulo = true;
  _isLimited = false;
  _stepsModulo = steps;
}

// remove limits and modulo
void Stepper::setUnlimited()
{
  _isLimited = false;
  _isModulo = false;
  _lowerLimit = 0x80000001;
  _upperLimit = 0x7fffffff;
  _stepsModulo = 0;
}

// set software limits
void Stepper::setPositionLimit(float lower, float upper)
{
  _isLimited = true;
  _isModulo = false;
  _lowerLimit = lower * _feedConst;
  _upperLimit = upper * _feedConst; 
}

// Feedrate per turn (default 360)
void Stepper::setFeedConst(float feed)
{
  _feedConst = _stepsTurn / feed;
}

// invert direction
void Stepper::reverseDir(bool neg)
{
  _negDir = neg;
}

void Stepper::setPowersaveTime(uint16_t seconds)
{
  _delayPowersave = 1000000UL * seconds;
}

bool Stepper::_stepUp()
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

bool Stepper::_stepDown()
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
void Stepper::_step()
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

// switch power off
void Stepper::_powerOff()
{
  digitalWrite(_pin1, 0);
  digitalWrite(_pin2, 0);
  digitalWrite(_pin3, 0);
  digitalWrite(_pin4, 0);
}
