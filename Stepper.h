#ifndef Stepper_h
#define Stepper_h

class Stepper
{
public:
  Stepper(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4);
  Stepper(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, uint16_t steps);
  void handle();
  void setIncrements(int32_t pos);
  void setIncrementsRelative(int32_t steps);
  int32_t getIncrements();
  void setPosition(float pos);
  void setPositionRelative(float pos);
  float getPosition();
  void moveTarget();
  bool inTarget();
  void setZero();
  void adjustZero(int32_t steps);
  void setBacklash(int32_t steps);
  void setFeedConst(float feed);
  void setModulo(uint16_t steps);
  void setUnlimited();
  void setPositionLimit(float lower, float upper);
  // void setFrequency(uint16_t freq);
  void setFrequency(uint16_t freq, uint16_t accTime = 0);
  void reverseDir(bool neg);
  void setPowersaveTime(uint16_t seconds);

private:
  bool _stepUp();
  bool _stepDown();
  void _step();
  int32_t _trimModulo(int32_t pos);
  int32_t _diffModulo(int32_t diff);
  void _calcDelay();
  void _addBacklash();
  void _powerOff();
  uint16_t _stepsTurn;
  int32_t _stepAct;
  int32_t _stepTarget;
  int32_t _backlash;
  int32_t _backlashAct;
  int32_t _stepMotor;
  bool _isModulo;
  bool _isLimited;
  int32_t _stepsModulo;
  int32_t _upperLimit;
  int32_t _lowerLimit;
  float _feedConst;
  float _gearRatio;
  bool _negDir;

  // motor pin numbers
  uint8_t _pin1;
  uint8_t _pin2;
  uint8_t _pin3;
  uint8_t _pin4;
  
  //timing
  unsigned long _delayPowersave;
  // ramp
  unsigned long _timeLastStep;
  unsigned long _delayStep;
  unsigned long _delayMin;
  unsigned long _delayMax;
  int32_t _rampConst;
  int32_t _rampStep;
  enum
  {
    dirStop,
    dirPos,
    dirNeg
  } _direction;

};

#endif