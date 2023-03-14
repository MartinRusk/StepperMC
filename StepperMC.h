#ifndef StepperMC_h
#define StepperMC_h

class StepperMC
{
public:
  StepperMC(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, uint16_t steps = 4096);
  StepperMC(uint8_t pinDir, uint8_t pinStep, uint16_t steps = 4096);
  void handle();
  void setIncrements(int32_t pos);
  void setIncrementsRelative(int32_t steps);
  int32_t getIncrements();
  void setPosition(float pos);
  void setPositionRelative(float pos);
  float getPosition();
  void moveTarget();
  bool inTarget();
  void stop();
  void setZero();
  void setSpeed(uint16_t freqMax, uint16_t acc = 0);
  void adjustZero(int32_t steps);
  void setBacklash(int32_t steps);
  void setGearRatio(int32_t motor, int32_t load);
  void setFeedConst(float feed);
  void setModulo(uint16_t steps = 0);
  void setUnlimited();
  void setPositionLimit(float lower, float upper);
  void reverseDir(bool neg);
  void setPowersaveTime(uint16_t seconds);

private:
  void _init(uint16_t steps);
  void _calcDelay();
  int32_t _trimModulo(int32_t pos);
  int32_t _diffModulo(int32_t diff);
  bool _stepUp();
  bool _stepDown();
  void _step();
  void _powerOff();
  enum {
    stp4Wire,
    stp2Wire
  } _interface;
  bool _isModulo;
  bool _isLimited;
  bool _negDir;
  uint16_t _stepsTurn;
  int32_t _stepAct;
  int32_t _stepTarget;
  int32_t _backlash;
  int32_t _backlashAct;
  int32_t _stepMotor;
  int32_t _stepsModulo;
  int32_t _upperLimit;
  int32_t _lowerLimit;
  float _feedConst;
  float _gearRatio;
  // motor pin numbers
  uint8_t _pin1;
  uint8_t _pin2;
  uint8_t _pin3;
  uint8_t _pin4;
  // timing
  unsigned long _delayPowersave;
  unsigned long _timeLastStep;
  unsigned long _delayStep;
  enum {
    dirStop,
    dirPos,
    dirNeg
  } _direction;
  // ramp
  float _cycle;
  float _cycleMin;
  float _cycleMax;
  float _rampConst;
  int32_t _rampStep;
  int32_t _stepsStop;
};

#endif
