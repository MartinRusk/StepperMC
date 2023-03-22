#ifndef StepperMC_h
#define StepperMC_h
#include <Arduino.h>

class StepperMC
{
public:
    /// @brief Constructor, connect 4 phase Stepper to Arduino pins
    /// @param pin1 Pin for Phase 1
    /// @param pin2 Pin for Phase 2
    /// @param pin3 Pin for Phase 3
    /// @param pin4 Pin for Phase 4
    /// @param steps Steps per motor revolution (optional, default = 4096)
    StepperMC(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, uint16_t steps = 4096);

    /// @brief Constructor, connect Stepper with directon and step lines
    /// @param pinDir Pin for motor direction
    /// @param pinStep Pin to execute a step
    /// @param steps Steps per motor revolution (optional, default = 4096)
    StepperMC(uint8_t pinDir, uint8_t pinStep, uint16_t steps = 4096);

    /// @brief Realtime handle, performs one motor step when necessary without blocking calling task.
    /// Call in fastest loop.
    void handle();

    /// @brief Set target position in increments
    /// @param pos Absolute target postion in inkrements (steps)
    void setIncrements(int32_t pos);

    /// @brief Set target position relative to current target position in increments
    /// @param steps Relative target postion in inkrements (steps)
    void setIncrementsRelative(int32_t steps);

    /// @brief Get current position in increments
    /// @return Current postion in inkrements (steps)
    int32_t getIncrements();

    /// @brief Set target position in engineering units
    /// @param pos Absolute target position in engineering units
    void setPosition(float pos);

    /// @brief Set target position relative to current target position in engineering units
    /// @param pos Relative target postion in engineering units
    void setPositionRelative(float pos);

    /// @brief Get current position in engineering units
    /// @return Current postion in engineering units
    float getPosition();

    /// @brief Call handle cyclic until target position reached (blocking)
    void moveTarget();

    /// @brief Check whether target position is reached
    /// @return true = target position reached, false = target not reached
    bool inTarget();

    /// @brief Set new target position for fast stop from current postion
    void stop();

    /// @brief  Set the current Position as zero. Use only on standstill.
    void setZero();

    /// @brief Set speed and optional acceleration for motion commands.
    /// @param freqMax Maximum speed as increments (steps) per second
    /// @param acc Optional acceleration for speed ramps. acc = 0 means constant speed and
    /// reduces computing time of handle() significantly.
    void setSpeed(uint16_t freqMax, uint16_t acc = 0);

    /// @brief Adjust the zero position by some increments (steps)
    /// @param steps Number of steps for adjustment
    void adjustZero(int32_t steps);

    /// @brief Set backlash of drive. Motion within backlash range is not counted for actual position.
    /// @param steps Basklash range in incremets (steps)
    void setBacklash(int32_t steps);

    /// @brief Set gear ratio between motor and load. Used for motion in engineering units.
    /// @param motor Number of gear teeth on motor side (default 1)
    /// @param load Number of gear teeth on load side (default 1)
    void setGearRatio(int32_t motor, int32_t load);

    /// @brief Set feed constant for motion in engineering units.
    /// @param feed One load side revolution in engineering units (default: 360)
    void setFeedConst(float feed);

    /// @brief Set modulo range for motion. After the modulo distance possition repeats. Useful e.g. for repeating 360° axis.
    /// @param steps Number of increments (steps) after which position is reset. steps = 0 takes one turn on liad side.
    /// Take care with uneven gear ratios leading to fractional increments, they lead to inaccuracies.
    void setModulo(uint16_t steps = 0);

    /// @brief Remove any limits and modulo settings for axis (default mode)
    void setUnlimited();

    /// @brief  Set position limits in engineering units and make axis limited. Target positions beyond positive or negative limit
    /// are truncated to the limit.
    /// @param lower Lower position limit in engineering units
    /// @param upper Upper position limit in engineering units
    void setPositionLimit(float lower, float upper);

    /// @brief Reverse motion direction of axis on low level
    /// @param neg true: invert motion direction, false: do not invert
    void reverseDir(bool neg);

    /// @brief Set timer for powersave mode of axis.
    /// @param seconds Number of seconds after which the amplifiers are switched to idle.
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
