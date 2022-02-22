#ifndef XS_RVIZ_XS_REGISTER_DESCRIPTIONS
#define XS_RVIZ_XS_REGISTER_DESCRIPTIONS

#include <unordered_map>

namespace xs_register_descriptions
{

typedef struct {
    std::string name;
    std::string description;
    int address;
    std::string units;
} XSRegisterDescription;

static XSRegisterDescription desc_goal_position = {
  "Goal_Position",
  "The Goal Position sets desired position [pulses]. From the front view of DYNAMIXEL, CCW is an increasing direction, whereas CW is a decreasing direction. Units: 1 pulse",
  116,
  "pulses"
};

static XSRegisterDescription desc_goal_velocity = {
  "Goal_Velocity",
  "Use the Goal Velocity to set a desired velocity when the Operating Mode is Velocity Control Mode. Units: 0.229 rev/min",
  104,
  "0.229 rev/min"
};

static XSRegisterDescription desc_goal_current = {
  "Goal_Current",
  "Use Goal Current to set a desired current when the Operating Mode is Torque Control Mode. The Goal Current can also be used to set current limits in Current-based Position Control Mode. Units: 2.69 mA",
  102,
  "2.69 mA"
};

static XSRegisterDescription desc_goal_pwm = {
  "Goal_PWM",
  "When the Operating Mode is PWM Control Mode, both the PID and Feedforward controllers will be deactivated as the Goal PWM value directly controls a motor via an inverter. But on the other Operating Mode, the Goal PWM limits PWM value only. Units: 0.113%",
  100,
  "0.113%"
};

static XSRegisterDescription desc_present_position = {
  "Present_Position",
  "The Present Position indicates present Position. Units: 1 pulse",
  132,
  "1 pulse"
};

static XSRegisterDescription desc_present_velocity = {
  "Present_Velocity",
  "This value indicates current Velocity. Units: 0.229 rev/min",
  128,
  "0.229 rev/min"
};

static XSRegisterDescription desc_present_current = {
  "Present_Current",
  "This value indicates current Current. Units: 2.69mA",
  126,
  "2.69 mA"
};

static XSRegisterDescription desc_present_temperature = {
  "Present_Temperature",
  "The Present Temperature indicates internal temperature of DYNAMIXEL. Units: 1 degree Celsius",
  146,
  "1 degree Celsius"
};

static std::unordered_map<std::string, XSRegisterDescription> descriptions = 
{
  {"Goal_Position",       desc_goal_position},
  {"Goal_Velocity",       desc_goal_velocity},
  {"Goal_Current",        desc_goal_current},
  {"Goal_PWM",            desc_goal_pwm},
  {"Present_Position",    desc_present_position},
  {"Present_Velocity",    desc_present_velocity},
  {"Present_Current",     desc_present_current},
  {"Present_Temperature", desc_present_temperature}
};
    
} // namespace xs_register_descriptions

#endif // XS_RVIZ_XS_REGISTER_DESCRIPTIONS
