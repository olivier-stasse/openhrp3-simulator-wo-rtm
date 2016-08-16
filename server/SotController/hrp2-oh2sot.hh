#ifndef _HRP2_OH_2_SOT_HH_
#define _HRP2_OH_2_SOT_HH_

#include <string>
#include <vector>

#include <hrpCorba/Controller.hh>
#include <hrpCorba/ViewSimulator.hh>
#include <hrpCorba/DynamicsSimulator.hh>

#include <sot/core/abstract-sot-external-interface.hh>
#include <dynamic_graph_bridge/sot_loader_basic.hh>

namespace dgsot=dynamicgraph::sot;

/** \brief Config variables
 */
struct robot_config_t
{
  /// \brief Name of the controller to load
  std::string libname;
  /// \brief Robot number of DoFs
  int nb_dofs;
  /// \brief Number of force sensors
  int nb_force_sensors;
  
};

class HRP2OH2SOT : public SotLoaderBasic 
{
public:
  HRP2OH2SOT();
  ~HRP2OH2SOT();

  void initialize();
  void start();
  void input();
  void output();
  void control();
  void stop();
  void restart();
  void shutdown();
  void set_path_to_library(std::string &);

  OpenHRP::DynamicsSimulator_ptr dynamicsSimulator_;

  std::string modelName_;

protected:

  /// Initialize position and velocity references.
  void initref();

  /// Initialize internal pids of the robot
  void initpids();

  /// Initialize list of link names.
  void initLinkNames();

  /// Initialize the stack of tasks.
  void initializeSoT();

  /// Store the PID values for each actuator.
  std::vector<double> P_,I_,D_;

  /// Number of degrees of freedom.
  unsigned int nb_dofs_;
  
  /// Store angle values, speed and torque.
  std::vector<double> q_,dq_, qref_, dqref_;
  std::vector<double> torques_, forces_;
  std::vector<double> accelerometer_,gyrometer_;
  std::vector<double> angleControl_;

  OpenHRP::DblSequence sendto_,recvfrom_;

  /// Boolean 
  bool perform_control_;

  /// list of link names.
  std::vector<std::string> listOfLinks_;

  /// \brief the sot-hrp2 controller
  dgsot::AbstractSotExternalInterface * m_sotController;

  void fillSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);
  void readControl(std::map<std::string,dgsot::ControlValues> &controlValues);

  /// \brief Config variables 
  robot_config_t robot_config_;

  /// Map of sensor readings
  std::map<std::string,dgsot::SensorValues> sensorsIn_;
  /// Map of control values
  std::map<std::string,dgsot::ControlValues> controlValues_;

};
#endif /* */
