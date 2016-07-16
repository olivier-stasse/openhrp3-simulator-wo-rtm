#ifndef _SAMPLE_ROBOT_O2SR_HH_
#define _SAMPLE_ROBOT_O2SR_HH_

#include <string>
#include <vector>

#include <hrpCorba/Controller.hh>
#include <hrpCorba/ViewSimulator.hh>
#include <hrpCorba/DynamicsSimulator.hh>


class SampleRobotO2SR
{
public:
  SampleRobotO2SR();
  ~SampleRobotO2SR();

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

  /// Store the PID values for each actuator.
  std::vector<double> P_,I_,D_;

  /// Number of degrees of freedom.
  unsigned int nb_dofs_;
  
  /// Store angle values, speed and torque.
  OpenHRP::DblSequence q_,dq_,torque_, qref_, dqref_;

  OpenHRP::DblSequence sendto_,recvfrom_;

  /// Boolean 
  bool perform_control_;

  /// list of link names.
  std::vector<std::string> listOfLinks_;
};
#endif /* */
