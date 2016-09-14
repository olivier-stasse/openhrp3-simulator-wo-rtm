#ifndef _SIM_SCHEDULER_HH_
#define _SIM_SCHEDULER_HH_

#include <hrpUtil/OnlineViewerUtil.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpCorba/DynamicsSimulator.hh>
#include <hrpUtil/Eigen3d.h>
#include <hrpCorba/Controller.hh>
#include <fstream>

class SimScheduler
{
public:
  SimScheduler();
  ~SimScheduler();
  
  void init(int argc,char *argv[]);
  void mainLoop();

protected:
  void loadModels(int argc, char * argv[]);
  void initCorba(int argc, char * argv[]);
  void initOLV(int argc, char * argv[]);
  void initDynamicsSimulator();
  void initController();

  void parseOptions(int argc,char *argv[]);
  void initRobotsFreeFlyerPosition();
  void initRobotsPose();
  void initRobotsJointMode();
  void initParallelMecanisms();
  void initCollisions();

private:

  /// Path to the floor wrl file
  std::string floorfilename_;
  /// Path to the robot wrl file.
  std::string robotfilename_;

  /// BodyInfo representation of the floor.
  OpenHRP::BodyInfo_var floor_;
  /// BodyInfo representation of the robot.
  OpenHRP::BodyInfo_var body_;

  /// Pointer to the OLV server.
  OpenHRP::OnlineViewer_var olv_;

  /// Controller
  OpenHRP::Controller_var controller_;
  bool controller_found_=false;
  std::string controllerName_;

  /// Pointer to the dynamics simulator server
  OpenHRP::DynamicsSimulator_var dynamicsSimulator_;

  // Corba context
  CosNaming::NamingContext_var cxt_;

  double timeStep_;  // (s)
  double controlTimeStep_;
  double EndTime_;
  
};
#endif /* _SIM_SCHEDULER_HH_ */
