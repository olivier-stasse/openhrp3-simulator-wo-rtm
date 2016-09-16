#ifndef _SCHEDULER_PROJECT_HH_
#define _SCHEDULER_PROJECT_HH_

#include <fstream>
#include <sstream>
#include <hrpUtil/OnlineViewerUtil.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpCorba/DynamicsSimulator.hh>
#include <hrpUtil/Eigen3d.h>
#include <hrpCorba/Controller.hh>

#include <libxml/parser.h>
#include <libxml/tree.h>

#include <loadproject.hh>
#include <logproject.hh>

struct ControllerModelItem
{
  OpenHRP::Controller_var controller;
  bool controller_found;
  std::string controllerName;
  
  ControllerModelItem():
    controller_found(false),
    controllerName("")
  {};

};

class SchedulerProject
{
public:
  SchedulerProject();
  ~SchedulerProject();
  
  void init(int argc,char *argv[]);
  void mainLoop();

protected:
  void loadModels(int argc, char * argv[]);
  void initCorba(int argc, char * argv[]);
  void initOLV(int argc, char * argv[]);
  void initDynamicsSimulator();
  void initController();

  void parseOptions(int argc,char *argv[]);
  //void initRobotsFreeFlyerPosition();
  //void initRobotsPose();
  void initRobotsJointData();
  void initParallelMecanisms();
  void initCollisions();
  void saveLog(double ltime);

  //@{ set joint data methods
  void setBodyMode(JointData &aJointData,
		   //OpenHRP::BodyInfo_var  aBodyInfo,
		   std::string &CharacterName,
		   CORBA::String_var CORBAbodyName);

  void setBodyVelocity(JointData &aJointData,
		       //OpenHRP::BodyInfo_var  aBodyInfo,
		       std::string &CharacterName,
		       CORBA::String_var CORBAbodyName);
  void setBodyAngle(JointData &aJointData,
		    //OpenHRP::BodyInfo_var  aBodyInfo,
		    std::string &CharacterName,
		    CORBA::String_var CORBAbodyName);
  void setBodyAbsolutePosition(JointData &aJointData,
			       //OpenHRP::BodyInfo_var  aBodyInfo,
			       std::string &CharacterName,
			       CORBA::String_var CORBAbodyName);
  

  //@}
private:

  /// Path to the project xml file
  std::string projectfilename_;

  /// Pointer to the OLV server.
  OpenHRP::OnlineViewer_var olv_;

  /// Controller
  std::vector<ControllerModelItem> aListOfControllers_;

  /// Pointer to the dynamics simulator server
  OpenHRP::DynamicsSimulator_var dynamicsSimulator_;

  // Corba context
  CosNaming::NamingContext_var cxt_;

  /// Time interval between each computation of the control.
  double controlTimeStep_;

  /// Simulation information are stored in Simulation Item.
  std::shared_ptr<SimulationItem> simulationData_;

  /// List of model items loaded from the project file.
  std::shared_ptr<std::vector<ModelItem> > aListOfModelItems_;
  
  /// List of collision pairs item loaded from the project file.
  std::shared_ptr<std::vector<CollisionPairItem> > aListOfCollisionPairItems_;

  // List of log files.
  std::vector<LogProject> listOfLogs_;

  /// Object to load projects
  LoadProject aProjectLoader_;
};
#endif /* _SIM_SCHEDULER_HH_ */
