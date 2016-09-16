#ifndef _LOAD_PROJECT_HH_
#define _LOAD_PROJECT_HH_

#include <fstream>
#include <sstream>
#include <hrpUtil/OnlineViewerUtil.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpCorba/DynamicsSimulator.hh>
#include <hrpUtil/Eigen3d.h>
#include <hrpCorba/Controller.hh>

#include <libxml/parser.h>
#include <libxml/tree.h>

typedef OpenHRP::DynamicsSimulator::IntegrateMethod integration_method;

struct SimulationItem
{
  std::string name;
  bool select;
  bool integrate;
  bool viewSimulate;
  double totalTime;
  double timeStep;
  bool realTime;
  double gravity;
  integration_method method;
  std::ostream & display(std::ostream & os) const
  {
    os << "name:" << name << std::endl;
    os << "select:" << select << std::endl;
    os << "integrate:" << integrate << std::endl;
    os << "viewSimulate:" << viewSimulate << std::endl;
    os << "totalTime:" << totalTime << std::endl;
    os << "timeStep:" << timeStep << std::endl;
    os << "realTime:" << realTime << std::endl;
    os << "gravity:" << gravity << std::endl;
    if (method==OpenHRP::DynamicsSimulator::RUNGE_KUTTA) 
      os << "method: RUNGE_KUTTA";
    else if (method==OpenHRP::DynamicsSimulator::EULER) 
      os << "method: EULER";
    os << std::endl;
    return os;
  }
  friend std::ostream & operator<< (std::ostream &os, const SimulationItem &aCP);
};

struct WorldStateItem
{
  double logTimeStep;
};

struct CollisionPairItem
{
  double slidingFriction;
  double staticFriction;
  double cullingThresh;
  bool springDamperModel;
  std::string jointName1;
  std::string jointName2;
  std::string objectName1;
  std::string objectName2;
  std::vector<double> springConstant;
  std::vector<double> damperConstant;
  CollisionPairItem():
    springConstant(0),
    damperConstant(0)
  {};
  std::ostream & display(std::ostream & os) const
  {
    os << "CollisionPairItem ========" << std::endl;
    os << "slidingFriction: "<< slidingFriction << std::endl;
    os << "staticFriction: " << staticFriction << std::endl;
    os << "cullingThresh: " << cullingThresh << std::endl;
    os << "springDamperModel: " << springDamperModel << std::endl;
    os << "jointName1: " << jointName1 << std::endl;
    os << "jointName2: " << jointName2 << std::endl;
    os << "objectName1: " << objectName1 << std::endl;
    os << "objectName2: " << objectName2 << std::endl;
    os << "springConstant ";
    for(unsigned int i=0;i<springConstant.size();i++) 
      os << springConstant[i]<< " ";
    os<< std::endl;
    os << "damperConstant ";
    for(unsigned int i=0;i<damperConstant.size();i++) 
      os << damperConstant[i]<< " ";
    os<< std::endl;
    os << "CollisionPairItem ========" << std::endl;
    return os;
  };
  friend std::ostream & operator<< (std::ostream &os, const CollisionPairItem &aCP);
};


struct PropertyModelItem
{
  std::string name,value;
};


struct JointData
{
  int mode; // 0: High gain, 1: Torque.
  double angle;
  bool angle_set;
  std::vector<double> velocity;
  std::vector<double> angularVelocity;
  std::vector<double> translation;
  std::vector<double> rotation;
  JointData():
    mode(1),angle(0.0),angle_set(false) {};
  void getInitialTransformArray(std::vector<double> &transformArray);
};

struct ModelItem
{
  std::string name;
  bool select;
  std::string url;
  bool isRobot;
  double markRadius;
  std::vector<PropertyModelItem> listOfVariableProperties;
  std::map<std::string,JointData> jointsMap;
  OpenHRP::BodyInfo_var bodyInfoVar; // Model of the item
  std::string controllerName;

  std::ostream & display(std::ostream & os) const
  {
    os << "Name: "<< name << std::endl;
    os << "select: "<< select << std::endl;
    os << "URL: "<< url << std::endl;
    os << "isRobot: " << isRobot << std::endl;
    os << "markRadius: " << markRadius << std::endl;
    os << "ControllerName: " << controllerName << std::endl;
    os << "listOfVariableProperties:" << std::endl;
    for(unsigned int i=0;i<listOfVariableProperties.size();
	i++)
      os << listOfVariableProperties[i].name << ": "
	 << listOfVariableProperties[i].value << std::endl;
    return os;
  };
  void fillBodyValues();
  friend std::ostream & operator<< (std::ostream &os, const ModelItem &aCP);
};

class LoadProject
{
public:
  LoadProject();
  ~LoadProject();
  
  void loadProject(std::string & projectfilename_,
		   std::shared_ptr<SimulationItem> simulationData,
		   std::shared_ptr<std::vector<ModelItem>>  
		   aListOfModelItems,
		   std::shared_ptr<std::vector<CollisionPairItem>>
		   aListOfCollisionPairItems);

protected:
  /// \name XML Parsing for loading a project file 
  /// @{ 
  void AnalyzeCollisionPairItem(xmlNode *aNode);
  void AnalyzeSimulationItem(xmlNode *aNode,
			     std::string &name,
			     std::string &select);
  void AnalyzeModelItem(xmlNode *aNode,
			std::string &name,
			std::string &select,
			std::string &url);

  void AnalyzeElementNames(xmlNode *aNode, int depth);
  void AnalyzeItem(xmlNode *aNode);
  void AnalyzeAttr(xmlAttr *an_Attr, int depth);
  /// @}

private:

  /// Path to the project xml file
  std::string projectfilename_;

  /// Simulation information are stored in Simulation Item.
  std::shared_ptr<SimulationItem> simulationData_;

  /// List of model items loaded from the project file.
  std::shared_ptr<std::vector<ModelItem>> aListOfModelItems_;
  
  /// List of collision pairs item loaded from the project file.
  std::shared_ptr<std::vector<CollisionPairItem>> aListOfCollisionPairItems_;
};
#endif /* _SIM_SCHEDULER_HH_ */
