#include <cmath>
#include <cstring>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <fstream>

#include <hrpModel/BodyCustomizerInterface.h>

#include <iostream>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define DLL_EXPORT __declspec(dllexport)
#else 
#define DLL_EXPORT 
#endif /* Windows */

#include <hrpUtil/EigenTypes.h>
#define NS_HRPMODEL hrp

// #ifndef NS_HRPMODEL
// #define NS_HRPMODEL OpenHRP
// typedef OpenHRP::vector3 Vector3;
// typedef OpenHRP::matrix33 Matrix33;
// #endif

using namespace std;
using namespace boost;
using namespace NS_HRPMODEL;

static BodyInterface* bodyInterface = 0;

static BodyCustomizerInterface aBodyCustomizerInterface;

struct JointValSet
{
  double* valuePtr;
  double* velocityPtr;
  double* torqueForcePtr;
};

struct BushCustomizerParam
{
  JointValSet jointValSets;
  std::string name;
  // Spring coefficient for bush. For slide joint, [N/m], for rotate joint, [Nm/rad]
  double spring;
  // Damping coefficient for bush. For slide joint, [N/(m/s)], for rotate joint, [Nm/(rad/s)]
  double damping;
  int index;
};

struct BushCustomizer
{
  BodyHandle bodyHandle;
  bool hasVirtualBushJoints;
  std::vector<BushCustomizerParam> params;
};

// Parameters should be specified by configure file using BUSH_CUSTOMIZER_CONFIG_PATH environment.

// Robot model name using bush
static std::string robot_model_name;

// Bush configuration parameters such as:
//   bush_config: joint1,spring1,damping1 joint2,spring2,damping2 joint3,spring3,damping3 ...
//   joint* should be included in VRML file.
struct BushConfigurationParam
{
  std::string name;
  std::string spring;
  std::string damping;
};
static std::vector<BushConfigurationParam> bush_config;

static const char** getTargetModelNames()
{
  static const char* names[] = {
    robot_model_name.c_str(),
    0 };
  return names;
}

static void getVirtualbushJoints(BushCustomizer* customizer, BodyHandle body)
{
  std::cerr << "[Bush customizer] Bush params" << std::endl;
  customizer->hasVirtualBushJoints = true;
  for (size_t i = 0; i < bush_config.size(); i++) {
    // tmp_config <= temp bush config, e.g., "joint*,spring*,damping*"
    int bushIndex = bodyInterface->getLinkIndexFromName(body, bush_config[i].name.c_str());
    if(bushIndex < 0){
      std::cerr << "[Bush customizer]   No such joint name (" << bush_config[i].name << ")" << std::endl;
      customizer->hasVirtualBushJoints = false;
    } else {
      BushCustomizerParam p;
      p.index = bushIndex;
      p.name = bush_config[i].name;
      p.spring = atof(bush_config[i].spring.c_str());
      p.damping= atof(bush_config[i].damping.c_str());
      p.jointValSets.valuePtr = bodyInterface->getJointValuePtr(body, bushIndex);
      p.jointValSets.velocityPtr = bodyInterface->getJointVelocityPtr(body, bushIndex);
      p.jointValSets.torqueForcePtr = bodyInterface->getJointForcePtr(body, bushIndex);
      customizer->params.push_back(p);
      std::cerr << "[Bush customizer]   name = " << p.name << ", index = " << p.index << ", spring = " << p.spring << ", damping = " << p.damping << std::endl;
    }
  }
}

static BodyCustomizerHandle create(BodyHandle bodyHandle, const char* modelName)
{
  BushCustomizer* customizer = 0;

  std::cerr << "[Bush customizer] Create " << std::string(modelName) << std::endl;
  customizer = new BushCustomizer;

  customizer->bodyHandle = bodyHandle;
  //customizer->hasVirtualBushJoints = false;
  // customizer->springT  = 5.0e5; // N/m
  // customizer->dampingT = 1.0e3; // N/(m/s)
  // customizer->springR  = 1e3; // Nm / rad
  // customizer->dampingR = 2.5e1;   // Nm / (rad/s)

  getVirtualbushJoints(customizer, bodyHandle);

  return static_cast<BodyCustomizerHandle>(customizer);
}


static void destroy(BodyCustomizerHandle customizerHandle)
{
  BushCustomizer* customizer = static_cast<BushCustomizer*>(customizerHandle);
  if(customizer){
    delete customizer;
  }
}

static void setVirtualJointForces(BodyCustomizerHandle customizerHandle)
{
  std::cout << "bush_customizer going through setVirtualJointForces " 
	    << std::endl;
  BushCustomizer* customizer = static_cast<BushCustomizer*>(customizerHandle);

  std::cout << customizer->hasVirtualBushJoints << " "
	    << customizer->params.size() 
	    << std::endl;

  if(customizer->hasVirtualBushJoints){
    for(int i=0; i < customizer->params.size(); ++i){
      BushCustomizerParam& param = customizer->params[i];
      *(param.jointValSets.torqueForcePtr) = - param.spring * (*param.jointValSets.valuePtr) - param.damping * (*param.jointValSets.velocityPtr);
      std::cout << "Bush " << i << " " << 0 << " " 
		<< *(param.jointValSets.torqueForcePtr) << " = " 
		<< *(param.jointValSets.valuePtr) << " + " 
		<< *(param.jointValSets.velocityPtr) 
		<< " ( " << param.spring << " , " << param.damping << " )"
		<< std::endl;
    }
  }
}

extern "C" DLL_EXPORT
NS_HRPMODEL::BodyCustomizerInterface* getHrpBodyCustomizerInterface(NS_HRPMODEL::BodyInterface* bodyInterface_)
{
  bodyInterface = bodyInterface_;
  char* tmpenv = getenv("BUSH_CUSTOMIZER_CONFIG_PATH");
  if (tmpenv) {
    std::ifstream ifs(tmpenv);
    if (ifs.fail() ) {
      std::cerr << "[Bush customizer] Could not open [" << tmpenv << "]" << std::endl;
    }
    else 
    {
      std::string tmpstr;
      std::cerr << "[Bush customizer] Open [" << tmpenv << "]" << std::endl;
      ifs >> robot_model_name ;
      std::cerr << "[Bush customizer]   robot_model_name = [" << robot_model_name << "]" << std::endl;
      
      while(!ifs.eof())
	{
	  struct BushConfigurationParam abush_config;
	  ifs >> abush_config.name;
	  ifs >> abush_config.spring;
	  ifs >> abush_config.damping;
	  if (abush_config.name.size()>0)
	    {
	      bush_config.push_back(abush_config);
	      std::cerr << "[Bush customizer]   bush_config = [" << abush_config.name<< "]" << std::endl;
	    }
        }
    }

  }

  aBodyCustomizerInterface.version = NS_HRPMODEL::BODY_CUSTOMIZER_INTERFACE_VERSION;
  aBodyCustomizerInterface.getTargetModelNames = getTargetModelNames;
  aBodyCustomizerInterface.create = create;
  aBodyCustomizerInterface.destroy = destroy;
  aBodyCustomizerInterface.initializeAnalyticIk = NULL;
  aBodyCustomizerInterface.calcAnalyticIk = NULL;
  aBodyCustomizerInterface.setVirtualJointForces = setVirtualJointForces;

  return &aBodyCustomizerInterface;
}
