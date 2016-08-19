#include <iostream>
#include <fstream>
#include <iomanip>
#include <dlfcn.h>

#include "hrp2-oh2sot.hh"

#if DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

#define DBGFILE "/tmp/hrp2oh2sot.dat"

#define RESETDEBUG5() { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::out);		\
    DebugFile.close();}
#define ODEBUG5FULL(x) { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::app);		\
    DebugFile << __FILE__ << ":"			\
	      << __FUNCTION__ << "(#"			\
	      << __LINE__ << "):" << x << std::endl;	\
    DebugFile.close();}
#define ODEBUG5(x) { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::app); \
    DebugFile << x << std::endl;		\
    DebugFile.close();}



HRP2OH2SOT::HRP2OH2SOT()
{

  SotLoaderBasic();

  robot_config_.nb_dofs =40;
  nb_dofs_ = 40;
  P_.resize(nb_dofs_);
  I_.resize(nb_dofs_);
  D_.resize(nb_dofs_);

  q_.resize(nb_dofs_);
  qref_.resize(nb_dofs_);
  pqref_.resize(nb_dofs_);
  dq_.resize(nb_dofs_);
  dqref_.resize(nb_dofs_);
  pdqref_.resize(nb_dofs_);
  ddqref_.resize(nb_dofs_);
  torques_.resize(nb_dofs_);
  
  sendto_.length(1);
  recvfrom_.length(1);

  perform_control_ = false;
  
  control_ticks_ = 0;
  control_ticks_max_ = 5;
  dt_ = 0.005;
  RESETDEBUG5();

  cumul_err_ = 0.0;

  std::ofstream LogFile;
  LogFile.open("/tmp/angle.dat",std::ofstream::out);
  LogFile.close();
}

HRP2OH2SOT::~HRP2OH2SOT()
{

}

void HRP2OH2SOT::initref()
{
  qref_[0] =  0.0    ; dqref_[0] = 0.0;
  qref_[1] =  0.0    ; dqref_[1] = 0.0;
  qref_[2] = -0.45379; dqref_[2] = 0.0;
  qref_[3] =  0.87266; dqref_[3] = 0.0;
  qref_[4] = -0.41888; dqref_[4] = 0.0; 
  qref_[5] =  0.0    ; dqref_[5] = 0.0;

  qref_[6] =  0.0    ; dqref_[6] = 0.0;
  qref_[7] =  0.0    ; dqref_[7] = 0.0;
  qref_[8] = -0.45379; dqref_[8] = 0.0;
  qref_[9] =  0.87266; dqref_[9] = 0.0;
  qref_[10] =-0.41888; dqref_[10] = 0.0;
  qref_[11] = 0.0    ; dqref_[11] = 0.0;

  qref_[12] = 0.0    ; dqref_[12] = 0.0;
  qref_[13] = 0.0    ; dqref_[13] = 0.0;

  qref_[14] = 0.0    ; dqref_[14] = 0.0;
  qref_[15] = 0.0    ; dqref_[15] = 0.0;

  qref_[16] = 0.2618 ; dqref_[16] = 0.0;
  qref_[17] =-0.17453; dqref_[17] = 0.0;
  qref_[18] = 0.0    ; dqref_[18] = 0.0;
  qref_[19] =-0.5236 ; dqref_[19] = 0.0;
  qref_[20] = 0.0    ; dqref_[20] = 0.0;
  qref_[21] = 0.0    ; dqref_[21] = 0.0;
  qref_[22] = 0.17453; dqref_[22] = 0.0;

  qref_[23] = 0.2618 ; dqref_[23] = 0.0;
  qref_[24] = 0.17453; dqref_[24] = 0.0;
  qref_[25] = 0.0    ; dqref_[25] = 0.0;
  qref_[26] =-0.5236 ; dqref_[26] = 0.0;
  qref_[27] = 0.0    ; dqref_[27] = 0.0;
  qref_[28] = 0.0    ; dqref_[28] = 0.0;
  qref_[29] = 0.17453; dqref_[29] = 0.0;

  qref_[30] = -0.17453; dqref_[30] = 0.0;
  qref_[31] = 0.17453; dqref_[31] = 0.0;
  qref_[32] = -0.17453; dqref_[32] = 0.0;
  qref_[33] = 0.17453; dqref_[33] = 0.0;
  qref_[34] = -0.17453; dqref_[34] = 0.0;

  qref_[35] = -0.17453; dqref_[35] = 0.0;
  qref_[36] = 0.17453; dqref_[36] = 0.0;
  qref_[37] = -0.17453; dqref_[37] = 0.0;
  qref_[38] = 0.17453; dqref_[38] = 0.0;
  qref_[39] = -0.17453; dqref_[39] = 0.0;
    
  
  for(unsigned int i=0;i<40;i++)
    { 
      pqref_[i] = qref_[i];
      dqref_[i] = 0.0; 
      pdqref_[i] = dqref_[i];
      ddqref_[i] = 0.0; 
    }
  
}

void HRP2OH2SOT::initpids()
{
  
  std::ifstream ifs;
  ifs.open(robot_config_.pid_gains_filename.c_str());

  if (!ifs.is_open())
    return;
  
  //  Read PIDs.
  for(int i=0;i<40;i++)
    {
      ifs >> D_[i];
      ifs >> I_[i];
      ifs >> P_[i];
      std::cout << i << " - " << D_[i] << " " << I_[i] << " " << P_[i] << std::endl;
    }
  ifs.close();
}

void HRP2OH2SOT::initLinkNames()
{
  std::string aLinkName;
  
  aLinkName = "RLEG_JOINT0";listOfLinks_.push_back(aLinkName);
  aLinkName = "RLEG_JOINT1";listOfLinks_.push_back(aLinkName);
  aLinkName = "RLEG_JOINT2";listOfLinks_.push_back(aLinkName);
  aLinkName = "RLEG_JOINT3";listOfLinks_.push_back(aLinkName);
  aLinkName = "RLEG_JOINT4";listOfLinks_.push_back(aLinkName);
  aLinkName = "RLEG_JOINT5";listOfLinks_.push_back(aLinkName);

  aLinkName = "LLEG_JOINT0";listOfLinks_.push_back(aLinkName);
  aLinkName = "LLEG_JOINT1";listOfLinks_.push_back(aLinkName);
  aLinkName = "LLEG_JOINT2";listOfLinks_.push_back(aLinkName);
  aLinkName = "LLEG_JOINT3";listOfLinks_.push_back(aLinkName);
  aLinkName = "LLEG_JOINT4";listOfLinks_.push_back(aLinkName);
  aLinkName = "LLEG_JOINT5";listOfLinks_.push_back(aLinkName);
  
  aLinkName = "CHEST_JOINT0";listOfLinks_.push_back(aLinkName);
  aLinkName = "CHEST_JOINT1";listOfLinks_.push_back(aLinkName);

  aLinkName = "HEAD_JOINT0";listOfLinks_.push_back(aLinkName);
  aLinkName = "HEAD_JOINT1";listOfLinks_.push_back(aLinkName);

  aLinkName = "RARM_JOINT0";listOfLinks_.push_back(aLinkName);
  aLinkName = "RARM_JOINT1";listOfLinks_.push_back(aLinkName);
  aLinkName = "RARM_JOINT2";listOfLinks_.push_back(aLinkName);
  aLinkName = "RARM_JOINT3";listOfLinks_.push_back(aLinkName);
  aLinkName = "RARM_JOINT4";listOfLinks_.push_back(aLinkName);
  aLinkName = "RARM_JOINT5";listOfLinks_.push_back(aLinkName);
  aLinkName = "RARM_JOINT6";listOfLinks_.push_back(aLinkName);

  aLinkName = "LARM_JOINT0";listOfLinks_.push_back(aLinkName);
  aLinkName = "LARM_JOINT1";listOfLinks_.push_back(aLinkName);
  aLinkName = "LARM_JOINT2";listOfLinks_.push_back(aLinkName);
  aLinkName = "LARM_JOINT3";listOfLinks_.push_back(aLinkName);
  aLinkName = "LARM_JOINT4";listOfLinks_.push_back(aLinkName);
  aLinkName = "LARM_JOINT5";listOfLinks_.push_back(aLinkName);
  aLinkName = "LARM_JOINT6";listOfLinks_.push_back(aLinkName);

  aLinkName = "RHAND_JOINT0";listOfLinks_.push_back(aLinkName);
  aLinkName = "RHAND_JOINT1";listOfLinks_.push_back(aLinkName);
  aLinkName = "RHAND_JOINT2";listOfLinks_.push_back(aLinkName);
  aLinkName = "RHAND_JOINT3";listOfLinks_.push_back(aLinkName);
  aLinkName = "RHAND_JOINT4";listOfLinks_.push_back(aLinkName);

  aLinkName = "LHAND_JOINT0";listOfLinks_.push_back(aLinkName);
  aLinkName = "LHAND_JOINT1";listOfLinks_.push_back(aLinkName);
  aLinkName = "LHAND_JOINT2";listOfLinks_.push_back(aLinkName);
  aLinkName = "LHAND_JOINT3";listOfLinks_.push_back(aLinkName);
  aLinkName = "LHAND_JOINT4";listOfLinks_.push_back(aLinkName);
   
}


void HRP2OH2SOT::initializeSoT()
{
  char * sLD_LIBRARY_PATH;
  sLD_LIBRARY_PATH=getenv("LD_LIBRARY_PATH");
  ODEBUG5("LoadSot - Start " << sLD_LIBRARY_PATH);
  char * sPYTHONPATH;
  sPYTHONPATH=getenv("PYTHONPATH");
  ODEBUG5("PYTHONPATH:" << sPYTHONPATH );
  sPYTHONPATH=getenv("PYTHON_PATH");
  ODEBUG5("PYTHON_PATH:" << sPYTHONPATH);

  ODEBUG5("Trying to load the library:" << robot_config_.libname);
  // Load the SotHRP2Controller library.
  void * SotHRP2ControllerLibrary = dlopen(robot_config_.libname.c_str(),
                                           RTLD_GLOBAL | RTLD_NOW);
  if (!SotHRP2ControllerLibrary) {
    ODEBUG5("Cannot load library: " << dlerror() );
    return ;
  }
  ODEBUG5("Success in loading the library:" << robot_config_.libname);
  // reset errors
  dlerror();
  
  // Load the symbols.
  createSotExternalInterface_t * createHRP2Controller =
    (createSotExternalInterface_t *) dlsym(SotHRP2ControllerLibrary, 
                                           "createSotExternalInterface");
  ODEBUG5("createHRPController call "<< std::hex
          << std::setbase(10));
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    ODEBUG5("Cannot load symbol create: " << dlsym_error );
    return ;
  }
  ODEBUG5("Success in getting the controller factory");
  
  // Create hrp2-controller
  try 
    {
      ODEBUG5("exception handled createHRP2Controller call "<< std::hex 
              << std::setbase(10));
      m_sotController = createHRP2Controller();
      ODEBUG5("After createHRP2Controller.");

    } 
  catch (std::exception &e)
    {
      ODEBUG5("Exception: " << e.what());
    }
  ODEBUG5("LoadSot - End");
}


void HRP2OH2SOT::initialize()
{
  int argc=1;
  char * argv[1];
  argv[0] = "hrp2ohsot";

  initref();
  initpids();
  initLinkNames();
  initializeSoT();
  SotLoaderBasic::initializeRosNode(argc,argv);
}


void HRP2OH2SOT::input()
{
  
  std::ofstream LogFile;

  LogFile.open("/tmp/angle.dat",std::ofstream::app);

  for(unsigned int i=0;i<listOfLinks_.size();i++)
    {
      std::string aLinkName = listOfLinks_[i];
      OpenHRP::DynamicsSimulator::LinkDataType linkDataType =
	OpenHRP::DynamicsSimulator::JOINT_VALUE;
      OpenHRP::DblSequence_var var_q;
      dynamicsSimulator_->getCharacterLinkData(modelName_.c_str(),
					       aLinkName.c_str(),
					       linkDataType,var_q);
      q_[i] = var_q[0];
      LogFile << q_[i] << " ";
      linkDataType = OpenHRP::DynamicsSimulator::JOINT_VELOCITY;
      OpenHRP::DblSequence_var var_dq;
      dynamicsSimulator_->getCharacterLinkData(modelName_.c_str(),
					       aLinkName.c_str(),
					       linkDataType,var_dq);
      dq_[i] = var_dq[0];

    }
  LogFile << std::endl;
  LogFile.close();
  fillSensors(sensorsIn_);
}

void HRP2OH2SOT::output()
{
  if (0)
    {
      for(unsigned int i=0;i<listOfLinks_.size();i++)
	{
	  std::string aLinkName = listOfLinks_[i];
	  sendto_[0] = torques_[i];
	  OpenHRP::DynamicsSimulator::LinkDataType linkDataType = OpenHRP::DynamicsSimulator::JOINT_TORQUE;
	  dynamicsSimulator_->setCharacterLinkData(modelName_.c_str(),
						   aLinkName.c_str(),
						   linkDataType,sendto_);
	}					      
    }
  else
    {
      for(unsigned int i=0;i<listOfLinks_.size();i++)
	{
	  std::string aLinkName = listOfLinks_[i];
	  sendto_[0] = qref_[i];
	  OpenHRP::DynamicsSimulator::LinkDataType linkDataType = OpenHRP::DynamicsSimulator::JOINT_VALUE;
	  dynamicsSimulator_->setCharacterLinkData(modelName_.c_str(),
						   aLinkName.c_str(),
						   linkDataType,sendto_);
	  sendto_[0] = dqref_[i];
	  linkDataType = OpenHRP::DynamicsSimulator::JOINT_VELOCITY;
	  dynamicsSimulator_->setCharacterLinkData(modelName_.c_str(),
						   aLinkName.c_str(),
						   linkDataType,sendto_);

	  sendto_[0] = ddqref_[i];
	  linkDataType = OpenHRP::DynamicsSimulator::JOINT_ACCELERATION;
	  dynamicsSimulator_->setCharacterLinkData(modelName_.c_str(),
						   aLinkName.c_str(),
						   linkDataType,sendto_);

	}					      
    }
}


void HRP2OH2SOT::control()
{
  /* std::cout << "perform_control_ :" << perform_control_ 
	    << "dynamic_graph_stopped_ :" << dynamic_graph_stopped_ 
	    << std::endl;
  */
  if (perform_control_ & !dynamic_graph_stopped_)
    {
      if (control_ticks_==0)
	{
	  try
	    {
	      m_sotController->setupSetSensors(sensorsIn_);
	      m_sotController->getControl(controlValues_);
	    } 
	  catch (std::exception &e) 
	    {  ODEBUG5("Exception on Execute: " << e.what());throw e; }
	  readControl(controlValues_);
	}
      control_ticks_++;
      if (control_ticks_==control_ticks_max_)
	control_ticks_ = 0;
    }

  for(unsigned int i=0;i<nb_dofs_;i++)
    {
      
      //std::cout << i << " : (" << q_[i] << " , " << qref_[i]  << ")" << std::endl;
      double error = q_[i] - qref_[i];
      cumul_err_ -= error;
      torques_[i] = -(q_[i] - qref_[i] ) *P_[i] 
	+ cumul_err_ * I_[i]
	- (dq_[i] -dqref_[i]) * D_[i];
      //      torques_[i] =0.0;
    }
}

void HRP2OH2SOT::stop()
{
  perform_control_ = false;
}

void HRP2OH2SOT::start()
{
  perform_control_ = true;
}

void HRP2OH2SOT::shutdown()
{
}

void HRP2OH2SOT::restart()
{
}


void 
HRP2OH2SOT::fillSensors(std::map<std::string,dgsot::SensorValues> & 
			sensorsIn)
{
  OpenHRP::SensorState_var aSensorState;
  dynamicsSimulator_->getCharacterSensorState(modelName_.c_str(),aSensorState);

  // Fill in the positions values.
  sensorsIn["joints"].setValues(q_);

  // Update forces
  if (aSensorState->force.length()==4)
    {
      sensorsIn["forces"].setName("force");
      
      forces_.resize(4*6);
      for(unsigned int i=0;i<4;i++)
	{
	  for(unsigned int j=0;j<6;j++)
	    {
	      forces_[i*6+j] = aSensorState->force[i][j];
	    }
	}
      
      sensorsIn["forces"].setValues(forces_);
    }

  // Update torque
  sensorsIn["torques"].setName("torque");
  if (aSensorState->u.length()!=0)
    {
      torques_.resize(aSensorState->u.length());
      for (unsigned int j = 0; j < aSensorState->u.length(); ++j)
        torques_[j] = aSensorState->u[j];
    }
  else 
    {
      torques_.resize(robot_config_.nb_dofs);
      for(unsigned int i=0;i<(unsigned int)robot_config_.nb_dofs;i++)
        torques_[i] = 0.0;
    }
  sensorsIn["torques"].setValues(torques_);

  // Update attitude
  // sensorsIn["attitude"].setName("attitude");
  // if (aSensorState.rateGyro.length()!=0)
  //   {
  //     baseAtt_.resize(aSensorState.rateGyro.length());
  //     for (unsigned int j = 0; j < aSensorState.rateGyro.length(); ++j)
  //       baseAtt_ [j] = aSensorState.rateGyro[j];
  //   }
  // else
  //   {
  //     baseAtt_.resize(9);
  //     for(unsigned int i=0;i<3;i++)
  //       {
  //         for(unsigned int j=0;j<3;j++)
  //           {
  //             if (i==j)
  //               baseAtt_[i*3+j]=1.0;
  //             else 
  //               baseAtt_[i*3+j]=0.0;
  //           }
  //       }
  //   }
  // sensorsIn["attitude"].setValues (baseAtt_);

  // Update accelerometer
  sensorsIn["accelerometer_0"].setName("accelerometer_0");
  if (aSensorState->accel.length()!=0)
    {
      accelerometer_.resize(3);
      for (unsigned int j = 0; j <3; ++j)
        accelerometer_[j] = aSensorState->accel[0][j];
    }
  else 
    {
      accelerometer_.resize(3);
      for(unsigned int i=0;i<3;i++)
        accelerometer_[i] = 0.0;
    }  
  sensorsIn["accelerometer_0"].setValues(accelerometer_);
  
  // Update gyrometer
  sensorsIn["gyrometer_0"].setName("gyrometer_0");
  if (aSensorState->rateGyro.length()!=0)
    {
      gyrometer_.resize(3);
      for (unsigned int j = 0; j < 3; ++j)
        gyrometer_[j] = aSensorState->rateGyro[0][j];
    }
  else 
    {
      gyrometer_.resize(3);
      for(unsigned int i=0;i<3;i++)
        gyrometer_[i] = 0.0;
    }  
  sensorsIn["gyrometer_0"].setValues(gyrometer_);
}

void 
HRP2OH2SOT::readControl(std::map<std::string,dgsot::ControlValues> 
			&controlValues)
{
  double R[9];

  // Update joint values.
  angleControl_ = controlValues["joints"].getValues();
  
  for(unsigned int i=0;i<angleControl_.size();i++)
    { 
      // Update qref from SoT.
      // dqref and ddref should/could be provided by SoT too.
      qref_[i] = angleControl_[i]; 
      ODEBUG("m_qRef["<<i<<"]=" << qRef_[i]);
      
      // Compute speed and acceleration by finite differences.
      dqref_[i] = (qref_[i] - pqref_[i])/dt_;
      ddqref_[i] = (dqref_[i]- pdqref_[i])/dt_;
      
      // Store previous values.
      pqref_[i] = qref_[i];
      pdqref_[i] = dqref_[i];
    }
  if (angleControl_.size()<(unsigned int)robot_config_.nb_dofs)
    {
      for(long unsigned int i=angleControl_.size();
          i<(long unsigned int)robot_config_.nb_dofs
            ;i++)
        {
          qref_[i] = 0.0;
	  dqref_[i] = 0.0;
	  ddqref_[i] = 0.0;
          ODEBUG("m_qRef["<<i<<"]=" << m_qRef.data[i]);
        }
    }
}


void HRP2OH2SOT::set_path_to_library(std::string &aLibName)
{
  robot_config_.libname = aLibName;
}

void HRP2OH2SOT::set_path_to_pid_gains(std::string &aFileName)
{
  robot_config_.pid_gains_filename = aFileName;
}

