#include <iostream>
#include "sample-robot-o2sr.hh"

SampleRobotO2SR::SampleRobotO2SR()
{
  nb_dofs_ = 30;
  P_.resize(nb_dofs_);
  I_.resize(nb_dofs_);
  D_.resize(nb_dofs_);

  q_.length(nb_dofs_);
  qref_.length(nb_dofs_);
  dq_.length(nb_dofs_);
  dqref_.length(nb_dofs_);
  torque_.length(nb_dofs_);
  
  sendto_.length(1);
  recvfrom_.length(1);

  perform_control_ = true;
}

SampleRobotO2SR::~SampleRobotO2SR()
{

}

void SampleRobotO2SR::initref()
{
  qref_[0] = 0.0; dqref_[0] = 0.0;
  qref_[1] = 0.0; dqref_[1] = 0.0;
  qref_[2] = 0.0; dqref_[2] = 0.0;
  qref_[3] = 0.0; dqref_[3] = 0.0;
  qref_[4] = 0.0; dqref_[4] = 0.0; 

  qref_[5] = 0.0; dqref_[5] = 0.0;
  qref_[6] = 0.0; dqref_[6] = 0.0;
  qref_[7] = 0.0; dqref_[7] = 0.0;
  qref_[8] = 0.0; dqref_[8] = 0.0;
  qref_[9] = 0.0; dqref_[9] = 0.0;

  qref_[10] = 0.0; dqref_[10] = 0.0;
  qref_[11] = 0.0; dqref_[11] = 0.0;
  qref_[12] = 0.0; dqref_[12] = 0.0;
  qref_[13] = 0.0; dqref_[13] = 0.0;
  qref_[14] = 0.0; dqref_[14] = 0.0;

  qref_[15] = 0.0; dqref_[15] = 0.0;
  qref_[16] = 0.0; dqref_[16] = 0.0;
  qref_[17] = 0.0; dqref_[17] = 0.0;
  qref_[18] = 0.0; dqref_[18] = 0.0;
  qref_[19] = 0.0; dqref_[19] = 0.0;

  qref_[20] = 0.0; dqref_[20] = 0.0;
  qref_[21] = 0.0; dqref_[21] = 0.0;
  qref_[22] = 0.0; dqref_[22] = 0.0;
  qref_[23] = 0.0; dqref_[23] = 0.0;
  qref_[24] = 0.0; dqref_[24] = 0.0;

  qref_[25] = 0.0; dqref_[25] = 0.0;
  qref_[26] = 0.0; dqref_[26] = 0.0;
  qref_[27] = 0.0; dqref_[27] = 0.0;
  qref_[28] = 0.0; dqref_[28] = 0.0;
  qref_[29] = 0.0; dqref_[29] = 0.0;
  

}

void SampleRobotO2SR::initpids()
{
  P_[ 0]=8000; D_[ 0] = 100;
  P_[ 1]=8000; D_[ 1] = 100;
  P_[ 2]=8000; D_[ 2] = 100;
  P_[ 3]=8000; D_[ 3] = 100;
  P_[ 4]=8000; D_[ 4] = 100;
  P_[ 5]=8000; D_[ 5] = 100;

  P_[ 6]=3000; D_[ 6] = 100;
  P_[ 7]=3000; D_[ 7] = 100;
  P_[ 8]=3000; D_[ 8] = 100;
  P_[ 9]=3000; D_[ 9] = 100;
  P_[10]=3000; D_[10] = 100;
  P_[11]=3000; D_[11] = 100;
  P_[12]=3000; D_[12] = 100;

  P_[13]=8000; D_[13] = 100;
  P_[14]=8000; D_[14] = 100;
  P_[15]=8000; D_[15] = 100;
  P_[16]=8000; D_[16] = 100;
  P_[17]=8000; D_[17] = 100;
  P_[18]=8000; D_[18] = 100;

  P_[19]=3000; D_[19] = 100;
  P_[20]=3000; D_[20] = 100;
  P_[21]=3000; D_[21] = 100;
  P_[22]=3000; D_[22] = 100;  
  P_[23]=3000; D_[23] = 100;
  P_[24]=3000; D_[24] = 100;
  P_[25]=3000; D_[25] = 100;  

  P_[26]=8000; D_[26] = 100;
  P_[27]=8000; D_[27] = 100;
  P_[28]=8000; D_[28] = 100;  
}

void SampleRobotO2SR::initLinkNames()
{
  std::string aLinkName;

  aLinkName = "LLEG_HIP_P";listOfLinks_.push_back(aLinkName);
  aLinkName = "LLEG_HIP_R";listOfLinks_.push_back(aLinkName);
  aLinkName = "LLEG_HIP_Y";listOfLinks_.push_back(aLinkName);
  aLinkName = "LLEG_KNEE";listOfLinks_.push_back(aLinkName);
  aLinkName = "LLEG_ANKLE_P";listOfLinks_.push_back(aLinkName);
  aLinkName = "LLEG_ANKLE_R";listOfLinks_.push_back(aLinkName);

  aLinkName = "LARM_SHOULDER_P";listOfLinks_.push_back(aLinkName);
  aLinkName = "LARM_SHOULDER_R";listOfLinks_.push_back(aLinkName);
  aLinkName = "LARM_SHOULDER_Y";listOfLinks_.push_back(aLinkName);
  aLinkName = "LARM_ELBOW";listOfLinks_.push_back(aLinkName);
  aLinkName = "LARM_WRIST_P";listOfLinks_.push_back(aLinkName);
  aLinkName = "LARM_WRIST_R";listOfLinks_.push_back(aLinkName);
  aLinkName = "LARM_WRIST_Y";listOfLinks_.push_back(aLinkName);

  aLinkName = "RLEG_HIP_P";listOfLinks_.push_back(aLinkName);
  aLinkName = "RLEG_HIP_R";listOfLinks_.push_back(aLinkName);
  aLinkName = "RLEG_HIP_Y";listOfLinks_.push_back(aLinkName);
  aLinkName = "RLEG_KNEE";listOfLinks_.push_back(aLinkName);
  aLinkName = "RLEG_ANKLE_P";listOfLinks_.push_back(aLinkName);
  aLinkName = "RLEG_ANKLE_R";listOfLinks_.push_back(aLinkName);

  aLinkName = "RARM_SHOULDER_P";listOfLinks_.push_back(aLinkName);
  aLinkName = "RARM_SHOULDER_R";listOfLinks_.push_back(aLinkName);
  aLinkName = "RARM_SHOULDER_Y";listOfLinks_.push_back(aLinkName);
  aLinkName = "RARM_ELBOW";listOfLinks_.push_back(aLinkName);
  aLinkName = "RARM_WRIST_P";listOfLinks_.push_back(aLinkName);
  aLinkName = "RARM_WRIST_R";listOfLinks_.push_back(aLinkName);
  aLinkName = "RARM_WRIST_Y";listOfLinks_.push_back(aLinkName);
  
  aLinkName = "CHEST";listOfLinks_.push_back(aLinkName);
  //aLinkName = "WAIST";listOfLinks_.push_back(aLinkName);
  aLinkName = "WAIST_P";listOfLinks_.push_back(aLinkName);
  aLinkName = "WAIST_R";listOfLinks_.push_back(aLinkName);
   
}

void SampleRobotO2SR::initialize()
{
  initref();
  initpids();
  initLinkNames();
}


void SampleRobotO2SR::input()
{
  
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
      linkDataType = OpenHRP::DynamicsSimulator::JOINT_VELOCITY;
      OpenHRP::DblSequence_var var_dq;
      dynamicsSimulator_->getCharacterLinkData(modelName_.c_str(),
					       aLinkName.c_str(),
					       linkDataType,var_dq);
      dq_[i] = var_dq[0];

    }
}

void SampleRobotO2SR::output()
{
  for(unsigned int i=0;i<listOfLinks_.size();i++)
    {
      
      std::string aLinkName = listOfLinks_[i];
      sendto_[0] = torque_[i];
      OpenHRP::DynamicsSimulator::LinkDataType linkDataType = OpenHRP::DynamicsSimulator::JOINT_TORQUE;
      dynamicsSimulator_->setCharacterLinkData(modelName_.c_str(),
					       aLinkName.c_str(),
					       linkDataType,sendto_);
    }
					      
}


void SampleRobotO2SR::control()
{
  if (perform_control_)
    for(unsigned int i=0;i<nb_dofs_;i++)
      {
	torque_[i] = -(q_[i] - qref_[i] ) *P_[i] 
	  - (dq_[i] -dqref_[i]) * D_[i];
      }
}

void SampleRobotO2SR::stop()
{
  perform_control_ = false;
}

void SampleRobotO2SR::start()
{
  perform_control_ = true;
}

void SampleRobotO2SR::shutdown()
{
}

void SampleRobotO2SR::restart()
{
}

void SampleRobotO2SR::set_path_to_library(std::string &)
{
}
