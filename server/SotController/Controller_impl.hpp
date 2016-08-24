/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 * General Robotix Inc. 
 */
/**
   \file
   \author Shin'ichiro Nakaoka
   \author Olivier Stasse
*/


#include <string>
#include <iostream>

#include <hrpCorba/ORBwrap.h>


using namespace std;

namespace {
  const bool CONTROLLER_BRIDGE_DEBUG = false;
}


template <typename T>
Controller_impl<T>::Controller_impl(CORBA::ORB_ptr )
  :   modelName(""),
      bRestart(false)
{
  if(CONTROLLER_BRIDGE_DEBUG){
    cout << "Controller_impl::Controller_impl" << endl;
  }
}


template <typename T>
Controller_impl<T>::~Controller_impl()
{
  if(CONTROLLER_BRIDGE_DEBUG){
    cout << "Controller_impl::~Controller_impl" << endl;
  }
}

template <class T>
void Controller_impl<T>::
setDynamicsSimulator(DynamicsSimulator_ptr dynamicsSimulator)
{
  this->dynamicsSimulator = DynamicsSimulator::_duplicate(dynamicsSimulator);
  RobotController_.dynamicsSimulator_ = 
    DynamicsSimulator::_duplicate(dynamicsSimulator);
}

template <class T>
void Controller_impl<T>::
setViewSimulator(ViewSimulator_ptr viewSimulator)
{
  this->viewSimulator = ViewSimulator::_duplicate(viewSimulator);
}

template <class T>
void Controller_impl<T>::
start()
{
  if(CONTROLLER_BRIDGE_DEBUG){
    cout << "Controller_impl::start" << endl;
  }

  controlTime = 0.0;
  try{
    if( bRestart ){
      restart();
    } else {
      if(!CORBA::is_nil(viewSimulator)) {
	viewSimulator->getCameraSequenceOf(modelName.c_str(), cameras);
      }else{
	cameras = new CameraSequence(0);
      }
    }
  } catch(CORBA_SystemException& ex){
    cerr << ex._rep_id() << endl;
    cerr << "exception in Controller_impl::start" << endl;
  } catch(...){
    cerr << "unknown exception in Controller_impl::start()" <<  endl;
  }
  RobotController_.start();
}

template <class T>
SensorState& Controller_impl<T>::
getCurrentSensorState()
{
  if(!sensorStateUpdated){
    dynamicsSimulator->getCharacterSensorState(modelName.c_str(), sensorState);
    sensorStateUpdated = true;
  }

  return sensorState;
}

template <class T>
DblSequence* Controller_impl<T>::getLinkDataFromSimulator
(const std::string& linkName, DynamicsSimulator::LinkDataType linkDataType)
{
  DblSequence_var data;
  dynamicsSimulator->getCharacterLinkData(modelName.c_str(), linkName.c_str(), linkDataType, data.out());
  return data._retn();
}


template <class T>
DblSequence* Controller_impl<T>::
getSensorDataFromSimulator(const std::string& sensorName)
{
  DblSequence_var data;
  dynamicsSimulator->getCharacterSensorValues(modelName.c_str(), sensorName.c_str(), data.out());
  return data._retn();
}

template <class T>
ImageData* Controller_impl<T>::
getCameraImageFromSimulator(int cameraId)
{
  if(cameras->length()!=0){
    ImageData_var imageData = cameras[cameraId]->getImageData();
    return imageData._retn();
  }else{
    ImageData* imageData = new ImageData;
    imageData->floatData.length(0);
    imageData->longData.length(0);
    imageData->octetData.length(0);
    return imageData;
  }
}

template <class T>
void Controller_impl<T>::input()
{
  if(CONTROLLER_BRIDGE_DEBUG){
    cout << "Controller_impl::input" << endl;
  }

  sensorStateUpdated = false;
  RobotController_.input();
}

template <class T>
DblSequence& Controller_impl<T>::
getJointDataSeqRef(DynamicsSimulator::LinkDataType linkDataType)
{
  return outputJointValueSeqInfos[linkDataType].values;
}

template <class T>
void Controller_impl<T>::output()
{
  if(CONTROLLER_BRIDGE_DEBUG){
    cout << "Controller_impl::output" << endl;
  }

  RobotController_.output();
}

template <class T>
void Controller_impl<T>::control()
{
  if(CONTROLLER_BRIDGE_DEBUG){
    cout << "Controller_impl::control" << endl;
  }
  
  RobotController_.control();
  controlTime += timeStep;
}

template <class T>
void Controller_impl<T>::stop()
{
  RobotController_.stop();
}

template <class T>
void Controller_impl<T>::destroy()
{
  if(CONTROLLER_BRIDGE_DEBUG){
    cout << "Controller_impl::destroy()" << endl;
  }
  PortableServer::POA_var poa = _default_POA();
  PortableServer::ObjectId_var id = poa->servant_to_id(this);
  poa->deactivate_object(id);
}


/// In initialize the corba server load the interface describing the relationship between
/// the robot and openhrp.
template <class T>
void Controller_impl<T>::initialize()
{
  if(CONTROLLER_BRIDGE_DEBUG)
    {
      cout << "Controller_impl::initialize()" << endl;
    }
  
  RobotController_.initialize();
}

template <class T>
void Controller_impl<T>::shutdown()
{
  if(CONTROLLER_BRIDGE_DEBUG){
    cout << "Controller_impl::shutdown()" << endl;
  }
  RobotController_.shutdown();
}

template <class T>
void Controller_impl<T>::restart()
{
  RobotController_.restart();
}

template <class T>
void Controller_impl<T>::set_path_to_library(std::string & path_to_library)
{
  RobotController_.set_path_to_library(path_to_library);
}

template <class T>
void Controller_impl<T>::set_path_to_pid_gains(std::string & path_to_pid_gains)
{
  RobotController_.set_path_to_pid_gains(path_to_pid_gains);
}
