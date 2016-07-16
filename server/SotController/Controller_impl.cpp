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

#include "Controller_impl.h"

#include <string>
#include <iostream>
#include <rtm/Manager.h>
#include <rtm/RTObject.h>
#include <rtm/NVUtil.h>

#include <hrpCorba/ORBwrap.h>

#include "VirtualRobotRTC.h"

using namespace std;
using namespace boost;

namespace {
  const bool CONTROLLER_BRIDGE_DEBUG = false;
}


Controller_impl::Controller_impl()
    :   modelName(""),
        bRestart(false)
{
    if(CONTROLLER_BRIDGE_DEBUG){
        cout << "Controller_impl::Controller_impl" << endl;
    }
}


Controller_impl::~Controller_impl()
{
    if(CONTROLLER_BRIDGE_DEBUG){
        cout << "Controller_impl::~Controller_impl" << endl;
    }
}


void Controller_impl::setDynamicsSimulator(DynamicsSimulator_ptr dynamicsSimulator)
{
    this->dynamicsSimulator = DynamicsSimulator::_duplicate(dynamicsSimulator);
}


void Controller_impl::setViewSimulator(ViewSimulator_ptr viewSimulator)
{
    this->viewSimulator = ViewSimulator::_duplicate(viewSimulator);
}


void Controller_impl::start()
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
            activeComponents();
        }
    } catch(CORBA_SystemException& ex){
        cerr << ex._rep_id() << endl;
        cerr << "exception in Controller_impl::start" << endl;
    } catch(...){
        cerr << "unknown exception in Controller_impl::start()" <<  endl;
    }
}


SensorState& Controller_impl::getCurrentSensorState()
{
    if(!sensorStateUpdated){
        dynamicsSimulator->getCharacterSensorState(modelName.c_str(), sensorState);
        sensorStateUpdated = true;
    }

    return sensorState;
}


DblSequence* Controller_impl::getLinkDataFromSimulator
(const std::string& linkName, DynamicsSimulator::LinkDataType linkDataType)
{
    DblSequence_var data;
    dynamicsSimulator->getCharacterLinkData(modelName.c_str(), linkName.c_str(), linkDataType, data.out());
    return data._retn();
}


DblSequence* Controller_impl::getSensorDataFromSimulator(const std::string& sensorName)
{
    DblSequence_var data;
    dynamicsSimulator->getCharacterSensorValues(modelName.c_str(), sensorName.c_str(), data.out());
    return data._retn();
}


ImageData* Controller_impl::getCameraImageFromSimulator(int cameraId)
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


void Controller_impl::input()
{
  //if(CONTROLLER_BRIDGE_DEBUG){
    cout << "Controller_impl::input" << endl;
    // }
  
  sensorStateUpdated = false;
  
  RobotController_.input();
}


DblSequence& Controller_impl::getJointDataSeqRef(DynamicsSimulator::LinkDataType linkDataType)
{
    return outputJointValueSeqInfos[linkDataType].values;
}


void Controller_impl::flushJointDataSeqToSimulator(DynamicsSimulator::LinkDataType linkDataType)
{
    JointValueSeqInfoMap::iterator p = outputJointValueSeqInfos.find(linkDataType);
    if(p != outputJointValueSeqInfos.end()){
        JointValueSeqInfo& info = p->second;
        if(!info.flushed){
            dynamicsSimulator->setCharacterAllLinkData(modelName.c_str(), linkDataType, info.values);
            info.flushed = true;
        }
    }
}

void Controller_impl::flushLinkDataToSimulator(const std::string& linkName,
					       DynamicsSimulator::LinkDataType linkDataType,
					       const DblSequence& linkData)
{
    dynamicsSimulator->setCharacterLinkData(modelName.c_str(), linkName.c_str(),
					linkDataType, linkData);
}

void Controller_impl::output()
{
  //    if(CONTROLLER_BRIDGE_DEBUG){
        cout << "Controller_impl::output" << endl;
	// }

    // for(JointValueSeqInfoMap::iterator p = outputJointValueSeqInfos.begin();
    //   p != outputJointValueSeqInfos.end(); ++p){
    //     JointValueSeqInfo& info = p->second;
    //     info.flushed = false;
    // }
    RobotController_.output();
}


void Controller_impl::control()
{
  if(CONTROLLER_BRIDGE_DEBUG){
    cout << "Controller_impl::control" << endl;
  }
  
  RobotController_.control();
    
  controlTime += timeStep;
}


void Controller_impl::stop()
{
  RobotController_.stop();
}

void Controller_impl::destroy()
{
    if(CONTROLLER_BRIDGE_DEBUG){
        cout << "Controller_impl::destroy()" << endl;
    }
    PortableServer::POA_var poa = _default_POA();
    PortableServer::ObjectId_var id = poa->servant_to_id(this);
    poa->deactivate_object(id);
}

// TAWARA INSERT CODE 2009/01/14 START

void Controller_impl::set_path_to_library(std::string & path_to_library)
{
  path_to_openhrp2robot_int = path_to_library;
}

/// In initialize the corba server load the interface describing the relationship between
/// the robot and openhrp.
void Controller_impl::initialize()
{
  RobotController_.initialize();
}

void Controller_impl::shutdown()
{
  if(CONTROLLER_BRIDGE_DEBUG){
    cout << "Controller_impl::shutdown()" << endl;
  }
  RobotController_.shutdown();
  destroy();
}

void Controller_impl::restart()
{
  RobotController_.restart();
}
