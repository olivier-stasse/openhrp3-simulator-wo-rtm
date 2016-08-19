// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
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
*/

#ifndef OPENHRP_CONTROLLER_BRIDGE_CONTROLLER_IMPL_H_INCLUDED
#define OPENHRP_CONTROLLER_BRIDGE_CONTROLLER_IMPL_H_INCLUDED

#include <string>
#include <map>

#include <hrpCorba/Controller.hh>
#include <hrpCorba/ViewSimulator.hh>
#include <hrpCorba/DynamicsSimulator.hh>


#include "config.h"

using namespace OpenHRP;

template <class T>
class Controller_impl
	: virtual public POA_OpenHRP::Controller
{
public:
	Controller_impl(CORBA::ORB_ptr orb);
	~Controller_impl();

	SensorState& getCurrentSensorState();
	DblSequence* getLinkDataFromSimulator
	(const std::string& linkName, DynamicsSimulator::LinkDataType linkDataType);
	DblSequence* getSensorDataFromSimulator(const std::string& sensorName);
	ImageData* getCameraImageFromSimulator(int cameraId);
	DblSequence& getJointDataSeqRef(DynamicsSimulator::LinkDataType linkDataType);
	virtual void setDynamicsSimulator(DynamicsSimulator_ptr dynamicsSimulator);
	virtual void setViewSimulator(ViewSimulator_ptr viewSimulator);
	void setTimeStep(CORBA::Double _timeStep){
        timeStep = _timeStep;
    }
    double getTimeStep(){
        return timeStep;
    }

	virtual void start();
	virtual void control();
	virtual void input();
	virtual void output();
	virtual void stop();
	virtual void destroy();

    virtual void shutdown();
    virtual omniObjRef* _do_get_interface(){return _this();}
    virtual void setModelName(const char* localModelName){ RobotController_.modelName_ = localModelName;}
    virtual void initialize();
    double controlTime;

	/// Set the openhrp-to-sot-robot interface library.
	/// In unix it should the path towards the library.
	void set_path_to_library(std::string & path_to_library);
	/// Set the pid gains filename
	void set_path_to_pid_gains(std::string & path_to_pid_gains);

private:
	std::string modelName;

	/// Robot controller.
	T RobotController_;

	DynamicsSimulator_var dynamicsSimulator;
	ViewSimulator_var viewSimulator;

	SensorState_var sensorState;
	bool sensorStateUpdated;

	struct JointValueSeqInfo {
		bool flushed;
		DblSequence values;
	};

	typedef std::map<DynamicsSimulator::LinkDataType, JointValueSeqInfo> JointValueSeqInfoMap;
	JointValueSeqInfoMap outputJointValueSeqInfos;

	CameraSequence_var cameras;
	Camera::CameraParameter_var cparam;

    double timeStep;
    bool bRestart;
    void restart();

	/// Path the dynamic library to map OpenHRP and the robot interface.
	std::string path_to_openhrp2robot_int_;

};

#include "Controller_impl.hpp"
#endif
