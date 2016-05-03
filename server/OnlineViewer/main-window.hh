
//
//  main-window.cpp
//  Test for study-reem-c
//
//  Created by Olivier Stasse in October 2015.
//  Copyright (c) 2015 LAAS-CNRS. All rights reserved.
//

#ifndef _MAIN_WINDOW_H_
#define _MAIN_WINDOW_H_

#include <iostream>
#include <string>


// Event handler
#include <gepetto/viewer/group-node.h>
#include <gepetto/viewer/window-manager.h>
#include "keyboard-handler.hh"

// 3D environment
#include <osgWidget/ViewerEventHandlers>
#include <osgDB/WriteFile>

// OpenHRP 
#include <hrpModel/ModelLoaderUtil.h>
// Local headers
#include "from-body-to-osg.hh"

using namespace graphics;

class MainWindow
{
protected:

  // OSG related variables.

  // World 
  GroupNodePtr_t world_;

  // Window manager.
  graphics::WindowManagerPtr_t gm_;

  // Convert BodyInfo object to OSG for display purposes.
  FromBodyToOsg from_body_to_osg_;

public:
  // Constructor
  MainWindow(CORBA::ORB_ptr orb);

  // Initialization of the main window.
  int Init();

  // Window manager
  void InitWindowManager();

  // Spin
  int spin();

  // Load object.
  void load(const char *name,
	    const char *url);

  // Update object.
void update(const OpenHRP::WorldState &wstate);

};

#endif /* _MAIN_WINDOW_H_ */
