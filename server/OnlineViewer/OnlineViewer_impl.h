/*
 * Copyright (c) 2016, CNRS,
 * All rights reserved. This program is made available under the terms of the
 * 
 */
/**
 * @file OnlineViewer_impl.h
 * @author Olivier Stasse
 */
#ifndef ONLINE_VIEWER_GEPETTO_IMPL_H_INCLUDED
#define ONLINE_VIEWER_GEPETTO_IMPL_H_INCLUDED

// OpenHRP headers
#include <hrpCorba/ORBwrap.h>
#include <hrpCorba/ModelLoader.hh>
#include <hrpCorba/OnlineViewer.hh>

#include <hrpModel/World.h>
#include <hrpUtil/TimeMeasure.h>

// Boost headers
#include <boost/scoped_ptr.hpp>

// OnlineViewer headers
#include "main-window.hh"

using namespace OpenHRP;

/**
 * OnlineViewer_impl class
 */
class OnlineViewer_impl : 
virtual public POA_OpenHRP::OnlineViewer,
  virtual public PortableServer::RefCountServantBase
{
  /**
   * ORB
   */
  CORBA::ORB_var orb_;
  
  CharacterPositionSequence_var allCharacterPositions;
  
  /// \brief Main window for display.
  MainWindow main_window_;

 public:
  
  OnlineViewer_impl(CORBA::ORB_ptr orb);
  
  ~OnlineViewer_impl();
  
  
  virtual void update(const WorldState & wstate);
  
  virtual void load(const char * name, const char * url);

  virtual void clearLog();

  virtual void clearData();

  virtual void drawScene(const WorldState & wstate);

  virtual void setLineWidth(CORBA::Float width);

  virtual void setLineScale(CORBA::Float scale);

  virtual CORBA::Boolean getPosture(const char * robotID,
				    DblSequence_out posture);

  virtual void setLogName(const char* name);
};


#endif
