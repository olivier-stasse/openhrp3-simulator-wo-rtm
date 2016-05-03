/*
 * Copyright (c) 2016, CNRS
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 */
/**
 * @file OnlineViewer_impl.cpp
 * @author Olivier Stasse
*/

#include "OnlineViewer_impl.h"

using namespace std;
using namespace hrp;


OnlineViewer_impl::OnlineViewer_impl(CORBA::ORB_ptr orb)
  : orb_(CORBA::ORB::_duplicate(orb)),
    main_window_(orb)
{
  main_window_.Init();
}

OnlineViewer_impl::~OnlineViewer_impl()
{
  
}

void OnlineViewer_impl::update(const WorldState & wstate)
{
  main_window_.update(wstate);
}

void OnlineViewer_impl::load(const char * name,
			     const char * url)
{
  main_window_.load(name,url);
}


void OnlineViewer_impl::clearLog()
{

}

void OnlineViewer_impl::clearData()
{

}

void OnlineViewer_impl::drawScene(const WorldState & wstate)
{

}

void OnlineViewer_impl::setLineWidth(CORBA::Float width)
{

}

void OnlineViewer_impl::setLineScale(CORBA::Float scale)
{

}

CORBA::Boolean OnlineViewer_impl::getPosture(const char *robotID,
					     DblSequence_out posture)
{

}

void OnlineViewer_impl::setLogName(const char * name)
{
  
}



