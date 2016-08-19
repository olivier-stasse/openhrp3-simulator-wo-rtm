/*
 * Copyright (c) 2016, CNRS
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * LAAS
 */
/**
   @file SotController/server/server.cpp
*/

#include "hrp2-oh2sot.hh"
#include "Controller_impl.h"

#ifdef _WIN32
#include "winbase.h"
#else
#include <unistd.h>
#endif /* _WIN32 */

#include <iostream>

using namespace std;


int main(int argc, char* argv[])
{

  ros::init(argc,argv,"openhrp_3_hrp2_encapsulator");

  if (argc!=3)
    {
      std::cout << argv[0] << " must have two parameter (path to sot library and pid gains)."
		<< std::endl;
      return -1;
    }
  std::string robot_libname = argv[1];
  std::string pid_gains_filename = argv[2];
 
  
  CORBA::ORB_var orb;
  try 
    {
      orb = CORBA::ORB_init(argc, argv);
      //
      // Resolve Root POA
      //
      CORBA::Object_var poaObj = orb -> resolve_initial_references("RootPOA");
      PortableServer::POA_var rootPOA = PortableServer::POA::_narrow(poaObj);
      
      //
      // Get a reference to the POA manager
      //
      PortableServer::POAManager_var manager = rootPOA -> the_POAManager();
      
      CosNaming::NamingContext_var cxT;
      CORBA::Object_var	nS = orb->resolve_initial_references("NameService");
      cxT = CosNaming::NamingContext::_narrow(nS);
      
      CORBA::Object_var hrp2ControllerObj;
      Controller_impl<HRP2OH2SOT> * hrp2ControllerImpl = new Controller_impl<HRP2OH2SOT>(orb);
      hrp2ControllerImpl->set_path_to_library(robot_libname);
      hrp2ControllerImpl->set_path_to_pid_gains(pid_gains_filename);
      hrp2ControllerObj = hrp2ControllerImpl -> _this();
      CosNaming::Name nc;
      nc.length(1);
      nc[0].id = CORBA::string_dup("HRP2PDController");
      nc[0].kind = CORBA::string_dup("");
      cxT -> rebind(nc, hrp2ControllerObj);
      
      manager -> activate();
      cout << "ready" << endl;
      
      orb -> run();
    } catch (CORBA::SystemException& ex) {
    cerr << ex._rep_id() << endl;
    return 1;
  }
  orb->destroy();
  return 0;
}
