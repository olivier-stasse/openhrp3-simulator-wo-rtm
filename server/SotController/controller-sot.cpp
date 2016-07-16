void ControllerSot::initialize()
{
  if(CONTROLLER_BRIDGE_DEBUG)
    {
      cout << "Controller_impl::initialize()" << endl;
    }
  
  char * sLD_LIBRARY_PATH;
  sLD_LIBRARY_PATH=getenv("LD_LIBRARY_PATH");
  ODEBUG5("Load OpenHRP2SOT - Start " << sLD_LIBRARY_PATH);
  char * sPYTHONPATH;
  sPYTHONPATH=getenv("PYTHONPATH");
  ODEBUG5("PYTHONPATH:" << sPYTHONPATH );
  sPYTHONPATH=getenv("PYTHON_PATH");
  ODEBUG5("PYTHON_PATH:" << sPYTHONPATH);
  
  // Load the openhrp-to-sot-robot-interface library.
  void * openhrp_to_sot_robot_int_lib = 
    dlopen(path_to_openhrp2robot_int_.c_str(),
	   RTLD_GLOBAL | RTLD_NOW);
  if (!openhrp_to_sot_robot_int_lib) 
    {
      ODEBUG5("Cannot load library: " << dlerror() );
      return ;
    }

  ODEBUG5("Success in loading the library:" << path_to_openhrp2robot_int_);
  // reset errors
  dlerror();
  
  // Load the symbols.
  CreateOpenHRPToSotRobotInterface_t * createopenhrp2sot =
    (CreateOpenHRPToSotRobotInterface_t *) dlsym(openhrp_to_sot_robot_int_lib,
						 "createOpenHRP2SotInternalInterface");
  ODEBUG5("createHRPController call "<< std::hex
          << std::setbase(10));
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    ODEBUG5("Cannot load symbol create: " << dlsym_error );
    return ;
  }
  ODEBUG5("Success in getting the controller factory");
  
  // Create OpenHRP 2 Sot Robot Interface
  try 
    {
      ODEBUG5("exception handled createHRP2Controller call "<< std::hex 
              << std::setbase(10));
      openhrp_2_sot_robot_interface_ = createopenhrp2sot();
      ODEBUG5("After createHRP2Controller.");
    } 
  catch (std::exception &e)
    {
      ODEBUG5("Exception: " << e.what());
    }
  ODEBUG5("LoadOpenHRP2SOT - End");
}
