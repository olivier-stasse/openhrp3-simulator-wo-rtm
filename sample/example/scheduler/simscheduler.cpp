#include <simscheduler.hh>

using namespace OpenHRP;
using namespace std;
using namespace hrp;

template <typename X, typename X_ptr>
X_ptr checkCorbaServer(std::string n, CosNaming::NamingContext_var &cxt)
{
  CosNaming::Name ncName;
  ncName.length(1);
  ncName[0].id = CORBA::string_dup(n.c_str());
  ncName[0].kind = CORBA::string_dup("");
  X_ptr srv = NULL;
  try {
    srv = X::_narrow(cxt->resolve(ncName));
  } catch(const CosNaming::NamingContext::NotFound &exc) {
    std::cerr << n << " not found: ";
    switch(exc.why) {
    case CosNaming::NamingContext::missing_node:
      std::cerr << "Missing Node" << std::endl;
    case CosNaming::NamingContext::not_context:
      std::cerr << "Not Context" << std::endl;
      break;
    case CosNaming::NamingContext::not_object:
      std::cerr << "Not Object" << std::endl;
      break;
    }
    return (X_ptr)NULL;
  } catch(CosNaming::NamingContext::CannotProceed &exc) {
    std::cerr << "Resolve " << n << " CannotProceed" << std::endl;
  } catch(CosNaming::NamingContext::AlreadyBound &exc) {
    std::cerr << "Resolve " << n << " InvalidName" << std::endl;
  }
  return srv;
}


SimScheduler::SimScheduler():
  controller_found_(false),
  timeStep_(0.001),  // (s)
  controlTimeStep_(0.001),
  EndTime_(2000.0)
{
}

SimScheduler::~SimScheduler()
{
}

void SimScheduler::parseOptions(int argc, char *argv[])
{
  std::string Model[2], ModelFname[2];
  
  
  for (int i=1; i<argc; i++)
    {
      if (strcmp("-ORBconfig", argv[i])==0 || strcmp("-ORBInitRef", argv[i])==0 )
	{
	  argv[++i];	// skip ORB parameter
	}
      else if (strcmp("-url", argv[i])==0)
	{
	  for(int j = 0; j < 2; j++)
	    {
	      ModelFname[j] = argv[++i];
	    }
	}
      else if (strcmp("-timeStep", argv[i])==0)
	{
	  timeStep_ = atof(argv[++i]);
	}
      else if (strcmp("-serverName", argv[i])==0)
	{
	  controllerName_ = argv[++i];
	}
      
    }
  
  for(int j = 0; j < 2; j++)
    {
      Model[j] = "file://" + ModelFname[j];
      std::cout << "Model: " << Model[j] << std::endl;
    }
  floorfilename_ = Model[0];
  robotfilename_ = Model[1];
}

void SimScheduler::loadModels(int argc, char *argv[])
{
  floor_ = hrp::loadBodyInfo(floorfilename_.c_str(), argc, argv);
  if(!floor_)
    {
      cerr << "ModelLoader: " << floorfilename_ << " cannot be loaded" << endl;
      return;
    }
  body_ = hrp::loadBodyInfo(robotfilename_.c_str(), argc, argv);
  if(!body_)
    {
      cerr << "ModelLoader: " << robotfilename_ << " cannot be loaded" << endl;
      return;
    }

  double lmass=0.0;
  LinkInfoSequence_var links = body_->links();
  for(unsigned int i=0;i<links->length();i++)
    {
      lmass+=links[i].mass;
    };
  std::cout << "Mass of " <<body_->name() << " : " << lmass << std::endl;
}

void SimScheduler::initCorba(int argc, char * argv[])
{
  // initialize CORBA 
  CORBA::ORB_var orb;
  orb = CORBA::ORB_init(argc, argv);
  
  // ROOT POA
  CORBA::Object_var poaObj = orb -> resolve_initial_references("RootPOA");
  PortableServer::POA_var rootPOA = PortableServer::POA::_narrow(poaObj);
  
  // get reference to POA manager
  PortableServer::POAManager_var manager = rootPOA -> the_POAManager();
  

  {
    CORBA::Object_var	nS = orb->resolve_initial_references("NameService");
    cxt_ = CosNaming::NamingContext::_narrow(nS);
  }
}

void SimScheduler::initOLV(int argc, char * argv[])
{
  olv_ = hrp::getOnlineViewer(argc, argv);
  
  if (CORBA::is_nil( olv_ )) 
    {
      std::cerr << "OnlineViewer not found" << std::endl;
      return;
    }
  try 
    {
      olv_->load(body_->name(), robotfilename_.c_str());
      olv_->load(floor_->name(), floorfilename_.c_str());
      olv_->clearLog();
    } 
  catch (CORBA::SystemException& ex) 
    {
      cerr << "Failed to connect GrxUI." << endl;
      return;
    }
}

void SimScheduler::initRobotsFreeFlyerPosition()
{
  // initial position and orientation
  Vector3  waist_p;
  Matrix33 waist_R;
  if (!strcmp(body_->name(),"HRP2"))
    waist_p << 0, 0, 0.705;
  else
    waist_p << 0, 0, 0.6487;
  waist_R = Matrix33::Identity();
  
  DblSequence trans;
  trans.length(12);
  for(int i=0; i<3; i++) trans[i]   = waist_p(i);
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++) trans[3+3*i+j] = waist_R(i,j);
  }
  dynamicsSimulator_->setCharacterLinkData( body_->name(), 
					    "WAIST", 
					    DynamicsSimulator::ABS_TRANSFORM, 
					    trans );

}

void SimScheduler::initRobotsPose()
{
  DblSequence angle;
  if (!strcmp(body_->name(),"sample"))
    {
      angle.length(29);
      angle[0] = 0.0;         angle[1] = -0.0360373;  
      angle[2] = 0.0;         angle[3] = 0.0785047;
      angle[4] = -0.0424675;  angle[5] = 0.0;         
      
      angle[6] = 0.174533;    angle[7] = -0.00349066;
      angle[8] = 0.0;         angle[9] = -1.5708;     
      angle[10] = 0.0;        angle[11] = 0.0;
      
      angle[12] = 0.0;        angle[13] = 0.0;        
      angle[14] = -0.0360373; angle[15] = 0.0;       
      
      angle[16] = 0.0785047;  angle[17] = -0.0424675; angle[18] = 0.0;        
      angle[19] = 0.174533;	angle[20] = -0.00349066; angle[21] = 0.0; angle[22] = -1.5708;    
      
      angle[23] = 0.0;	angle[24] = 0.0;  angle[25] = 0.0;        
      angle[26] = 0.0;  angle[27] = 0.0;  angle[28] = 0.0;
    }
  else 	if (!strcmp(body_->name(),"HRP2JRL"))
    {
      angle.length(40);
      angle[0] = 0.0    ; angle[1]  = 0.0     ; angle[2] = -0.45379;
      angle[3] = 0.87266; angle[4]  =-0.41888 ; angle[5] = 0.0;
      
      angle[6] = 0.0    ; angle[7]  = 0.0     ; angle[8] = -0.45379;
      angle[9] = 0.87266; angle[10] =-0.41888 ; angle[11] = 0.0;
      
      angle[12] = 0.0   ; angle[13] = 0.0;
      angle[14] = 0.0   ; angle[15] = 0.0;
      
      angle[16] = 0.2618; angle[17] =-0.17453 ; angle[18] = 0.0;
      angle[19] =-0.5236; angle[20] = 0.0     ; angle[21] = 0.0; 
      angle[22] = 0.17453;
      
      angle[23] = 0.2618; angle[24] = 0.17453 ; angle[25] = 0.0;
      angle[26] =-0.5236; angle[27] = 0.0     ; angle[28] = 0.0;
      angle[29] = 0.17453;
      angle[30] =-0.17453; angle[31] = 0.17453;	 angle[32] = -0.17453;		angle[33] = 0.17453;			angle[34] = -0.17453;
      angle[35] =-0.17453; angle[36] = 0.17453;	 angle[37] = -0.17453;		angle[38] = 0.17453;			angle[39] = -0.17453;
    }
  
  dynamicsSimulator_->setCharacterAllLinkData( body_->name(), 
					       DynamicsSimulator::JOINT_VALUE, 
					       angle );

}

void SimScheduler::initRobotsJointMode()
{
  dynamicsSimulator_->setCharacterAllJointModes( body_->name(), 
						 DynamicsSimulator::HIGH_GAIN_MODE);
  DblSequence wdata;
  wdata.length(1);
  wdata[0] = -1.0;
  // The bush fill are torque controlled.
  dynamicsSimulator_->setCharacterLinkData( body_->name(), 
					    "RLEG_BUSH_Z", 
					    DynamicsSimulator::POSITION_GIVEN, 
					    wdata ); 
  dynamicsSimulator_->setCharacterLinkData( body_->name(), 
					    "RLEG_BUSH_ROLL", 
					    DynamicsSimulator::POSITION_GIVEN, 
					    wdata );
  dynamicsSimulator_->setCharacterLinkData( body_->name(), 
					    "RLEG_BUSH_PITCH", 
					    DynamicsSimulator::POSITION_GIVEN, 
					    wdata );
  
  dynamicsSimulator_->setCharacterLinkData( body_->name(), 
					    "LLEG_BUSH_Z", 
					    DynamicsSimulator::POSITION_GIVEN, 
					    wdata ); 
  dynamicsSimulator_->setCharacterLinkData( body_->name(), 
					    "LLEG_BUSH_ROLL", 
					    DynamicsSimulator::POSITION_GIVEN, 
					    wdata );
  dynamicsSimulator_->setCharacterLinkData( body_->name(), 
					    "LLEG_BUSH_PITCH", 
					    DynamicsSimulator::POSITION_GIVEN, 
					    wdata );
}

void SimScheduler::initParallelMecanisms()
{
  DblSequence3 aLink1LocalPos,aLink2LocalPos;
  aLink1LocalPos.length(3); aLink2LocalPos.length(3);
  aLink1LocalPos[0] = 0.0; 	aLink1LocalPos[1] = 0.004; 	aLink1LocalPos[2] = -0.11;
  aLink2LocalPos[0] = 0.0; 	aLink2LocalPos[1] = 0.0; aLink2LocalPos[2] = 0.06 ;
  DblSequence3 aJointAxis;
  aJointAxis.length(3);
  aJointAxis[0] = 1.0; aJointAxis[1] = 0.0; aJointAxis[2] = 0.0;
  
  // Close the kinematic chain of the Right hand - gripper
  dynamicsSimulator_->registerExtraJoint(body_->name(), "RARM_JOINT5",
					 body_->name(), "RHAND_JOINT1",
					 aLink1LocalPos, aLink2LocalPos,
					 ExtraJointType::EJ_XYZ,
					 aJointAxis,
					 "RL6RL1");
  
  aLink1LocalPos[0] = 0.0; 	aLink1LocalPos[1] = -0.004; 	aLink1LocalPos[2] = -0.11;
  aLink2LocalPos[0] = 0.0; 	aLink2LocalPos[1] = 0.0; aLink2LocalPos[2] = 0.06 ;
  dynamicsSimulator_->registerExtraJoint(body_->name(), "RARM_JOINT5",
					 body_->name(), "RHAND_JOINT4",
					 aLink1LocalPos, aLink2LocalPos,
					 ExtraJointType::EJ_XYZ,
					 aJointAxis,
					 "RH2RL4");
	
  // Close the kinematic chain of the Left hand - gripper
  aLink1LocalPos[0] = 0.0; 	aLink1LocalPos[1] = -0.004; 	aLink1LocalPos[2] = -0.11;
  aLink2LocalPos[0] = 0.0; 	aLink2LocalPos[1] = 0.0; aLink2LocalPos[2] = 0.06 ;
  dynamicsSimulator_->registerExtraJoint(body_->name(), "LARM_JOINT5",
					 body_->name(), "LHAND_JOINT1",
					 aLink1LocalPos, aLink2LocalPos,
					 ExtraJointType::EJ_XYZ,
					 aJointAxis,
					 "RL6RL1");
  aLink1LocalPos[0] = 0.0; 	aLink1LocalPos[1] = 0.004; 	aLink1LocalPos[2] = -0.11;
  aLink2LocalPos[0] = 0.0; 	aLink2LocalPos[1] = 0.0; aLink2LocalPos[2] = 0.06 ;
  dynamicsSimulator_->registerExtraJoint(body_->name(), "LARM_JOINT5",
					 body_->name(), "LHAND_JOINT4",
					 aLink1LocalPos, aLink2LocalPos,
					 ExtraJointType::EJ_XYZ,
					 aJointAxis,
					 "RH2RL4");
}

void SimScheduler::initCollisions()
{
  double statFric,slipFric;
  statFric = slipFric = 0.5;   // static/slip friction coefficient 
  double culling_thresh = 0.01;
  
  DblSequence6 K, C;    // spring-damper parameters are not used now
  K.length(0);
  C.length(0);
  dynamicsSimulator_->registerCollisionCheckPair(floor_->name(),"", body_->name() ,"",
						 statFric,slipFric,K,C,culling_thresh,0.0);
  
}
void SimScheduler::initDynamicsSimulator()
{
  DynamicsSimulatorFactory_var dynamicsSimulatorFactory;
  dynamicsSimulatorFactory =
    checkCorbaServer <DynamicsSimulatorFactory, DynamicsSimulatorFactory_var> ("DynamicsSimulatorFactory", cxt_);
  
  if (CORBA::is_nil(dynamicsSimulatorFactory)) {
    std::cerr << "DynamicsSimulatorFactory not found" << std::endl;
  }
  
  // Create dynamics simulator
  dynamicsSimulator_ = dynamicsSimulatorFactory->create();
  
  // Register robots.
  cout << "** Dynamics server setup ** " << endl;
  cout << "Character  :" << floor_->name() << endl;
  dynamicsSimulator_->registerCharacter(floor_->name(), floor_);			    
  cout << "Character  :" << body_->name() << endl;
  dynamicsSimulator_->registerCharacter(body_->name(), body_);

  // Enable sensor and gravity.
  dynamicsSimulator_->init(timeStep_, 
			   DynamicsSimulator::RUNGE_KUTTA,
			   DynamicsSimulator::ENABLE_SENSOR);
  DblSequence3 g;
  g.length(3);
  g[0] = 0.0;
  g[1] = 0.0;
  double world_gravity = 9.8;  // default gravity acceleration [m/s^2]
  g[2] = world_gravity;
  dynamicsSimulator_->setGVector(g);
  
  initRobotsFreeFlyerPosition();
  initRobotsPose();
  initRobotsJointMode();
  dynamicsSimulator_->calcWorldForwardKinematics();
  
  initCollisions();
  initParallelMecanisms();
  dynamicsSimulator_->initSimulation();

}

void SimScheduler::initController()
{

  try 
    {
      controller_ = checkCorbaServer <Controller, Controller_var> 
	(controllerName_.c_str(), cxt_);
      controller_found_ = true;
    }
  catch (CORBA::SystemException& ex) 
    {
      cerr << "Failed to connect to " << controllerName_.c_str() << endl;
      controller_found_ = false;
    }
  
  if (CORBA::is_nil(controller_)) {
    std::cerr << "Controller " << controllerName_ << " not found" << std::endl;
  }
  
  if (controller_found_)
    {
      controller_->setModelName(body_->name());
      controller_->setDynamicsSimulator(dynamicsSimulator_);
      controller_->initialize();
      controller_->setTimeStep(controlTimeStep_);
      controller_->start();
    }

}

void SimScheduler::init(int argc,char * argv[])
{
  // Parse options
  parseOptions(argc,argv);

  //================== Model Load ===============================
  loadModels(argc,argv);
  
  //================== CORBA init ===============================
  initCorba(argc,argv);
  
  //==================== OnlineViewer (GrxUI) setup ===============
  initOLV(argc,argv);
  
  //================= DynamicsSimulator setup ======================
  initDynamicsSimulator();
  
  // ==================  Controller setup ==========================
  initController();
}
void SimScheduler::mainLoop()
{
  // ==================  log file   ======================
  static ofstream log_file;
  log_file.open("samplePD.log");
  
  WorldState_var state;
  int i=0;
  int j = 0;
  double time=0.0;
  double controlTime=0.0;
  while ( 1 ) {
    bool control=false;
    if(controlTime <= time){
      control=true;
      j++;
    }

    if(control)
      {
	if (controller_found_)
	  controller_->input();
      }

    i++;
    if (i%1000==0)
      std::cout << " Counter: " << i << std::endl;

    time = timeStep_ * i;
    controlTime = controlTimeStep_ * j;

    if(control)
      if(controller_found_)
	controller_->control();
      
    // ================== simulate one step ==============
    dynamicsSimulator_->stepSimulation();					
               
    // ================== viewer update ====================
    try {
      dynamicsSimulator_ -> getWorldState( state );
      olv_->update( state );
    } catch (CORBA::SystemException& ex) {
      return ;
    }
	
    // ===================== get robot status ===================
    DblSequence_var waist_pR,lleg_joint5,rarm_joint5, rhand_joint1;  // position and attitude
    DblSequence_var waist_vw;  // linear and angular velocities
    dynamicsSimulator_->getCharacterLinkData(body_->name(), 
					     "WAIST", 
					     DynamicsSimulator::ABS_TRANSFORM, 
					     waist_pR);
    dynamicsSimulator_->getCharacterLinkData(body_->name(), 
					     "LLEG_JOINT5", 
					     DynamicsSimulator::ABS_TRANSFORM, 
					     lleg_joint5);
    dynamicsSimulator_->getCharacterLinkData(body_->name(), 
					     "RARM_JOINT5", 
					     DynamicsSimulator::ABS_TRANSFORM, 
					     rarm_joint5);
    dynamicsSimulator_->getCharacterLinkData(body_->name(), 
					     "RHAND_JOINT1", 
					     DynamicsSimulator::ABS_TRANSFORM, 
					     rhand_joint1);

    // ================== log data save =====================
    if(control)
      {
	if(controller_found_)
	  controller_->output();
      }

    if( time > EndTime_ ) break;
			
  }

  //controller->stop();
  log_file.close();
  dynamicsSimulator_->destroy();

}
