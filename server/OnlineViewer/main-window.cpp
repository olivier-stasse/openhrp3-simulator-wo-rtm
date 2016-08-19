#include <osgGA/TrackballManipulator>
#include "main-window.hh"
#include "create-axis.hh"

void * osgOnlineViewer_thread(void *arg)
{
  MainWindow * a_main_win= static_cast<MainWindow*>(arg);

  a_main_win->spin();
  return 0;
}

MainWindow::MainWindow(CORBA::ORB_ptr orb):
  from_body_to_osg_(orb)
{
  
}


void MainWindow::InitWindowManager()
{
  gm_ = WindowManager::create();
  gm_->addNode(world_);
  //  osgViewer::Viewer viewer;
  // Handler keyboard event handler
  TestKeyboardEventHandler* keyboardEventHandler = new TestKeyboardEventHandler(world_->asGroup().get());
  osgViewer::ViewerRefPtr viewer_ptr = gm_.get()->getViewerClone();
  viewer_ptr.get()->addEventHandler(keyboardEventHandler); 
  viewer_ptr.get()->setSceneData( world_->asGroup() );

  osgGA::OrbitManipulator * anOrbitManipulator = 
    dynamic_cast<osgGA::OrbitManipulator *>(viewer_ptr.get()->getCameraManipulator());
  if (anOrbitManipulator!=0)
    {
      std::cout << "Camera Manipulator:" <<  anOrbitManipulator->getDistance() << std::endl;
      anOrbitManipulator->setDistance(0.95);
    }
  else
    std::cout << "Unable to detect: " << typeid(viewer_ptr.get()->getCameraManipulator()).name() << std::endl;

  keyboardEventHandler->setViewerRef(viewer_ptr);

  // Set View Matrix
  // osg::Matrixd aViewMatrix(-0.988063,-0.153967,0.00499288,0,
  // 					0.0106572,-0.0359862,0.999295,0,
  // 					-0.153678,0.987421,0.0371975,0,
  // 					-2.17096,8.09489,1.20741,1);
  osg::Matrixd aViewMatrix(-0.997808,-0.0612453,-0.0250599,0,
			   -0.0198733,-0.0838741,0.996278,0,
			   -0.0631193,0.994592,0.0824732,0,
			   -0.171284,3.21756,0.985355,1);


 
  // Custom camera position   0
  osg::Vec3d eye(-4.64704,9.30118,2.52838),
    center(-4.64704,9.30118,2.52838), 
    up(0.119454,-0.230789,0.965643);
  //-3.74331 6.67923 2.14167 -3.74331 6.67923 2.14167 0.108617 -0.205991 0.972507 0

  viewer_ptr.get()->getCameraManipulator()->setByMatrix(aViewMatrix);

}
osg::Node * createLights(osg::StateSet* rootStateSet)
{

  osg::Group* lightGroup = new osg::Group;
  
  // create a spot light.
  osg::Light* myLight1 = new osg::Light;
  myLight1->setLightNum(0);
  myLight1->setPosition(osg::Vec4(3.0f,0.0f,4.0f,1.0f));
  myLight1->setAmbient(osg::Vec4(0.8f,0.8f,0.8f,1.0f));
  myLight1->setDiffuse(osg::Vec4(0.2f,0.2f,0.2f,1.0f));
  //myLight1->setSpotCutoff(50.0f);
  //myLight1->setSpotExponent(100.0f);
  myLight1->setConstantAttenuation(1.0f);
  //myLight1->setSpecular(osg::Vec4(0.5f,0.5f,0.5f,0.5f));
  myLight1->setDirection(osg::Vec3(-1.0f,0.0f,-1.0f));
	
  osg::LightSource* lightS1 = new osg::LightSource;
  lightS1->setLight(myLight1);
  lightS1->setLocalStateSetModes(osg::StateAttribute::ON);
	
  lightS1->setStateSetModes(*rootStateSet,osg::StateAttribute::ON);
  lightGroup->addChild(lightS1);

  return lightGroup;
}

int MainWindow::Init()
{
  world_ = GroupNode::create(std::string("world"));
  
  // Add axis to the drawing.
  createAxis(world_);
  osg::StateSet* rootStateSet = new osg::StateSet;
  
  world_->getOsgNode()->setStateSet(rootStateSet);
  world_->asGroup()->addChild(createLights(rootStateSet));

  InitWindowManager();

  pthread_t ptId;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_create(&ptId, &attr, 
		 osgOnlineViewer_thread,(void *)this);
  return 1;
}

int MainWindow::spin()
{
  return gm_->run();
}

void MainWindow::load(const char *name,
		      const char *url)
{
  from_body_to_osg_.load(world_,name,url);
}
		      
void MainWindow::update(const OpenHRP::WorldState &wstate)
{
  from_body_to_osg_.update(wstate);
}
		      
