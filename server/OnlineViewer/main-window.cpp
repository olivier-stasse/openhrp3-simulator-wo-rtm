#include <osgGA/TrackballManipulator>
#include "main-window.hh"
#include "create-axis.hh"

void * osgOnlineViewer_thread(void *arg)
{
  MainWindow * a_main_win= static_cast<MainWindow*>(arg);

  a_main_win->spin();
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
  //  osg::Matrixd aViewMatrix(0.715411,-0.467416,0.519335,0,
  //                           0.609386,0.0537899,-0.791047,0,
  //                           0.341813,0.882399,0.323319,0,
  //                           -1.75166,-0.735162,-0.5359,1);
  osg::Matrixd aViewMatrix(-0.988063,-0.153967,0.00499288,0,
					0.0106572,-0.0359862,0.999295,0,
					-0.153678,0.987421,0.0371975,0,
					-2.17096,8.09489,1.20741,1);


 
  // Custom camera position   0
  osg::Vec3d eye(-4.64704,9.30118,2.52838),
    center(-4.64704,9.30118,2.52838), 
    up(0.119454,-0.230789,0.965643);
  //-3.74331 6.67923 2.14167 -3.74331 6.67923 2.14167 0.108617 -0.205991 0.972507 0

  viewer_ptr.get()->getCameraManipulator()->setByMatrix(aViewMatrix);
}

int MainWindow::Init()
{
  world_ = GroupNode::create(std::string("world"));
  
  // Add axis to the drawing.
  createAxis(world_);

  InitWindowManager();

  pthread_t ptId;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_create(&ptId, &attr, 
		 osgOnlineViewer_thread,(void *)this);
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
		      
