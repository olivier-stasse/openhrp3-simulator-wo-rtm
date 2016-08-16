#include <iostream>
#include "keyboard-handler.hh"

#include <osgWidget/ViewerEventHandlers>
#include <osgDB/WriteFile>

bool TestKeyboardEventHandler::handle(const osgGA::GUIEventAdapter& ea,
                                      osgGA::GUIActionAdapter& )
{
  double dpitch=1.0,dyaw=1.0,droll=1.0,dforward=0.25;
  switch(ea.getEventType())
    {
    case(osgGA::GUIEventAdapter::KEYDOWN):
      {
        switch(ea.getKey())
          {
          case 'w':
            if (tGroup_ !=0)
              {
                if (osgDB::writeNodeFile(*tGroup_, "/tmp/saved.dae")==false)
                  std::cerr << "Unable to write saved.dae" << std::endl;
              }
            return false;
            break;


	  case '8':
	    {
	      osg::Matrixd aViewMatrix = viewer_ptr_.get()->getCameraManipulator()->getMatrix();
	      osg::Matrixd cameraRotation;
	      
	      cameraRotation.makeRotate(
					osg::DegreesToRadians(0.0), osg::Vec3(0,1,0), // roll
					osg::DegreesToRadians(dpitch), osg::Vec3(1,0,0) , // pitch
					osg::DegreesToRadians(0.0), osg::Vec3(0,0,1) ); // heading

	      osg::Matrixd aNewViewMatrix = cameraRotation * aViewMatrix;
	      viewer_ptr_.get()->getCameraManipulator()->setByMatrix(aNewViewMatrix);
	      return false;
	      break;
	    }
	  case '2':
	    {
	      osg::Matrixd aViewMatrix = viewer_ptr_.get()->getCameraManipulator()->getMatrix();
	      osg::Matrixd cameraRotation;
	      
	      cameraRotation.makeRotate(
					osg::DegreesToRadians(0.0), osg::Vec3(0,1,0), // roll
					osg::DegreesToRadians(-dpitch), osg::Vec3(1,0,0) , // pitch
					osg::DegreesToRadians(0.0), osg::Vec3(0,0,1) ); // heading

	      osg::Matrixd aNewViewMatrix = cameraRotation * aViewMatrix;
	      viewer_ptr_.get()->getCameraManipulator()->setByMatrix(aNewViewMatrix);
	      return false;
	      break;
	    }
	  case '4':
	    {
	      osg::Matrixd aViewMatrix = viewer_ptr_.get()->getCameraManipulator()->getMatrix();
	      osg::Matrixd cameraRotation;
	      
	      cameraRotation.makeRotate(
					osg::DegreesToRadians(dyaw), osg::Vec3(0,1,0), // roll
					osg::DegreesToRadians(0.0), osg::Vec3(1,0,0) , // pitch
					osg::DegreesToRadians(0.0), osg::Vec3(0,0,1) ); // heading

	      osg::Matrixd aNewViewMatrix = cameraRotation * aViewMatrix;
	      viewer_ptr_.get()->getCameraManipulator()->setByMatrix(aNewViewMatrix);
	      return false;
	      break;
	    }
	  case '6':
	    {
	      osg::Matrixd aViewMatrix = viewer_ptr_.get()->getCameraManipulator()->getMatrix();
	      osg::Matrixd cameraRotation;
	      
	      cameraRotation.makeRotate(
					osg::DegreesToRadians(-dyaw), osg::Vec3(0,1,0), // roll
					osg::DegreesToRadians(0.0), osg::Vec3(1,0,0) , // pitch
					osg::DegreesToRadians(0.0), osg::Vec3(0,0,1) ); // heading

	      osg::Matrixd aNewViewMatrix = cameraRotation * aViewMatrix;
	      viewer_ptr_.get()->getCameraManipulator()->setByMatrix(aNewViewMatrix);
	      return false;
	      break;
	    }
	  case '7':
	    {
	      osg::Matrixd aViewMatrix = viewer_ptr_.get()->getCameraManipulator()->getMatrix();
	      osg::Matrixd cameraRotation;
	      
	      cameraRotation.makeRotate(
					osg::DegreesToRadians(0.0), osg::Vec3(0,1,0), // roll
					osg::DegreesToRadians(0.0), osg::Vec3(1,0,0) , // pitch
					osg::DegreesToRadians(droll), osg::Vec3(0,0,1) ); // heading

	      osg::Matrixd aNewViewMatrix = cameraRotation * aViewMatrix;
	      viewer_ptr_.get()->getCameraManipulator()->setByMatrix(aNewViewMatrix);
	      return false;
	      break;
	    }
	  case '9':
	    {
	      osg::Matrixd aViewMatrix = viewer_ptr_.get()->getCameraManipulator()->getMatrix();
	      osg::Matrixd cameraRotation;
	      
	      cameraRotation.makeRotate(
					osg::DegreesToRadians(0.0), osg::Vec3(0,1,0), // roll
					osg::DegreesToRadians(0.0), osg::Vec3(1,0,0) , // pitch
					osg::DegreesToRadians(-droll), osg::Vec3(0,0,1) ); // heading

	      osg::Matrixd aNewViewMatrix = cameraRotation * aViewMatrix;
	      viewer_ptr_.get()->getCameraManipulator()->setByMatrix(aNewViewMatrix);
	      return false;
	      break;
	    }

          case 'c':
            {
	      osg::Matrixd aViewMatrix0(0.393369,0.917324,0.0614601,0,
					-0.230706,0.0337793,0.972437,0,
					0.889964,-0.396706,0.22492,0,
					0.364105,-0.393902,0.680456,1);

	      osg::Matrixd aViewMatrix1(0.121563,0.99252,0.0112091,0,
					-0.118254,0.00326934,0.992978,0,
					0.985514,-0.122035,0.117767,0,
					7.17417,-0.107913,1.12234,1);
              osg::Matrixd aViewMatrix2(-0.988063,-0.153967,0.00499288,0,
					0.0106572,-0.0359862,0.999295,0,
					-0.153678,0.987421,0.0371975,0,
					-2.17096,8.09489,1.20741,1);
	      osg::Matrixd aViewMatrix3(-0.997808,-0.0612453,-0.0250599,0,
					-0.0198733,-0.0838741,0.996278,0,
					-0.0631193,0.994592,0.0824732,0,
					-0.171284,3.21756,0.985355,1);
              static unsigned int roundrobin=0;
              if (roundrobin==0)
                { viewer_ptr_.get()->getCameraManipulator()->setByMatrix(aViewMatrix0); }
              else if (roundrobin==1)
                { viewer_ptr_.get()->getCameraManipulator()->setByMatrix(aViewMatrix1); }
              else if (roundrobin==2)
                { viewer_ptr_.get()->getCameraManipulator()->setByMatrix(aViewMatrix2); }
              else if (roundrobin==3)
                { viewer_ptr_.get()->getCameraManipulator()->setByMatrix(aViewMatrix3); }

              std::cout << "roundrobin:" << roundrobin<<std::endl;
              roundrobin++;
              if (roundrobin==4)
                roundrobin=0;
              return false;
            }
            break;

          case 'p':
            {
              osg::Matrixd aViewMatrix;
              aViewMatrix = viewer_ptr_->getCameraManipulator()->getMatrix();  
              double *ptrVM = aViewMatrix.ptr();
              std::cout << ptrVM[0] << "," <<   ptrVM[1] << "," <<  ptrVM[2] << "," <<  ptrVM[3] << "," << std::endl
                        << ptrVM[4] << "," <<   ptrVM[5] << "," <<  ptrVM[6] << "," <<  ptrVM[7] << "," << std::endl
                        << ptrVM[8] << "," <<   ptrVM[9] << "," << ptrVM[10] << "," << ptrVM[11] << "," << std::endl
                        << ptrVM[12] << "," << ptrVM[13] << "," << ptrVM[14] << "," << ptrVM[15] << "," << std::endl;
              return false;
            }
            break;

          case osgGA::GUIEventAdapter::KEY_Up:
            {
	      
              osg::Matrixd aViewMatrix;
              aViewMatrix = viewer_ptr_->getCameraManipulator()->getMatrix();  
	      osg::Matrixd aTranslationMatrix(1.0,0.0,0.0,0.0,
					      0.0,1.0,0.0,0.0,
					      0.0,0.0,1.0,0.0,
					      0.0,0.0,-dforward,1.0);
	      osg::Matrixd aNewViewMatrix = aTranslationMatrix*aViewMatrix;
	      viewer_ptr_.get()->getCameraManipulator()->setByMatrix(aNewViewMatrix);
	      return false;
            }
          case osgGA::GUIEventAdapter::KEY_Down:
            {
              osg::Matrixd aViewMatrix;
              aViewMatrix = viewer_ptr_->getCameraManipulator()->getMatrix();  
	      osg::Matrixd aTranslationMatrix(1.0,0.0,0.0,0.0,
					      0.0,1.0,0.0,0.0,
					      0.0,0.0,1.0,0.0,
					      0.0,0.0,dforward,1.0);
	      osg::Matrixd aNewViewMatrix = aTranslationMatrix*aViewMatrix;
	      viewer_ptr_.get()->getCameraManipulator()->setByMatrix(aNewViewMatrix);
	      return false;
            }
            break;

          default:
            std::cout << " a key has been pressed" << std::endl;
            return false;
          } 
      }
    default:
      return false;
    }
}
