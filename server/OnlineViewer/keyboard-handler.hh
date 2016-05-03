#ifndef KEYBOARD_HANDLER_H
#define KEYBOARD_HANDLER_H

#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <gepetto/viewer/config-osg.h>

class TestKeyboardEventHandler : public osgGA::GUIEventHandler
{
protected:
  osg::Group * tGroup_;
  ::osgViewer::ViewerRefPtr viewer_ptr_;

public:
  TestKeyboardEventHandler(osg::Group * aGroup)
  { tGroup_ = aGroup; }


  void setViewerRef(osgViewer::ViewerRefPtr aViewerRefPtr)
  { viewer_ptr_ = aViewerRefPtr;}

  virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
  
};


#endif
