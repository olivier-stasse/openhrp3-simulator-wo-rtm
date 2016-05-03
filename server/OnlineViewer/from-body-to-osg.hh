#ifndef _FROM_BODY_TO_OSG_HH_
#define _FROM_BODY_TO_OSG_HH_

// STL 
#include <map>

// Osg headers
#include <osgWidget/ViewerEventHandlers>
#include <osgDB/WriteFile>

// OpenHRP headers
#include <hrpModel/ModelLoaderUtil.h>

// Gepetto headers
#include <gepetto/viewer/group-node.h>

using namespace graphics;

typedef std::map<std::string,graphics::GroupNodePtr_t> MapOfOsgs;
typedef std::map<std::string,graphics::GroupNodePtr_t>::iterator MapOfOsgs_it;

class FromBodyToOsg
{
public:
  FromBodyToOsg(CORBA::ORB_ptr orb);
  ~FromBodyToOsg();

  // Read a body description through corba
  // using name and url parameters.
  void load(GroupNodePtr_t & world,
	    const char *name, 
	    const char * url);

  // Update the links position from the current world state.
  void update(const ::OpenHRP::WorldState & state);

  void display();

protected:

  /** ORB  */
  CORBA::ORB_var orb_;

  /** Map of objects */
  MapOfOsgs map_of_osgs_;
};

class FromBodyToOsgDataType : public osg::Referenced
{
protected:
  FromBodyToOsg * from_body_to_osg_;

public:
  FromBodyToOsgDataType(FromBodyToOsg * n)
  {
    from_body_to_osg_ = n;
  }

  void display()
  {
    if (from_body_to_osg_!=0)
      from_body_to_osg_->display();
  }
};

class FromBodyToOsgCallback : public osg::NodeCallback 
{
public:
  virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
  {
    osg::ref_ptr<FromBodyToOsgDataType> from_body_to_osg_data = 
      dynamic_cast<FromBodyToOsgDataType*> (node->getUserData() );
    if(from_body_to_osg_data)
      {
        from_body_to_osg_data->display();
      }
    traverse(node, nv); 
  }
};

#endif /* _FROM_BODY_TO_OSG_HH_ */
