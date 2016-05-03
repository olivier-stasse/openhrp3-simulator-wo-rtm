
// Gepetto viewer headers
#include <gepetto/viewer/leaf-node-box.h>
#include <gepetto/viewer/leaf-node-cylinder.h>
#include <gepetto/viewer/leaf-node-cone.h>
#include <gepetto/viewer/leaf-node-sphere.h>

#include <gepetto/viewer/group-node.h>

// OpenHRP headers
#include <hrpCorba/OpenHRPCommon.hh>
#include "from-body-to-osg.hh"

FromBodyToOsg::FromBodyToOsg(CORBA::ORB_ptr orb)
  : orb_(CORBA::ORB::_duplicate(orb))
{
  
}


FromBodyToOsg::~FromBodyToOsg()
{

}

void FromBodyToOsg::load(GroupNodePtr_t & world,
			 const char *name,
			 const char *url)
{
  graphics::GroupNodePtr_t new_osg_obj;
  std::string obj_name(name);

  new_osg_obj= graphics::GroupNode::create(name);
  map_of_osgs_[obj_name] = new_osg_obj;
  world->addChild(map_of_osgs_[obj_name]);

  // Create callback
  FromBodyToOsgDataType * from_body_to_osg_data_type = 
    new FromBodyToOsgDataType(this);
  new_osg_obj->asGroup()->setUserData(from_body_to_osg_data_type);
  new_osg_obj->asGroup()->setUpdateCallback(new FromBodyToOsgCallback);

  OpenHRP::BodyInfo_var aBodyInfo;
  aBodyInfo = hrp::loadBodyInfo(url,orb_);
  
  std::cout << "Name " << aBodyInfo->name() << std::endl;
  std::cout << "Url: " << aBodyInfo->url() << std::endl;

  OpenHRP::StringSequence_var anInfoSeq = aBodyInfo->info();
  for(unsigned int id=0;id < anInfoSeq->length();id++)
    {
      CORBA::String_var anInfo = anInfoSeq[id];
      std::cout << anInfo << std::endl;
    }
  OpenHRP::LinkInfoSequence_var anLinkSeq = aBodyInfo->links();
  for(unsigned int id=0;id < anLinkSeq->length();id++)
    {
      OpenHRP::LinkInfo aLinkInfo = anLinkSeq[id];
      std::cout << aLinkInfo.name << std::endl;
    }
  
  OpenHRP::ShapeInfoSequence_var aShapeInfoSeq = aBodyInfo->shapes();
  for(unsigned int id=0;id < aShapeInfoSeq->length();id++)
    {
      OpenHRP::ShapeInfo aShapeInfo = aShapeInfoSeq[id];
      
      switch (aShapeInfo.primitiveType)
	{
	case OpenHRP::SP_MESH: 
	  {
	    std::cout << "SP_MESH" << std::endl;
	    std::cout << aShapeInfo.url << std::endl;
	    break;
	  }
	case OpenHRP::SP_BOX : 
	  {
	    // std::cout << "SP_BOX (" << std::endl;
	    osgVector3 half_axis;
	    for(unsigned int i=0;i<3;i++)
	      {
		half_axis[i] = aShapeInfo.primitiveParameters[i];
		//std::cout << half_axis[i] << ", ";
	      }
	    //	    std::cout << " ) " << std::endl;
	    std::string aStr("sp_box");
	    graphics::LeafNodeBoxPtr_t osgLNB =
	      graphics::LeafNodeBox::create(aStr,half_axis);

	    new_osg_obj->addChild(osgLNB);

	    osgVector3 position1(0.0,0.0,0.0);
	    osg::Matrix aMatrix(1.0, 0.0, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 1.0);
	    osgQuat attitude1;
	    attitude1.set(aMatrix);
	    osgLNB->applyConfiguration(position1,attitude1);
	    osgLNB->setVisibilityMode(graphics::VISIBILITY_ON);  

	    break;
	  }

	case OpenHRP::SP_CYLINDER:
	  {
	    std::cout << "SP_CYLINDER" << std::endl;
	    break;
	  }
	case OpenHRP::SP_CONE : 
	  {
	    std::cout << "SP_CONE" << std::endl;
	    break;
	  }
	case OpenHRP::SP_SPHERE : 
	  {
	    std::cout << "SP_SPHERE" << std::endl;
	    break;
	  }
	case OpenHRP::SP_PLANE : 
	  {
	    std::cout << "SP_PLANE" << std::endl;
	    break;
	  }
	}
    }

}

void FromBodyToOsg::update(const ::OpenHRP::WorldState & state)
{
  std::cout << "time: "<< state.time;

  const OpenHRP::CharacterPositionSequence & aCharPosSeq = 
    state.characterPositions;

  // Over the sequence of characters
  for(unsigned int idCharPos=0;
      idCharPos < aCharPosSeq.length(); 
      idCharPos++)
    {
      // Find the corresponding one inside the one.
      MapOfOsgs_it it_osg = 
	map_of_osgs_.find(std::string(aCharPosSeq[idCharPos].characterName));
      if (it_osg!=map_of_osgs_.end())
	{
	  graphics::GroupNodePtr_t  osg_obj= it_osg->second;

	  const OpenHRP::LinkPositionSequence & aLinkPosSeq = 
	    aCharPosSeq[idCharPos].linkPositions;
	  
	  // Iterate over LinkPositionSequence
	  for(unsigned int idLinkPos=0;
	      idLinkPos < aLinkPosSeq.length(); 
	      idLinkPos++)
	    {
	      const OpenHRP::LinkPosition & aLinkPos = aLinkPosSeq[idLinkPos];
	      osgVector3 position1(aLinkPos.p[0],
				   aLinkPos.p[1],
				   aLinkPos.p[2]);
	      
	      osg::Matrix aMatrix(aLinkPos.R[0], aLinkPos.R[1], aLinkPos.R[2], 0.0,
				  aLinkPos.R[3], aLinkPos.R[4], aLinkPos.R[5], 0.0,
				  aLinkPos.R[6], aLinkPos.R[7], aLinkPos.R[8], 0.0,
				  0.0, 0.0, 0.0, 1.0);
	      osgQuat attitude1;
	      attitude1.set(aMatrix);
	      
	      graphics::NodePtr_t aNodePtr = osg_obj->getChild(idLinkPos);
	      aNodePtr->applyConfiguration(position1,attitude1);
	      
	    }
	}
    }
}

void FromBodyToOsg::display()
{
  
  for (MapOfOsgs_it it_osg= map_of_osgs_.begin(); 
       it_osg!=map_of_osgs_.end(); 
       ++it_osg)
    {

      graphics::GroupNodePtr_t  osg_obj = it_osg->second;
      for(unsigned int i=0;i<osg_obj->getNumOfChildren();i++)
	{

	  graphics::NodePtr_t aNodePtr = osg_obj->getChild(i);
	  aNodePtr->setVisibilityMode(graphics::VISIBILITY_ON);  
	}
    }
}
