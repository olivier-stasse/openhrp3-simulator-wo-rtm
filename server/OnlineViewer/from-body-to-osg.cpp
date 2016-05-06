
// Gepetto viewer headers
#include <gepetto/viewer/leaf-node-box.h>
#include <gepetto/viewer/leaf-node-cylinder.h>
#include <gepetto/viewer/leaf-node-cone.h>
#include <gepetto/viewer/leaf-node-sphere.h>

#include <gepetto/viewer/group-node.h>

// OpenHRP headers
#include <hrpCorba/OpenHRPCommon.hh>
#include "from-body-to-osg.hh"

OLVLinkInfo::
OLVLinkInfo()
{
  m_Debug=0;
}

void OLVLinkInfo::
set_shape_transform_matrix(OpenHRP::TransformedShapeIndex & a_trans_shape_id,
			   unsigned int id_in_list_of_shapes, 
			   bool isZCylinder)
{
  OpenHRP::DblArray12 & aTransformMatrix = a_trans_shape_id.transformMatrix; 
  int shape_id =a_trans_shape_id.shapeIndex;

  osgVector3 position1((float)aTransformMatrix[3],
		       (float)aTransformMatrix[7],
		       (float)aTransformMatrix[11]);

  osg::Matrix ZtoY( 1.0, 0.0, 0.0, 0.0,
		    0.0, 0.0,-1.0, 0.0,
		    0.0, 1.0, 0.0, 0.0,
		    0.0, 0.0, 0.0, 1.0);

  osg::Matrix aMatrix(aTransformMatrix[0],
		      aTransformMatrix[4],
		      aTransformMatrix[8],
		      0.0,
		      // End of First line
		      aTransformMatrix[1],
		      aTransformMatrix[5],
		      aTransformMatrix[9],
		      0.0,
		      // End of Snd line
		      aTransformMatrix[2],
		      aTransformMatrix[6],
		      aTransformMatrix[10],
		      0.0,
		      // End of 3rd line
		      0.0, 0.0, 0.0, 1.0);
  /*  */

  osgQuat attitude1;
  if (!isZCylinder)
    {
      attitude1.set(aMatrix);
      if (m_Debug)
	std::cout << aMatrix(0,0) << " "
		  << aMatrix(0,1) << " "
		  << aMatrix(0,2) << std::endl
		  << aMatrix(1,0) << " "
		  << aMatrix(1,1) << " "
		  << aMatrix(1,2) << std::endl
		  << aMatrix(2,0) << " "
		  << aMatrix(2,1) << " "
		  << aMatrix(2,2) << std::endl;
    }
  else
    {
      osg::Matrix FinalMatrix = ZtoY * aMatrix;
      attitude1.set(FinalMatrix);
      if (m_Debug)
	std::cout << FinalMatrix(0,0) << " "
		  << FinalMatrix(0,1) << " "
		  << FinalMatrix(0,2) << std::endl
		  << FinalMatrix(1,0) << " "
		  << FinalMatrix(1,1) << " "
		  << FinalMatrix(1,2) << std::endl
		  << FinalMatrix(2,0) << " "
		  << FinalMatrix(2,1) << " "
		  << FinalMatrix(2,2) << std::endl; 

    }
  list_of_shapes[id_in_list_of_shapes]->applyConfiguration(position1,attitude1);
  
}

void OLVLinkInfo::set_link_transform_matrix(OpenHRP::LinkInfo & a_link_info)
{
  osgVector3 position1((float)a_link_info.translation[0],
		       (float)a_link_info.translation[1],
		       (float)a_link_info.translation[2]);

  osg::Matrix aMatrix=osg::Matrix::rotate( a_link_info.rotation[3], 
					   a_link_info.rotation[0], 
					   a_link_info.rotation[1],
					   a_link_info.rotation[2]);
  
  osgQuat attitude1;
  attitude1.set(aMatrix);
  grp_snd_id->applyConfiguration(position1,attitude1);
}

FromBodyToOsg::FromBodyToOsg(CORBA::ORB_ptr orb)
  : orb_(CORBA::ORB::_duplicate(orb))
{
  m_Debug=0;
}


FromBodyToOsg::~FromBodyToOsg()
{

}

void FromBodyToOsg::display_body(OpenHRP::BodyInfo_var aBodyInfo)
{
  // Analysis result of ModelLoader
  if (m_Debug)
    {
      std::cout << "Name " << aBodyInfo->name() << std::endl;
      std::cout << "Url: " << aBodyInfo->url() << std::endl;
    }

  OpenHRP::StringSequence_var anInfoSeq = aBodyInfo->info();
  for(unsigned int id=0;id < anInfoSeq->length();id++)
    {
      CORBA::String_var anInfo = anInfoSeq[id];
      if (m_Debug)
	std::cout << "Info: " << anInfo << std::endl;
    }

  OpenHRP::LinkInfoSequence_var aLinkSeq = aBodyInfo->links();
  for(unsigned int id=0;id < aLinkSeq->length();id++)
    {
      OpenHRP::LinkInfo aLinkInfo = aLinkSeq[id];
      display_link(aLinkInfo);
    }
}

void FromBodyToOsg::display_link(OpenHRP::LinkInfo & aLinkInfo)
{
  std::cout << "Link Name:  " << aLinkInfo.name << std::endl;
  std::cout << "jointID:    " << aLinkInfo.jointId << std::endl;
  std::cout << "jointValue: " << aLinkInfo.jointValue << std::endl;
  std::cout << "jointAxis:  " << aLinkInfo.jointAxis[0] << " "
	    << aLinkInfo.jointAxis[1] << " "
    	    << aLinkInfo.jointAxis[2] << std::endl;
  
  std::cout << "parentIndex:" << aLinkInfo.parentIndex << std::endl;
  std::cout << "translation:" << aLinkInfo.translation[0] << " "
	    << aLinkInfo.translation[1] << " "
    	    << aLinkInfo.translation[2] << std::endl;

  std::cout << "rotation:" << aLinkInfo.rotation[0] << " "
	    << aLinkInfo.rotation[1] << " "
    	    << aLinkInfo.rotation[2] << " "
	    << aLinkInfo.rotation[3] << std::endl;

}

void FromBodyToOsg::insert_box(OpenHRP::ShapeInfo &aShapeInfo,
			       const char *name,
			       OLVLinkInfo & an_olv_link_info,
			       OpenHRP::TransformedShapeIndex & a_trans_shape_id,
			       unsigned int id_in_list_of_shapes)
{
  osgVector3 half_axis;
  for(unsigned int i=0;i<3;i++)
    {
      half_axis[i] = (float)(aShapeInfo.primitiveParameters[i]/2.0);
    }
  if (m_Debug)
    std::cout << "Link " << name << " uses a box (" 
	      << half_axis[0] << "," <<  half_axis[1] << "," 
	      << half_axis[2] << ")" << std::endl;
  std::string aStr(name);
  graphics::LeafNodeBoxPtr_t osgLNB =
    graphics::LeafNodeBox::create(aStr,half_axis);
  
  an_olv_link_info.grp_snd_id->addChild(osgLNB);
  an_olv_link_info.list_of_shapes[ id_in_list_of_shapes] = osgLNB;
  
  an_olv_link_info.set_shape_transform_matrix(a_trans_shape_id,id_in_list_of_shapes);

  osgLNB->setVisibilityMode(graphics::VISIBILITY_ON);  

}

void FromBodyToOsg::insert_cylinder(OpenHRP::ShapeInfo &aShapeInfo,
				    const char *name,
				    OLVLinkInfo & an_olv_link_info,
				    OpenHRP::TransformedShapeIndex & a_trans_shape_id,
				    unsigned int id_in_list_of_shapes)
{

  float cylinderParameters[5];
  if (m_Debug)
    std::cout << "Link " << name << " uses a cylinder (";
  for(unsigned int i=0;i<5;i++)
    {
      cylinderParameters[i] = aShapeInfo.primitiveParameters[i];
      if (m_Debug)
	std::cout << cylinderParameters[i] << ", " ;
    }
  if (m_Debug)
    std::cout << ");"<< std::endl;

  std::string aStr(name);
  graphics::LeafNodeCylinderPtr_t osgLNB =
    graphics::LeafNodeCylinder::create(aStr,
				       cylinderParameters[0], // radius
				       cylinderParameters[1]  // height
				       );
  
  // Create the link between the osgID and the link Id.
  an_olv_link_info.grp_snd_id->addChild(osgLNB);
  an_olv_link_info.list_of_shapes[id_in_list_of_shapes] = osgLNB;
  an_olv_link_info.set_shape_transform_matrix(a_trans_shape_id,id_in_list_of_shapes,true);
  
  osgLNB->setVisibilityMode(graphics::VISIBILITY_ON);  

}

void FromBodyToOsg::insert_mesh(OpenHRP::ShapeInfo &aShapeInfo,
				const char *name,
				OLVLinkInfo & an_olv_link_info,
				OpenHRP::TransformedShapeIndex & a_trans_shape_idunsigned,
				unsigned int id_in_list_of_shapes)
{
  osg::Geode* mesh_geode = new osg::Geode();
  // new_osg_obj->asQueue()->addChild(mesh_geode);

  osg::Geometry* mesh_geometry = new osg::Geometry();
  
  osg::Vec3Array* mesh_vertices = new osg::Vec3Array;
  // Insert vertices 
  for(unsigned int id_vertices=0;
      id_vertices < aShapeInfo.vertices.length();
      id_vertices+=3)
    {
      mesh_vertices->push_back(osg::Vec3(aShapeInfo.vertices[id_vertices],
					 aShapeInfo.vertices[id_vertices+1],
					 aShapeInfo.vertices[id_vertices+2]));
    }
  mesh_geometry->setVertexArray( mesh_vertices ); 

  
  // Insert triangles / creates the surface.
  for(unsigned int id_triangles=0;
      id_triangles < aShapeInfo.triangles.length();
      id_triangles+=3)
    {
      osg::DrawElementsUInt* triangle_base = 
	new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
      
      triangle_base->push_back(aShapeInfo.triangles[id_triangles]);
      triangle_base->push_back(aShapeInfo.triangles[id_triangles+1]);
      triangle_base->push_back(aShapeInfo.triangles[id_triangles+2]);
      mesh_geometry->addPrimitiveSet(triangle_base); 
    }
}

void FromBodyToOsg::
analysis_shape_info_sequence(OpenHRP::BodyInfo_var & aBodyInfo,
			     OLVBodyInfo & an_olv_body_info)

{
  OpenHRP::LinkInfoSequence_var aLinkInfoSeq = aBodyInfo->links();
  if (aLinkInfoSeq->length()!=0)
    {
      an_olv_body_info.links.resize(aLinkInfoSeq->length());
      if (m_Debug)
	std::cout << "LinkSeq.size: " << aLinkInfoSeq->length() << std::endl;
    }

  OpenHRP::ShapeInfoSequence_var aShapeInfoSeq = aBodyInfo->shapes();

  // For each LinkInfo create the corresponding geometric model.
  for(unsigned int id=0;id < aLinkInfoSeq->length();id++)
    {
      OpenHRP::LinkInfo aLinkInfo = aLinkInfoSeq[id];

      // Build internal OLV data structure.
      OLVLinkInfo & an_olv_link_info = an_olv_body_info.links[id];

      // Build name
      std::string link_info_name(std::string(aLinkInfo.name));
      std::string link_sub_info_name(std::string("sub")+link_info_name);

      // Create the link group.
      an_olv_link_info.grp_id = graphics::GroupNode::create(link_info_name);
      // Create the local frame of the group
      an_olv_link_info.grp_snd_id = graphics::GroupNode::create(link_sub_info_name);
      //      an_olv_link_info.set_link_transform_matrix(aLinkInfo);
      an_olv_link_info.grp_id->addChild(an_olv_link_info.grp_snd_id);
      
      
      // Initialize the size of the map of child shapes.
      an_olv_link_info.list_of_shapes.resize(aLinkInfo.shapeIndices.length());

      // Add the link group in the osg tree of the character.
      an_olv_body_info.grp_id->addChild(an_olv_link_info.grp_id);

      if (m_Debug)
	std::cout << "For link " << aLinkInfo.name 
		  << " number of shapes is : " << aLinkInfo.shapeIndices.length()
		  << std::endl;
	
      // Iterate over the shapes related to the link
      for(unsigned int id_shapeIdx = 0;
	  id_shapeIdx < aLinkInfo.shapeIndices.length();
	  id_shapeIdx++)
	{
	  OpenHRP::ShapeInfo aShapeInfo = 
	    aShapeInfoSeq[aLinkInfo.shapeIndices[id_shapeIdx].shapeIndex];
	  
	  switch (aShapeInfo.primitiveType)
	    {
	    case OpenHRP::SP_MESH: 
	      {
		//insert_cylinder(aShapeInfo,aLinkSeq[id].name,new_osg_obj);
		break;
	      }
	      
	    case OpenHRP::SP_BOX : 
	      {
		insert_box(aShapeInfo,aLinkInfo.name,
			   an_olv_link_info,
			   aLinkInfo.shapeIndices[id_shapeIdx],
			   id_shapeIdx);
		break;
	      }
	      
	    case OpenHRP::SP_CYLINDER:
	      {
		insert_cylinder(aShapeInfo, aLinkInfo.name,
				an_olv_link_info,
				aLinkInfo.shapeIndices[id_shapeIdx],
				id_shapeIdx);
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
}

void FromBodyToOsg::load(GroupNodePtr_t & world,
			 const char *name,
			 const char *url)
{
  graphics::GroupNodePtr_t new_osg_obj;
  OLVBodyInfo an_olv_body_info;

  // Call ModelLoader CORBA service.
  OpenHRP::BodyInfo_var aBodyInfo;
  aBodyInfo = hrp::loadBodyInfo(url,orb_);

  // Display the result
  if (m_Debug)
    display_body(aBodyInfo);

  // Create object
  std::string obj_name = aBodyInfo->name();
  new_osg_obj= graphics::GroupNode::create(obj_name);

  // Update map of bodys.
  an_olv_body_info.grp_id = new_osg_obj;
  map_of_bodys_[obj_name] = an_olv_body_info;

  // and osg tree.
  world->addChild(new_osg_obj);

  // Create callback
  FromBodyToOsgDataType * from_body_to_osg_data_type = 
    new FromBodyToOsgDataType(this);
  new_osg_obj->asGroup()->setUserData(from_body_to_osg_data_type);
  new_osg_obj->asGroup()->setUpdateCallback(new FromBodyToOsgCallback);
  
  // Create related Osg objects
  analysis_shape_info_sequence(aBodyInfo,
			       map_of_bodys_[obj_name]);
}

void FromBodyToOsg::update(const ::OpenHRP::WorldState & state)
{
  if (m_Debug)
    std::cout << "time: "<< state.time <<std::endl;

  const OpenHRP::CharacterPositionSequence & aCharPosSeq = 
    state.characterPositions;

  // Over the sequence of characters
  for(unsigned int idCharPos=0;
      idCharPos < aCharPosSeq.length(); 
      idCharPos++)
    {
      // Find the corresponding character inside the map.
      MapOfBodys_it it_osg = 
	map_of_bodys_.find(std::string(aCharPosSeq[idCharPos].characterName));

      if (it_osg!=map_of_bodys_.end())
	{
	  if (m_Debug)
	    std::cout << "Found " << std::string(aCharPosSeq[idCharPos].characterName)
		      << " : " <<it_osg->second.links.size()  << std::endl;

	  OLVBodyInfo & an_olv_body_info = it_osg->second;
	  
	  const OpenHRP::LinkPositionSequence & aLinkPosSeq = 
	    aCharPosSeq[idCharPos].linkPositions;
	  
	  // Iterate over LinkPositionSequence
	  for(unsigned int idLinkPos=0;
	      idLinkPos< aLinkPosSeq.length(); 
	      idLinkPos++)
	    {
	      const OpenHRP::LinkPosition & aLinkPos = aLinkPosSeq[idLinkPos];
	      
	      osgVector3 position1((float)aLinkPos.p[0],
				   (float)aLinkPos.p[1],
				   (float)aLinkPos.p[2]);
	      
	      osg::Matrix aMatrix(aLinkPos.R[0], aLinkPos.R[3], aLinkPos.R[6],0.0,
				  aLinkPos.R[1], aLinkPos.R[4], aLinkPos.R[7],0.0,
				  aLinkPos.R[2], aLinkPos.R[5], aLinkPos.R[8],0.0,
				  0.0,0.0,0.0,1.0);
	      osgQuat attitude1;
	      attitude1.set(aMatrix);

	      if (m_Debug)
		{
		  std::cout << "Position of " << idLinkPos 
			    << " " 
			    << (float)aLinkPos.p[0] << " " 
			    << (float)aLinkPos.p[1] << " "
			    << (float)aLinkPos.p[2] << std::endl;
		  std::cout << " " 
			    << aLinkPos.R[0] << " " << aLinkPos.R[1] << " "<< aLinkPos.R[2] << std::endl
			    << aLinkPos.R[3] << " " << aLinkPos.R[4] << " " << aLinkPos.R[5] << std::endl
			    << aLinkPos.R[6] << " " << aLinkPos.R[7] << " " << aLinkPos.R[8] << std::endl;
		}
	      graphics::NodePtr_t aNodePtr = 
		an_olv_body_info.links[idLinkPos].grp_id;
	      aNodePtr->applyConfiguration(position1,attitude1);
	    }
	}
    }
}

void FromBodyToOsg::display()
{
  
  for (MapOfBodys_it it_body= map_of_bodys_.begin(); 
       it_body!=map_of_bodys_.end(); 
       ++it_body)
    {

      graphics::GroupNodePtr_t  osg_obj = it_body->second.grp_id;
      for(unsigned int i=0;i<osg_obj->getNumOfChildren();i++)
	{

	  graphics::NodePtr_t aNodePtr = osg_obj->getChild(i);
	  aNodePtr->setVisibilityMode(graphics::VISIBILITY_ON);  
	}
    }
}
