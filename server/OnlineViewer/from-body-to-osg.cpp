/* Copyright CNRS 2106
 * Author: Olivier Stasse
 */
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
  m_Debug=0;
}


FromBodyToOsg::~FromBodyToOsg()
{

}

void FromBodyToOsg::display_appearance(OpenHRP::AppearanceInfo & an_appearance)
{
  std::cout << "materialIndex: " << an_appearance.materialIndex << std::endl;
  
  std::cout << "Nb of normals: " << an_appearance.normals.length() << std::endl;
  
  if (0)
    {
      for(unsigned int id_normal_idx=0;
	  id_normal_idx<an_appearance.normals.length();
	  id_normal_idx+=3)
	std::cout << "(" << an_appearance.normals[id_normal_idx] <<" , "
		  << an_appearance.normals[id_normal_idx+1] <<" , "
	      << an_appearance.normals[id_normal_idx+2] <<") ";
      std::cout << std::endl;
    }
  std::cout << "Nb of normalIndices: " 
	    << an_appearance.normalIndices.length() << std::endl;
  
  if (0)
    {
      for(unsigned int id_normal_idx=0;
	  id_normal_idx<an_appearance.normalIndices.length();
	  id_normal_idx++)
	std::cout << an_appearance.normalIndices[id_normal_idx]<<" ";
      std::cout << std::endl;
    }

  std::cout << "normalPerVertex: " 
	    << an_appearance.normalPerVertex << std::endl;
  
  std::cout << "solid: " 
	    << an_appearance.solid << std::endl;

  std::cout << "creaseAngle: " 
	    << an_appearance.creaseAngle << std::endl;

  std::cout << "colors: " 
	    << an_appearance.colors.length() << std::endl;
  
  std::cout << "colorPerVertex: " 
	    << an_appearance.colorPerVertex << std::endl;

  std::cout << "textureIndex: " << an_appearance.textureIndex << std::endl;
  
  std::cout << "textureCoordinate: " 
	    << an_appearance.textureCoordinate.length() << std::endl;

  std::cout << "textureCoordinateIndices: " 
	    << an_appearance.textureCoordIndices.length() << std::endl;
  
  
}

void FromBodyToOsg::display_material(OpenHRP::MaterialInfo & aMaterialInfo)
{
  std::cout << "ambientIntensity: " 
	    << aMaterialInfo.ambientIntensity << std::endl;
  
  std::cout << "diffuseColor: "
	    << aMaterialInfo.diffuseColor[0] << " "
	    << aMaterialInfo.diffuseColor[1] << " "
	    << aMaterialInfo.diffuseColor[2] << std::endl;

  std::cout << "emissiveColor: "
	    << aMaterialInfo.emissiveColor[0] << " "
	    << aMaterialInfo.emissiveColor[1] << " "
	    << aMaterialInfo.emissiveColor[2] << std::endl;  

  std::cout << "shininess: " 
	    << aMaterialInfo.shininess << std::endl;

  std::cout << "specularColor: "
	    << aMaterialInfo.specularColor[0] << " "
	    << aMaterialInfo.specularColor[1] << " "
	    << aMaterialInfo.specularColor[2] << std::endl;  

  std::cout << "transparency : "
	    << aMaterialInfo.transparency << std::endl;
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

  OpenHRP::ShapeInfoSequence_var aShapeInfoSeq = aBodyInfo->shapes();
  if (m_Debug)
    std::cout << "Number of shapes: " << aShapeInfoSeq->length()
	      << std::endl;

  OpenHRP::AppearanceInfoSequence_var anAppearanceInfoSeq = 
    aBodyInfo->appearances();
  
  //  if (m_Debug)
    {
      std::cout << "Number of appearances: " << anAppearanceInfoSeq->length()
		<< std::endl;
      
      for(unsigned int id_app=0;
	  id_app < anAppearanceInfoSeq->length();
	  id_app++)
	display_appearance(anAppearanceInfoSeq[id_app]);
    }

  OpenHRP::MaterialInfoSequence_var aMaterialInfoSeq = 
    aBodyInfo->materials();
  
  if (m_Debug)
    {
      std::cout << "Number of materials: " << aMaterialInfoSeq->length()
		<< std::endl;
      
      for(unsigned int id_mat=0;
	  id_mat < aMaterialInfoSeq->length();
	  id_mat++)
	display_material(aMaterialInfoSeq[id_mat]);
    }

  OpenHRP::TextureInfoSequence_var aTextureInfoSeq =
    aBodyInfo->textures();

  if (m_Debug)
    std::cout << "Number of textures: " << aTextureInfoSeq->length()
	      << std::endl;
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

OLVAppearanceInfo & FromBodyToOsg::
find_appearance(GeometricPrimitiveInsertParameters & aGPIP)
{
  //Specifying the yellow colour of the object
  OpenHRP::ShapeInfo * aShapeInfo = aGPIP.aShapeInfo;
  long int anAppearanceIndex = aShapeInfo->appearanceIndex;
  if (anAppearanceIndex>=0)
    {
      return aGPIP.an_olv_body_info->appearances[anAppearanceIndex];
    }
  return null_olv_appearance_info_;
}

OLVMaterialInfo & FromBodyToOsg::
find_material(GeometricPrimitiveInsertParameters & aGPIP)
{
  //Specifying the yellow colour of the object
  OpenHRP::ShapeInfo * aShapeInfo = aGPIP.aShapeInfo;
  long int anAppearanceIndex = aShapeInfo->appearanceIndex;
  if (anAppearanceIndex>=0)
    {
      OpenHRP::AppearanceInfoSequence_var anAppearanceInfoSeq = 
	aGPIP.aBodyInfo->appearances();
      
      OpenHRP::AppearanceInfo & anAppearanceInfo = 
	anAppearanceInfoSeq[anAppearanceIndex];
      long int aMaterialIndex = anAppearanceInfo.materialIndex;
      if (aMaterialIndex>=0)
	{

	  return aGPIP.an_olv_body_info->materials[aMaterialIndex];
	}
    }
  return null_olv_material_info_;
}

void FromBodyToOsg::insert_box(GeometricPrimitiveInsertParameters & aGPIP)
{
  osgVector3 half_axis;
  for(unsigned int i=0;i<3;i++)
    {
      half_axis[i] = (float)(aGPIP.aShapeInfo->primitiveParameters[i]/2.0);
    }
  if (m_Debug)
    std::cout << "Link " << aGPIP.name << " uses a box (" 
	      << half_axis[0] << "," <<  half_axis[1] << "," 
	      << half_axis[2] << ")" << std::endl;
  std::string aStr(aGPIP.name);
  graphics::LeafNodeBoxPtr_t osgLNB =
    graphics::LeafNodeBox::create(aStr,half_axis);
  
  aGPIP.an_olv_link_info->grp_snd_id->addChild(osgLNB);
  aGPIP.an_olv_link_info->list_of_shapes[aGPIP.id_in_list_of_shapes] = osgLNB;
  
  aGPIP.an_olv_link_info->set_shape_transform_matrix(*aGPIP.a_trans_shape_id,
						     aGPIP.id_in_list_of_shapes);

  // Specify color
  OLVMaterialInfo & aMaterialInfo = find_material(aGPIP);
  osgLNB->setColor(osg::Vec4(aMaterialInfo.diffuseColor[0],
			     aMaterialInfo.diffuseColor[1],
			     aMaterialInfo.diffuseColor[2],1.0f));
  
  osgLNB->setVisibilityMode(graphics::VISIBILITY_ON);  

}


void FromBodyToOsg::insert_cylinder(GeometricPrimitiveInsertParameters & aGPIP)
{

  float cylinderParameters[5];
  if (m_Debug)
    std::cout << "Link " << aGPIP.name << " uses a cylinder (";
  for(unsigned int i=0;i<5;i++)
    {
      cylinderParameters[i] = aGPIP.aShapeInfo->primitiveParameters[i];
      if (m_Debug)
	std::cout << cylinderParameters[i] << ", " ;
    }
  if (m_Debug)
    std::cout << ");"<< std::endl;

  std::string aStr(aGPIP.name);
  graphics::LeafNodeCylinderPtr_t osgLNB =
    graphics::LeafNodeCylinder::create(aStr,
				       cylinderParameters[0], // radius
				       cylinderParameters[1]  // height
				       );
  
  // Create the link between the osgID and the link Id.
  aGPIP.an_olv_link_info->grp_snd_id->addChild(osgLNB);
  aGPIP.an_olv_link_info->list_of_shapes[aGPIP.id_in_list_of_shapes] = osgLNB;
  aGPIP.an_olv_link_info->set_shape_transform_matrix(*aGPIP.a_trans_shape_id,
						     aGPIP.id_in_list_of_shapes,
						     true);
  
  // Specify color
  OLVMaterialInfo & aMaterialInfo = find_material(aGPIP);
  osgLNB->setColor(osg::Vec4(aMaterialInfo.diffuseColor[0],
			     aMaterialInfo.diffuseColor[1],
			     aMaterialInfo.diffuseColor[2],1.0f));
  
  osgLNB->setVisibilityMode(graphics::VISIBILITY_ON);  

}

void FromBodyToOsg::insert_mesh(GeometricPrimitiveInsertParameters &aGPIP)
{
  osg::Geode* mesh_geode = new osg::Geode();
  ::osg::GroupRefPtr agrpref_ptr = 
      osg::GroupRefPtr(aGPIP.an_olv_link_info->grp_snd_id->asGroup());
  agrpref_ptr->addChild(mesh_geode);

  osg::Geometry* mesh_geometry = new osg::Geometry();
  mesh_geode->addDrawable(mesh_geometry);
  
  /* TO DO 
  aGPIP.an_olv_link_info->list_of_shapes[aGPIP.id_in_list_of_shapes] = 
    mesh_geode;
  aGPIP.an_olv_link_info->set_shape_transform_matrix(*aGPIP.a_trans_shape_id,
						     aGPIP.id_in_list_of_shapes,
						     false);
  */

  osg::Vec3Array* mesh_vertices = new osg::Vec3Array;

  // Insert vertices 
  for(unsigned int id_vertices=0;
      id_vertices < aGPIP.aShapeInfo->vertices.length();
      id_vertices+=3)
    {
      mesh_vertices->push_back(osg::Vec3(aGPIP.aShapeInfo->vertices[id_vertices],
					 aGPIP.aShapeInfo->vertices[id_vertices+1],
					 aGPIP.aShapeInfo->vertices[id_vertices+2]));
    }
  mesh_geometry->setVertexArray( mesh_vertices ); 
  
  //if (m_Debug)
    std::cout << "ShapeInfo nb of triangles:" << aGPIP.aShapeInfo->triangles.length()/3 << std::endl;

  // Insert triangles / creates the surface.
  for(unsigned int id_triangles=0;
      id_triangles < aGPIP.aShapeInfo->triangles.length();
      id_triangles+=3)
    {
      osg::DrawElementsUInt* triangle_base = 
	new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
      
      triangle_base->push_back(aGPIP.aShapeInfo->triangles[id_triangles]);
      triangle_base->push_back(aGPIP.aShapeInfo->triangles[id_triangles+1]);
      triangle_base->push_back(aGPIP.aShapeInfo->triangles[id_triangles+2]);
      mesh_geometry->addPrimitiveSet(triangle_base); 
    }

  // Create Material and assign color.
  osg::ref_ptr<osg::StateSet> nodess (agrpref_ptr->getOrCreateStateSet());

  // Creating the material object
  osg::ref_ptr<osg::Material> mat (new osg::Material);
  
  // Finding the material in the list.
  OLVMaterialInfo & aMaterialInfo = find_material(aGPIP);
  aMaterialInfo.set_osg_material_info(mat);

  // Finding the appearance in the list.
  OLVAppearanceInfo & anAppearanceInfo = find_appearance(aGPIP);

  //Attaching the newly defined state set object to the node state set
  //mat->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
  mat->setColorMode(osg::Material::SPECULAR);
  nodess->setAttribute(mat.get());

  osg::Vec4Array* colors = new osg::Vec4Array;
  colors->push_back(osg::Vec4(aMaterialInfo.diffuseColor[0],
			      aMaterialInfo.diffuseColor[1],
			      aMaterialInfo.diffuseColor[2],1.0f));
  mesh_geometry->setColorArray(colors);
  mesh_geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
  
  // Create normals
  osg::Vec3Array* normals = new osg::Vec3Array;
  if (m_Debug)
    std::cout << "Normal indices:" << std::endl;
  for(unsigned int id_normal_idx=0;
      id_normal_idx < anAppearanceInfo.normalIndices.size();
      id_normal_idx++)
    {
      if (m_Debug)
	std::cout << "id_normal_idx:" << id_normal_idx << std::endl;
      unsigned int id_normal = anAppearanceInfo.normalIndices[id_normal_idx]*3;
      normals->push_back(osg::Vec3(anAppearanceInfo.normals[id_normal],
				   anAppearanceInfo.normals[id_normal+1],
				   anAppearanceInfo.normals[id_normal+2]));
    }
  if (m_Debug)
    std::cout << std::endl;
  mesh_geometry->setNormalArray(normals);
  if (anAppearanceInfo.normalPerVertex)
    mesh_geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
  else
    mesh_geometry->setNormalBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

  aGPIP.an_olv_link_info->grp_snd_id->setVisibilityMode(graphics::VISIBILITY_ON);  
}

void FromBodyToOsg::
extract_material_info(OpenHRP::BodyInfo_var & aBodyInfo,
		      OLVBodyInfo & an_olv_body_info)
{
  OpenHRP::MaterialInfoSequence_var aMaterialInfoSeq = 
    aBodyInfo->materials();
  
  an_olv_body_info.materials.resize(aMaterialInfoSeq->length());
  
  for(unsigned int id_mat=0;
      id_mat < aMaterialInfoSeq->length();
      id_mat++)
    {
      an_olv_body_info.materials[id_mat].ambientIntensity = 
	aMaterialInfoSeq[id_mat].ambientIntensity;

      an_olv_body_info.materials[id_mat].shininess = 
	aMaterialInfoSeq[id_mat].shininess;
      
      an_olv_body_info.materials[id_mat].transparency = 
	aMaterialInfoSeq[id_mat].transparency;

      for (unsigned int id_vec=0;id_vec<3;id_vec++)
	{
	   an_olv_body_info.materials[id_mat].diffuseColor[id_vec] = 
	    aMaterialInfoSeq[id_mat].diffuseColor[id_vec];
	   an_olv_body_info.materials[id_mat].emissiveColor[id_vec] = 
	    aMaterialInfoSeq[id_mat].emissiveColor[id_vec];
	   an_olv_body_info.materials[id_mat].specularColor[id_vec] = 
	     aMaterialInfoSeq[id_mat].specularColor[id_vec];
	}
    }
}

void FromBodyToOsg::
extract_appearance_info(OpenHRP::BodyInfo_var & aBodyInfo,
			OLVBodyInfo & an_olv_body_info)
{
  OpenHRP::AppearanceInfoSequence_var anAppearanceInfoSeq = 
    aBodyInfo->appearances();
  
  an_olv_body_info.appearances.resize(anAppearanceInfoSeq->length());
  
  for(unsigned int id_app=0;
      id_app < anAppearanceInfoSeq->length();
      id_app++)
    {

      OpenHRP::AppearanceInfo &anAppearanceInfo = 	
	anAppearanceInfoSeq[id_app];

      OLVAppearanceInfo & anOLVAppearanceInfo = 
	an_olv_body_info.appearances[id_app];
      
      anOLVAppearanceInfo.materialIndex = 
	anAppearanceInfo.materialIndex;

      /// Normals
      anOLVAppearanceInfo.normals.resize(anAppearanceInfo.normals.length());
      
      for(unsigned int id_normals=0;
	  id_normals<anAppearanceInfo.normals.length();
	  id_normals++)
	{
	  anOLVAppearanceInfo.normals[id_normals]=
	    anAppearanceInfo.normals[id_normals];
	}

      /// Normal Indices
      anOLVAppearanceInfo.normalIndices.
	resize(anAppearanceInfo.normalIndices.length());
      
      for(unsigned int id_normal_idx=0;
	  id_normal_idx<anAppearanceInfo.normalIndices.length();
	  id_normal_idx++)
	{
	  anOLVAppearanceInfo.normalIndices[id_normal_idx]=
	    anAppearanceInfo.normalIndices[id_normal_idx];
	}

      anOLVAppearanceInfo.normalPerVertex = 
	anAppearanceInfo.normalPerVertex;

      anOLVAppearanceInfo.solid = 
	anAppearanceInfo.solid;

      anOLVAppearanceInfo.creaseAngle = 
	anAppearanceInfo.creaseAngle;

      /// Colors
      anOLVAppearanceInfo.colors.resize(anAppearanceInfo.colors.length());

      for(unsigned int id_colors=0;
	  id_colors<anAppearanceInfo.colors.length();
	  id_colors++)
	{
	  anOLVAppearanceInfo.colors[id_colors]=
	    anAppearanceInfo.colors[id_colors];
	}
      
      anOLVAppearanceInfo.colorIndices.
	resize(anAppearanceInfo.colorIndices.length());

      for(unsigned int id_colordIdx=0;
	  id_colordIdx<anAppearanceInfo.colorIndices.length();
	  id_colordIdx++)
	{
	  anOLVAppearanceInfo.
	    colorIndices[id_colordIdx]=
	    anAppearanceInfo.colorIndices[id_colordIdx];
	}

      anOLVAppearanceInfo.colorPerVertex = anAppearanceInfo.colorPerVertex;
		
      /// Textures
      anOLVAppearanceInfo.textureIndex = 
	anAppearanceInfo.textureIndex;

      anOLVAppearanceInfo.textureCoordinate.
	resize(anAppearanceInfo.textureCoordinate.length());

      for(unsigned int id_texcoord=0;
	  id_texcoord<anAppearanceInfo.textureCoordinate.length();
	  id_texcoord++)
	{
	  anOLVAppearanceInfo.textureCoordinate[id_texcoord]=
	    anAppearanceInfo.textureCoordinate[id_texcoord];
	}

      anOLVAppearanceInfo.textureCoordIndices.
	resize(anAppearanceInfo.textureCoordIndices.length());

      for(unsigned int id_texcoordid=0;
	  id_texcoordid<anAppearanceInfo.textureCoordIndices.length();
	  id_texcoordid++)
	{
	  anOLVAppearanceInfo.textureCoordIndices[id_texcoordid]=
	    anAppearanceInfo.textureCoordIndices[id_texcoordid];
	}
      
      
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
	  
	  // Prepare 
	  GeometricPrimitiveInsertParameters aGPIP;
	  aGPIP.aShapeInfo = & aShapeInfo;
	  aGPIP.name = aLinkInfo.name;
	  aGPIP.an_olv_link_info = & an_olv_link_info;
	  aGPIP.a_trans_shape_id = & aLinkInfo.shapeIndices[id_shapeIdx];
	  aGPIP.id_in_list_of_shapes = id_shapeIdx;
	  aGPIP.aBodyInfo = aBodyInfo;
	  aGPIP.an_olv_body_info = & an_olv_body_info;

	  switch (aShapeInfo.primitiveType)
	    {
	    case OpenHRP::SP_MESH: 
	      {
		insert_mesh(aGPIP);
		break;
	      }
	      
	    case OpenHRP::SP_BOX : 
	      {
		insert_box(aGPIP);
		break;
	      }
	      
	    case OpenHRP::SP_CYLINDER:
	      {
		insert_cylinder(aGPIP);
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
  //  if (m_Debug)
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

  // Extract material info.
  extract_material_info(aBodyInfo,map_of_bodys_[obj_name]);

  // Extract appearance info.
  extract_appearance_info(aBodyInfo,map_of_bodys_[obj_name]);

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
