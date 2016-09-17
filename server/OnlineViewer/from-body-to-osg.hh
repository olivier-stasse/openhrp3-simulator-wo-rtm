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

// OnLineViewer headers
#include "olv-link-info.hh"
#include "olv-material-info.hh"
#include "olv-appearance-info.hh"

using namespace graphics;

typedef struct 
{
  graphics::GroupNodePtr_t grp_id;

  /* Map of materials. */
  std::vector<OLVMaterialInfo> materials;

  /* Map of appearances. */
  std::vector<OLVAppearanceInfo> appearances;

  /* Map from osgId to jointId */
  std::vector<OLVLinkInfo> links;  

} OLVBodyInfo;

typedef struct
{
  OpenHRP::ShapeInfo * aShapeInfo;
  const char *name;
  OLVLinkInfo * an_olv_link_info;
  OpenHRP::TransformedShapeIndex * a_trans_shape_id;
  unsigned int id_in_list_of_shapes;
  OpenHRP::BodyInfo_var aBodyInfo;
  OLVBodyInfo * an_olv_body_info;
} GeometricPrimitiveInsertParameters;

typedef std::map<std::string,OLVBodyInfo> MapOfBodys;
typedef std::map<std::string,OLVBodyInfo>::iterator MapOfBodys_it;

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
  MapOfBodys map_of_bodys_;

  /** Display body */
  void display_body(OpenHRP::BodyInfo_var aBodyInfo);
  
  /** Display link */
  void display_link(OpenHRP::LinkInfo & aLinkInfo);

  /** Display material */
  void display_material(OpenHRP::MaterialInfo & aMaterialInfo);

  /** Display appareance */
  void display_appearance(OpenHRP::AppearanceInfo & an_appearance);
  
  /** @name Analysis and insert shapes @{*/
  
  void insert_box(GeometricPrimitiveInsertParameters & aGPIP);

  void insert_cylinder(GeometricPrimitiveInsertParameters & aGPIP);

  void insert_mesh(GeometricPrimitiveInsertParameters & aGPIP);

  void analysis_shape_info_sequence(OpenHRP::BodyInfo_var & aBodyInfo,
				    OLVBodyInfo & an_olv_body_info);
  /* @} */

  void extract_material_info(OpenHRP::BodyInfo_var & aBodyInfo,
			     OLVBodyInfo & an_olv_body_info);

  void extract_appearance_info(OpenHRP::BodyInfo_var & aBodyInfo,
			       OLVBodyInfo & an_olv_body_info);

  /** Appearance and material related methods */
  OLVMaterialInfo & find_material(GeometricPrimitiveInsertParameters & aGPIP);
  OLVAppearanceInfo & find_appearance(GeometricPrimitiveInsertParameters & aGPIP);
  
  /* NIL value for material info. */
  OLVMaterialInfo null_olv_material_info_;
  
  /* NIL value for appearance info. */
  OLVAppearanceInfo null_olv_appearance_info_;
  int m_Debug;

  // Avoid reetrance
  bool busy_;
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
