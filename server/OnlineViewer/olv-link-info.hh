#ifndef _OLV_LINK_INFO_HH_
#define _OLV_LINK_INFO_HH_

#include <gepetto/viewer/group-node.h>

#include <hrpModel/ModelLoaderUtil.h>

using namespace graphics;

class OLVLinkInfo
{
public:
  std::vector<graphics::NodePtr_t> list_of_shapes;
  graphics::GroupNodePtr_t grp_id;
  graphics::GroupNodePtr_t grp_snd_id;

  void set_shape_transform_matrix(OpenHRP::TransformedShapeIndex & a_trans_shape_id,
				  unsigned int id_int_list_of_shapes,
				  bool isZCylinder=false);  
  void set_link_transform_matrix(OpenHRP::LinkInfo & aLinkInfo);
  int m_Debug;

  // Constructor
  OLVLinkInfo();

};

#endif /* _OLV_LINK_INFO_HH_ */
