/* Copyright CNRS 2106
 * Author: Olivier Stasse
 */

#include "olv-link-info.hh"

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

