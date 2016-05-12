#ifndef _OLV_MATERIAL_INFO_HH_
#define _OLV_MATERIAL_INFO_HH_

struct OLVMaterialInfo
{
  float ambientIntensity;
  float diffuseColor[3];
  float emissiveColor[3];
  float shininess;
  float specularColor[3];
  float transparency;

  void set_osg_material_info(osg::ref_ptr<osg::Material> &amat)
  {
    amat->setDiffuse(osg::Material::FRONT_AND_BACK,
		     osg::Vec4(diffuseColor[0],
			       diffuseColor[1],
			       diffuseColor[2],1.0f));
    amat->setEmission(osg::Material::FRONT_AND_BACK,
		      osg::Vec4(emissiveColor[0],
				emissiveColor[1],
				emissiveColor[2],1.0f));

    amat->setSpecular(osg::Material::FRONT_AND_BACK,
		      osg::Vec4(specularColor[0],
				specularColor[1],
				specularColor[2],1.0f));

    amat->setTransparency(osg::Material::FRONT_AND_BACK,
			  transparency);

    amat->setShininess(osg::Material::FRONT_AND_BACK,
		       shininess);
    
  }
};

#endif /* _OLV_MATERIAL_INFO_HH_ */
