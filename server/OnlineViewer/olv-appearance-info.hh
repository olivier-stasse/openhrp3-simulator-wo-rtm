#ifndef _OLV_APPEARANCE_INFO_HH_
#define _OLV_APPEARANCE_INFO_HH_

typedef struct 
{
  long int materialIndex;
  std::vector<double> normals;
  std::vector<long> normalIndices;
  bool normalPerVertex;
  bool solid;
  float creaseAngle;
  std::vector<float> colors;

  std::vector<long> colorIndices;
  bool colorPerVertex;
  
  long int textureIndex;

  std::vector<float> textureCoordinate;
  std::vector<long> textureCoordIndices;
  float textransformMatrix[9];

} OLVAppearanceInfo;

#endif /* _OLV_APPEARANCE_INFO_HH_ */
