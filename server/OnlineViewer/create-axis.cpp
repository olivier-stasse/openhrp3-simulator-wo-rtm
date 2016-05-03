#include <gepetto/viewer/leaf-node-cylinder.h>
#include "create-axis.hh"

void createAxis(graphics::GroupNodePtr_t &world)
{
  using namespace graphics;
  float radius=(float)0.01,height=(float)1.0;


  // Create axis
  // Z Axis - Red.
  osgVector3 position1(0.0,0.0,0.5);
  osgQuat attitude1;
  attitude1.makeRotate(0.0,0.0,0.0,0.0);
  LeafNodeCylinderPtr_t cylinder1 = LeafNodeCylinder::create("cylinder1", radius,height);
  cylinder1->applyConfiguration(position1,attitude1);
  cylinder1->setColor(osgVector4(1.0,0.0,0.0,1.0));
  
  // Y Axis - Green.
  osgVector3 position2(0.0,0.25,0.0);
  osgQuat attitude2;
  attitude2.makeRotate(M_PI/2,1.0,0.0,0.0);
  LeafNodeCylinderPtr_t cylinder2 = LeafNodeCylinder::create("cylinder2", radius,height);
  cylinder2->applyConfiguration(position2,attitude2);
  cylinder2->setColor(osgVector4(0.0,1.0,0.0,1.0));
  
  // X Axis - Blue.
  osgVector3 position3(0.25,0.,0.0);
  osgQuat attitude3;
  attitude3.makeRotate(M_PI/2,0.0,1.0,0.0);
  LeafNodeCylinderPtr_t cylinder3 = LeafNodeCylinder::create("cylinder", radius,height);
  cylinder3->applyConfiguration(position3,attitude3);
  cylinder3->setColor(osgVector4(0.0,0.0,1.0,1.0));

  world->addChild(cylinder1);
  world->addChild(cylinder2);
  world->addChild(cylinder3);
}
