#include <loadproject.hh>

using namespace OpenHRP;
using namespace std;
using namespace hrp;

#if 1
#define ODEBUG(X)
#else
#define ODEBUG(X) X
#endif

#define ODEBUG3(X)

template <typename X, typename X_ptr>
X_ptr checkCorbaServer(std::string n, CosNaming::NamingContext_var &cxt)
{
  CosNaming::Name ncName;
  ncName.length(1);
  ncName[0].id = CORBA::string_dup(n.c_str());
  ncName[0].kind = CORBA::string_dup("");
  X_ptr srv = NULL;
  try {
    srv = X::_narrow(cxt->resolve(ncName));
  } catch(const CosNaming::NamingContext::NotFound &exc) {
    std::cerr << n << " not found: ";
    switch(exc.why) {
    case CosNaming::NamingContext::missing_node:
      std::cerr << "Missing Node" << std::endl;
    case CosNaming::NamingContext::not_context:
      std::cerr << "Not Context" << std::endl;
      break;
    case CosNaming::NamingContext::not_object:
      std::cerr << "Not Object" << std::endl;
      break;
    }
    return (X_ptr)NULL;
  } catch(CosNaming::NamingContext::CannotProceed &exc) {
    std::cerr << "Resolve " << n << " CannotProceed" << std::endl;
  } catch(CosNaming::NamingContext::AlreadyBound &exc) {
    std::cerr << "Resolve " << n << " InvalidName" << std::endl;
  }
  return srv;
}

std::ostream & operator<< (std::ostream &os, const CollisionPairItem &aCP) 
{
  return aCP.display(os);
}

std::ostream & operator<< (std::ostream &os, const SimulationItem &aCP) 
{
  return aCP.display(os);
}

std::ostream & operator<< (std::ostream &os, const ModelItem &aCP) 
{
  return aCP.display(os);
}

LoadProject::LoadProject()
{
}

LoadProject::~LoadProject()
{
}


void LoadProject::AnalyzeAttr(xmlAttr * an_Attr,int depth)
{
  ODEBUG(std::cout << "node type: Attr, name: " << an_Attr->name
	 << std::enl);
      
  AnalyzeElementNames(an_Attr->children,depth+1);

}

void extractBoolFromAttr(xmlAttr *attr_node, bool & abool)
{
  const std::string content = (const char *)attr_node->children->content;
  if (content=="true")
    abool = true;
  else
    abool = false;
  ODEBUG3(std::cout << "Extracted bool value: " << content << " " << abool << std::endl);
}

void extractDoubleFromAttr(xmlAttr *attr_node, double & aReal)
{
  const std::string content = (const char *)attr_node->children->content;
  istringstream iss(content);
  iss >> aReal;
  ODEBUG3(std::cout << "Extracted double value: " << aReal << std::endl);
    
}

																				     
void extractDoubleVectorFromAttr(xmlAttr *attr_node, std::vector<double> & advec,unsigned int size)
{
  const std::string content = (const char *)attr_node->children->content;
  istringstream iss(content);
  if ((advec.size()!=size) && (size>0))
    advec.resize(size);

  for (unsigned int i=0;i<size;i++)
    {
      iss >> advec[i];
      ODEBUG3(std::cout << "Extracted double value: " << advec[i] << std::endl);
    }
    
}

void extractStringFromAttr(xmlAttr *attr_node, std::string & aString)
{
  const std::string content = (const char *)attr_node->children->content;
  aString = content;
  ODEBUG3(std::cout << "Extracted string value: " << aString << std::endl);
    
}

void extractIntegrationMethodFromAttr(xmlAttr *attr_node, 
				      integration_method & amethod)
{
  const std::string content = (const char *)attr_node->children->content;
  if (content=="RUNGE_KUTTA")
    {
      amethod = OpenHRP::DynamicsSimulator::RUNGE_KUTTA;
    }
  else    if (content=="EULER")
    {
      amethod = OpenHRP::DynamicsSimulator::EULER;
    }
}


void LoadProject::AnalyzeSimulationItem(xmlNode * a_node,
					std::string &name,
					std::string &select)
{
  ODEBUG(std::cout << "AnalyzeSimulationItem " << std::endl);
  xmlAttr *attr_node = NULL;
  xmlNode * cur_node;
  
  simulationData_->name = name;
  if (select=="true")
    simulationData_->select = true;
  else simulationData_->select = false;

  for (cur_node = a_node; cur_node; cur_node = cur_node->next) 
    {
      
      ODEBUG(std::cout << "ASI node name:" << cur_node->name <<std::endl);
      if (cur_node->type == XML_TEXT_NODE) 
	{
	  ODEBUG(printf("ASI node type: Text, value: %s\n", cur_node->name));
	  ODEBUG(printf("ASI node type: Text, content: %s\n", cur_node->content));
	}
      else 
	{
	  // This a property
	  if (cur_node->type == XML_ELEMENT_NODE) 
	    {
	      std::string currentSubFieldName;
	      // For all attributes of the properties
	      for(attr_node = cur_node->properties; attr_node; attr_node = attr_node->next)
		{
		  const std::string attrName = (const char *) attr_node->name;
		  ODEBUG(std::cout << "ASI Attribute of " << cur_node->name << " : " << attrName <<std::endl);
		  
		  //  If the attribute is the name of a SimulationItem Subfield.
		  if (attrName=="name")
		    {
		      const std::string attrContent = (const char *) attr_node->children->content;
		      currentSubFieldName = attrContent;
		      ODEBUG(std::cout << "currentSubdFieldName = " << currentSubFieldName << std::endl);
		    }
		  else if (attrName =="value")
		    {
		      if (currentSubFieldName=="integrate") extractBoolFromAttr(attr_node,simulationData_->integrate);
		      else if (currentSubFieldName=="viewsimulate") extractBoolFromAttr(attr_node,simulationData_->viewSimulate);
		      else if (currentSubFieldName=="timeStep") extractDoubleFromAttr(attr_node,simulationData_->timeStep);
		      else if (currentSubFieldName=="totalTime") extractDoubleFromAttr(attr_node,simulationData_->totalTime);
		      else if (currentSubFieldName=="realTime") extractBoolFromAttr(attr_node,simulationData_->realTime);
		      else if (currentSubFieldName=="gravity") extractDoubleFromAttr(attr_node,simulationData_->gravity);
		      else if (currentSubFieldName=="method") extractIntegrationMethodFromAttr(attr_node,simulationData_->method);
		    }
		}
	    }
	}
    }
  ODEBUG(std::cout << "Display simulation data" << std::endl);
  ODEBUG(std::cout << *simulationData_.get() << std::endl);
}

void LoadProject::AnalyzeModelItem(xmlNode * a_node,
				   std::string &name,
				   std::string &select,
				   std::string &url)
{
  ODEBUG3(std::cout << "AnalyzeModelItem " << std::endl);
  xmlAttr *attr_node = NULL;
  ModelItem aModelItem;
  aModelItem.name = name;
  if (select=="true")
    aModelItem.select = true;
  else aModelItem.select = false;
  aModelItem.url = url;

  xmlNode * cur_node;

  for (cur_node = a_node; cur_node; cur_node = cur_node->next) 
    {
      
      ODEBUG3(std::cout << "AMI node name:" << cur_node->name <<std::endl);
      if (cur_node->type == XML_TEXT_NODE) 
	{
	  ODEBUG(printf("ASI node type: Text, value: %s\n", cur_node->name));
	  ODEBUG(printf("ASI node type: Text, content: %s\n", cur_node->content));
	}
      else 
	{
	  // This a property
	  if (cur_node->type == XML_ELEMENT_NODE) 
	    {
	      std::string currentSubFieldName;
	      // For all attributes of the properties
	      for(attr_node = cur_node->properties; attr_node; attr_node = attr_node->next)
		{
		  const std::string attrName = (const char *) attr_node->name;
		  ODEBUG3(std::cout << "AMI Attribute of " << cur_node->name << " : " << attrName <<std::endl);
		  
		  //  If the attribute is the name of a ModelItem Subfield.
		  if (attrName=="name")
		    {
		      const std::string attrContent = (const char *) attr_node->children->content;
		      currentSubFieldName = attrContent;
		      ODEBUG3(std::cout << "currentSubdFieldName = " << currentSubFieldName << std::endl);
		    }
		  else if (attrName =="value")
		    {
		      if (currentSubFieldName=="isRobot") extractBoolFromAttr(attr_node,aModelItem.isRobot);
		      else if (currentSubFieldName=="markRadius") extractDoubleFromAttr(attr_node,aModelItem.markRadius);
		      else if (currentSubFieldName=="controller") extractStringFromAttr(attr_node,aModelItem.controllerName);
		      else
			{
			  PropertyModelItem apropModelItem;
			  apropModelItem.name = currentSubFieldName;
			  const std::string valueContent= (const char *)attr_node->children->content;
			  apropModelItem.value = valueContent;
			  aModelItem.listOfVariableProperties.push_back(apropModelItem);
			  ODEBUG3(std::cout << "Variable property:" << currentSubFieldName << " " << valueContent<< std::endl);
			}
		    }
		}
	    }
	}
    }  

  /// Expand the list of model items
  aModelItem.fillBodyValues();
  aListOfModelItems_->push_back(aModelItem);
  ODEBUG(std::cout << aModelItem << std::endl);
}

void LoadProject::AnalyzeCollisionPairItem(xmlNode * a_node)
{
  ODEBUG3(std::cout << "AnalyzeCollisionPairItem " << std::endl);
  xmlAttr *attr_node = NULL;
  CollisionPairItem aCollisionPairItem;
  xmlNode * cur_node;

  for (cur_node = a_node; cur_node; cur_node = cur_node->next) 
    {
      
      ODEBUG3(std::cout << "ACPI node name:" << cur_node->name <<std::endl);
      if (cur_node->type == XML_TEXT_NODE) 
	{
	  ODEBUG(printf("ASI node type: Text, value: %s\n", cur_node->name));
	  ODEBUG(printf("ASI node type: Text, content: %s\n", cur_node->content));
	}
      else 
	{
	  // This a property
	  if (cur_node->type == XML_ELEMENT_NODE) 
	    {
	      std::string currentSubFieldName;
	      // For all attributes of the properties
	      for(attr_node = cur_node->properties; attr_node; attr_node = attr_node->next)
		{
		  const std::string attrName = (const char *) attr_node->name;
		  ODEBUG3(std::cout << "ACPI Attribute of " << cur_node->name << " : " << attrName <<std::endl);
		  
		  //  If the attribute is the name of a ModelItem Subfield.
		  if (attrName=="name")
		    {
		      const std::string attrContent = (const char *) attr_node->children->content;
		      currentSubFieldName = attrContent;
		      ODEBUG3(std::cout << "currentSubdFieldName = " << currentSubFieldName << std::endl);
		    }
		  else if (attrName =="value")
		    {
		      if (currentSubFieldName=="slidingFriction") extractDoubleFromAttr(attr_node,aCollisionPairItem.slidingFriction);
		      if (currentSubFieldName=="staticFriction") extractDoubleFromAttr(attr_node,aCollisionPairItem.staticFriction);
		      if (currentSubFieldName=="cullingThresh") extractDoubleFromAttr(attr_node,aCollisionPairItem.cullingThresh);
		      if (currentSubFieldName=="springDamperModel") extractBoolFromAttr(attr_node,aCollisionPairItem.springDamperModel);
		      if (currentSubFieldName=="jointName1") extractStringFromAttr(attr_node,aCollisionPairItem.jointName1);
		      if (currentSubFieldName=="jointName2") extractStringFromAttr(attr_node,aCollisionPairItem.jointName2);
		      if (currentSubFieldName=="objectName1") extractStringFromAttr(attr_node,aCollisionPairItem.objectName1);
		      if (currentSubFieldName=="objectName2") extractStringFromAttr(attr_node,aCollisionPairItem.objectName2);
		      if (currentSubFieldName=="springConstant") extractDoubleVectorFromAttr(attr_node,aCollisionPairItem.springConstant,6);
		      if (currentSubFieldName=="damperConstant") extractDoubleVectorFromAttr(attr_node,aCollisionPairItem.damperConstant,6);
		    }
		}
	    }
	}
    }
  /// Expand the list of CollisionPairItem
  aListOfCollisionPairItems_->push_back(aCollisionPairItem);
  ODEBUG(std::cout << aCollisionPairItem << std::endl);
}

void LoadProject::AnalyzeItem(xmlNode * cur_node)
{
  std::string ItemName,ItemClass,ItemSelect,ItemURL;
	  
  xmlAttr *attr_node = NULL;
  // Find the class.
  for(attr_node = cur_node->properties; attr_node; attr_node = attr_node->next)
    {
	      
      const std::string attrName = (const char *) attr_node->name;
      if (attrName=="class")
	{
	  const std::string class_content = (const char *)attr_node->children->content;
	  ODEBUG(std::cout << "Class value: " << class_content << std::endl);
	  ItemClass = class_content;
	}
      else if (attrName=="name")
	{
	  const std::string name_content = (const char *)attr_node->children->content;
	  ODEBUG(std::cout << "name value: " << name_content << std::endl);
	  ItemName = name_content;
	}
      else if (attrName=="select")
	{
	  const std::string select_content = (const char *)attr_node->children->content;
	  ODEBUG(std::cout << "select value: " << select_content << std::endl);
	  ItemSelect = select_content;
	}
      else if (attrName=="url")
	{
	  const std::string url_content = (const char *)attr_node->children->content;
	  ODEBUG(std::cout << "url value: " << url_content << std::endl);
	  ItemURL = url_content;
	}
    }

  // Analyze according to class name
  if (ItemClass=="com.generalrobotix.ui.item.GrxSimulationItem")
    {
      AnalyzeSimulationItem(cur_node->children,ItemName,ItemSelect);
    }
  else if (ItemClass=="com.generalrobotix.ui.item.GrxModelItem")
    {
      AnalyzeModelItem(cur_node->children,ItemName,ItemSelect,ItemURL);
    }
  else if (ItemClass=="com.generalrobotix.ui.item.GrxCollisionPairItem")
    {
      AnalyzeCollisionPairItem(cur_node->children);
    }
  else 
    { std::cerr << "Unable to find class: " << ItemClass << std::endl; }

}

void LoadProject::AnalyzeElementNames(xmlNode * a_node,int depth)
{
  xmlNode *cur_node = NULL;

  ODEBUG3(std::cout << "AEN - depth: "<< depth << std::endl;)
  for (cur_node = a_node; cur_node; cur_node = cur_node->next) {

    if (cur_node->type == XML_ELEMENT_NODE) 
      {
	ODEBUG(printf("node type: Element, name: %s\n", cur_node->name));
	
	// Check if this is an item
	const std::string nodeName= (const char *)cur_node->name;
	if (nodeName=="item") 
	  AnalyzeItem(cur_node);
	else 
	  {
	    xmlAttr *attr_node = NULL;
	    // Find the class.
	    for(attr_node = cur_node->properties; 
		attr_node; 
		attr_node = attr_node->next)
	      {
		AnalyzeAttr(attr_node,depth);
	      }
	    AnalyzeElementNames(cur_node->children,depth+1);
	  }
      }
    else
      {
	if (cur_node->type == XML_ATTRIBUTE_NODE) {
	  ODEBUG(printf("node type: Attribute, name: %s\n", cur_node->name));
	  
	}
	if (cur_node->type == XML_TEXT_NODE) {
	  ODEBUG(printf("node type: Text, value: %s\n", cur_node->name));
	  ODEBUG(printf("node type: Text, content: %s\n", cur_node->content));
	}
	
	AnalyzeElementNames(cur_node->children,depth+1);
      }
  }
  ODEBUG3(std::cout << "AEN - depth: "<< depth << std::endl);
}

void LoadProject::loadProject(std::string &projectfilename_,
			      std::shared_ptr<SimulationItem> simulationData,
			      std::shared_ptr<std::vector<ModelItem> > aListOfModelItems,
			      std::shared_ptr<vector<CollisionPairItem> > aListOfCollisionPairItems)
{
  xmlDoc *doc = NULL;
  xmlNode *root_element = NULL; 
  simulationData_ = simulationData;
  aListOfModelItems_ = aListOfModelItems;
  aListOfCollisionPairItems_ = aListOfCollisionPairItems;
  /*
   * this initialize the library and check potential ABI mismatches
   * between the version it was compiled for and the actual shared
   * library used.
   */
  LIBXML_TEST_VERSION
    
    /*parse the file and get the DOM */
    doc = xmlReadFile(projectfilename_.c_str(), NULL, 0);
  
  if (doc == NULL) {
    std::cerr<< "error: could not parse file " << projectfilename_
	     << std::endl;
  }
  
  /*Get the root element node */
  root_element = xmlDocGetRootElement(doc);

  AnalyzeElementNames(root_element,0);
  
  /*free the document */
  xmlFreeDoc(doc);
  
  /*
   *Free the global variables that may
   *have been allocated by the parser.
   */
  xmlCleanupParser();
  
}

void ModelItem::fillBodyValues()
{
  for(unsigned int i=0;i<listOfVariableProperties.size()
	;i++)
    {
      std::string label=listOfVariableProperties[i].name;
      std::size_t found  = label.find('.');
      if (found != std::string::npos)
	{
	  // Extract joint Name
	  std::string jointName = label.substr(0,found);
	  // Extract field name
	  std::string field = label.substr(found+1,label.length()-found -1);
	  // Put value of the field in istringstream for extraction.
	  istringstream iss(listOfVariableProperties[i].value);

	  if (field=="angle")
	    {
	      double oneAngle; iss >> oneAngle;
	      jointsMap[jointName].angle = oneAngle;
	    }
	  else if (field=="mode")
	    {
	      if (iss.str()=="Torque")
		jointsMap[jointName].mode=1;
	      else if (iss.str()=="HighGain")
		jointsMap[jointName].mode=0;
	    }
	  else if (field=="rotation")
	    {
	      jointsMap[jointName].rotation.resize(4);
	      for(unsigned int j=0;j<4;j++)
		iss >> jointsMap[jointName].rotation[j];
	    }
	  else if (field=="translation")
	    {
	      jointsMap[jointName].translation.resize(3);
	      for(unsigned int j=0;j<3;j++)
		iss >> jointsMap[jointName].translation[j];
	    }
	  else if (field=="velocity")
	    {
	      jointsMap[jointName].velocity.resize(3);
	      for(unsigned int j=0;j<3;j++)
		iss >> jointsMap[jointName].velocity[j];
	    }
	  else if (field=="angularVelocity")
	    {
	      jointsMap[jointName].angularVelocity.resize(3);
	      for(unsigned int j=0;j<3;j++)
		iss >> jointsMap[jointName].angularVelocity[j];
	    }
	}
    }
}

