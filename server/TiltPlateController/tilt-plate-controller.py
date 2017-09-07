#!/usr/bin/env python
# Import the Stub of Controller IDL
import sys,os
import CORBA, Controller_idl

# Implements the tilt-plate-controller interface as a CORBA server
class TiltPlateControllerServer_i (Controller_idl._0_OpenHRP__POA.Controller):
    def __init__(self):
        self.q=[]

    def setDynamicsSimulator(self, dynamicsSimulator):
        self.dynamicsSimulator = dynamicsSimulator
        
    def setViewSimulator(self, viewSimulator):
        self.viewSimulator = viewSimulator

    def setModelName(self,modelName):
        self.modelName = modelName

    def start(self):
        print("start")
    
    def initialize(self):
        print("initialize")
    
    def control(self):
        print("control")

    def input(self):
        print("input")
        modelName="tiltplate"
        linkName="WAIST_P"
        linkDataType=Controller_idl._0_OpenHRP.DynamicsSimulator.JOINT_VALUE
        self.q = self.dynamicsSimulator.getCharacterLinkData(modelName,linkName,linkDataType)
        print "q="+str(self.q)
    def output(self):
        print("output")
        modelName="tiltplate"
        linkName="WAIST_P"
        linkDataType=Controller_idl._0_OpenHRP.DynamicsSimulator.JOINT_TORQUE
        torquetosend=[5.0]
        self.dynamicsSimulator.setCharacterLinkData(modelName,linkName,linkDataType,torquetosend)


    def stop(self):
        print("stop")
    
    def destroy(self):
        print("destroy")
    
    def setTimeStep(self,timeStep):
        print("timestep" + str(timeStep))

orb= CORBA.ORB_init(sys.argv)
poa = orb.resolve_initial_references("RootPOA")

servant = TiltPlateControllerServer_i()
poa.activate_object(servant)


print orb.object_to_string(servant._this())

import CosNaming
name_service_obj = orb.resolve_initial_references("NameService")
name_service_root = name_service_obj._narrow(CosNaming.NamingContext)
assert name_service_root is not None, "Failed to narrow to NamingContext."


tiltplatecontroller_obj= servant._this()
tiltplatecontroller_name= [CosNaming.NameComponent("TiltPlateController","")]

try:
    name_service_root.bind(tiltplatecontroller_name,tiltplatecontroller_obj)
    print "Bound the TiltPlateController to the naming service."
except CosNaming.NamingContext.AlreadyBound, msg:
    print "TiltPlateController already bound, rebinding the new object."
    name_service_root.rebind(tiltplatecontroller_name, tiltplatecontroller_obj)

poa._get_the_POAManager().activate()
orb.run()




