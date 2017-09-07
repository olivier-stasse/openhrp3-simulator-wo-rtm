# standard modules
import sys
# omniORB
import CORBA
# CORBA naming service
import CosNaming

# generated stubs
import Controller_idl

# check to make sure we ve got good arguments
if len(sys.argv) < 2:
  print "Usage:"
  print "python tiltplatecontroller_client.py <Identifier>"
  print "<Identifier> is an accession number to retrieve the sequence for."
  sys.exit()

# Now start up the ORB and get the server
sys.argv.extend(("-ORBInitRef", "NameService=corbaname::localhost"))
orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)

# get the naming service
name_service_obj = orb.resolve_initial_references("NameService")
name_service_root = name_service_obj._narrow(CosNaming.NamingContext)
assert name_service_root is not None, "Failed to narrow to NamingContext."

# set up the name of the TiltPlateController server
tiltplatecontroller_name = [CosNaming.NameComponent("TiltPlateController","")]

# get the server object from the naming service
try:
  tiltplatecontroller_server = name_service_root.resolve(tiltplatecontroller_name)
except CosNaming.NamingContext.NotFound, msg:
  print "Could not find TiltPlateController object"
  sys.exit(1)

# Try to query the accession, and catch errors if they occur
try:
  tiltplatecontroller_server.start()
  tiltplatecontroller_server.stop()
except :
  print "pb will calling tiltplatecontroller_server" 
