from snpx_client import SnpxClient, VariableTypes
import time

client = SnpxClient(ip="127.0.0.2", connect_on_init=True)
print("Connected")

# Read robot position
#joints = client.j_pos.read()

# Write to DIs
#client.di.write([True, True], start_index=1)

#Read system variable
#sys_var = client.read_sys_var("$ANGTOL[1]", VariableTypes.REAL)

#Set custom assignment for SNPX. Custom assignments are stored for the next time that variable is read or written.
#client.set_asg(asg_num = 1, var_name = "$ANGTOL[1]")
