from snpx_client import SnpxClient

client = SnpxClient(connect_on_init=True)

joints = client.j_pos.read()

print(joints)