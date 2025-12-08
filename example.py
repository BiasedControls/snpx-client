from snpx_client import SnpxClient

client = SnpxClient(ip="127.0.0.2", connect_on_init=True)

while True:
    joints = client.j_pos.read()
    client.di.write([True, True], start_index=1)
    print(joints)