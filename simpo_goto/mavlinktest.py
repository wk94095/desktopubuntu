from pymavlink import mavutil

the_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14560')

the_connection.wait_heartbeat()
print("Heartbeat from system(sysyem %u component %u)" % (the_connection.target_system, the_connection.target_component))
