from dronekit import connect, VehicleMode
import getch

vehicle = connect('127.0.0.1:14560', wait_ready=True)

while True:
    if ord(getch.getch()) in [68, 100]:
        vehicle.mode = VehicleMode("GUIDED")
    else:
        vehicle.mode = VehicleMode("AUTO")
