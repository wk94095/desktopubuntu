import math
from dronekit import LocationGlobalRelative

def target(vehicle,gimbalangle):
    lat = vehicle.location.global_relative_frame.lat #無人機緯度座標
    lon = vehicle.location.global_relative_frame.lon #無人機經度座標
    alt = vehicle.location.global_relative_frame.alt
    print(vehicle.attitude.pitch) #顯示無人機pitch角度
    gimbal_angle = math.radians(gimbalangle) #雲台角度
    droneheading = math.radians(90-vehicle.heading) #飛機頭向
    height = alt #飛機高度
    distance = height*math.tan(gimbal_angle)
    print("離目標物距離:",distance)
    earth_radius=6378137.0 #地球半徑
    lat1 = distance*math.sin(droneheading)
    lon1 = distance*math.cos(droneheading)
    print(lat1,lon1)
    dlat = lat1/earth_radius
    dlon = lon1/(earth_radius*math.cos(math.pi*lat/180))

    newlat = lat + (dlat * 180/math.pi)
    newlon = lon + (dlon * 180/math.pi)
    print("new",newlat,newlon)

    distancelat = newlat - lat
    distancelon = newlon - lon
    get_distance_metres = math.sqrt((distancelat**2)+(distancelon**2))* 1.113195e5
    print("座標離目標物距離:",get_distance_metres)

    return LocationGlobalRelative(newlat, newlon, alt)