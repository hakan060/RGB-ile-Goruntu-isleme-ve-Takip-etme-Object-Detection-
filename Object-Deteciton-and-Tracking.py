import subprocess
import sys
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
import math
import cv2
import numpy as np



dispW = 640
dispH = 480
flip = 2
camSet = 'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(
    flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
cam = cv2.VideoCapture(camSet)

'''
21. satirda kamera bağlantisi sağlanmistir
'''
ucanAracimizinAdi = connect('/dev/ttyUSB0', wait_ready=False, baud=57600)
home_point = ucanAracimizinAdi.location.global_relative_frame


'''
DroneKit kütüphanesi kullanarak Döner Kanatlı aracımızın anlık olarak bilgilerine erişebiliyoruz
'''

print("ucanAracimizinAdi arac baglantisi saglandi.")
print()
print('Lokasyon: %s' % home_point)
print('GPS Durumu: %s' % ucanAracimizinAdi.gps_0)
print('Zemin Hizi: %s' % ucanAracimizinAdi.groundspeed)
print('Hava Hizi: %s' % ucanAracimizinAdi.airspeed)
print('Batarya Durumu: %s' % ucanAracimizinAdi.battery)
print('EKF Durumu: %s' % ucanAracimizinAdi.ekf_ok)
print('Sistem Durumu: %s' % ucanAracimizinAdi.system_status.state)
print('Mod: %s' % ucanAracimizinAdi.mode.name)

'''
is_armable komutu ile döner kanatli aracimizin çalışmasını bekliyoruz
'''
while not ucanAracimizinAdi.is_armable:
    print("ARM edilmeyi bekliyor...")
    time.sleep(1)

'''
aracimizi arm ediyoruz ve kanatlar dönmeye başlıyor
'''
def set_arm():
    print("ARM etme basladi...")
    ucanAracimizinAdi.mode = VehicleMode("GUIDED")
    ucanAracimizinAdi.armed = True

    while not ucanAracimizinAdi.armed:
        print("ARM edilmeyi bekliyor...")
        time.sleep(1)

    print("ARM etme basarili!")
    print()
    '''
    take off komutu ile havadan kaç metre yuksege cikmasini istiyorsak parametreye onu yaziyoruz
    '''
    take_off(12)


def take_off(aTargetAltitude):
    print("Havalanma basladi...")
    ucanAracimizinAdi.simple_takeoff(aTargetAltitude)
    while True:
        print("Yukseklik: ",
              ucanAracimizinAdi.location.global_relative_frame.alt)
        if ucanAracimizinAdi.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Hedef yukseklige ulasildi!")
            print("Hava Hizi 2 olarak ayarlandi!")
            ucanAracimizinAdi.groundspeed = 3
            ucanAracimizinAdi.airspeed = 2
            '''
            go_to_target komutu ile IHA mızın hangi konumlarda ucus yapacagini seciyoruz
            '''
            go_to_target(40.2326092, 29.0089609)
            go_to_target(40.2326062, 29.0092009)
            go_to_target(40.2328928, 29.0092063)
            go_to_target(40.2328897, 29.0089702)
            break
        time.sleep(1)


def get_location_metres(original_location, targetLat, targetLon):
    earth_radius = 6378137.0
    dLat = targetLat/earth_radius
    dLon = targetLon/(earth_radius*math.cos(math.pi*original_location.lat/180))
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation = LocationGlobal(newlat, newlon, 5)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation = LocationGlobalRelative(
            newlat, newlon, 5)
    else:
        raise Exception("Yanlis lokasyon objesi gonderildi.")

    return targetlocation


def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def go_to_target(targetLat, targetLon):
    currentLocation = ucanAracimizinAdi.location.global_relative_frame
    targetLocation = LocationGlobalRelative(targetLat, targetLon, 12)
    targetDistance = get_distance_metres(currentLocation, targetLocation)

    print("Rotaya Gidiliyor: " +
          str(targetLocation) + ", " + str(targetDistance))

    i = 0
    newDestination = 0
    counter = 0
    while ucanAracimizinAdi.mode.name == "GUIDED":
        ret, frame = cam.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        '''
        RGB ile alt sinir ve ust sinir renkleri belirlenerek uygun renkleri bulmaya calisiyoruz
        '''
        lower_brown = np.array([6, 63, 0])
        upper_brown = np.array([23, 255, 81])
        mask = cv2.inRange(hsv, lower_brown, upper_brown)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(
            contours, key=lambda x: cv2.contourArea(x), reverse=True)
        merkezNoktasi = cv2.putText(frame, "merkez", (320, 240),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        if len(contours) != 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            (x, y, w, h) = cv2.boundingRect(c)
            if area >= 25:
                if i == 0:
                    catchTarget = ucanAracimizinAdi.location.global_relative_frame
                    i += 1
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
                cv2.putText(frame, "center", (cX - 20, cY - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 3)
                if cX < 320 and cY < 240:
                    newLocation = get_location_metres(
                        ucanAracimizinAdi.location.global_relative_frame, 2, -2)
                    newRemainingDistance = get_distance_metres(
                        catchTarget, newLocation)
                    if newRemainingDistance < 20:
                        newDestination = 1
                        print("Hedef goruldu takip ediliyor...")
                        ucanAracimizinAdi.simple_goto(newLocation)
                        print(newRemainingDistance)
                    else:
                        ucanAracimizinAdi.simple_goto(catchTarget)
                elif cX > 320 and cY < 240:
                    newLocation = get_location_metres(
                        ucanAracimizinAdi.location.global_relative_frame, 2, 2)
                    newRemainingDistance = get_distance_metres(
                        catchTarget, newLocation)
                    if newRemainingDistance < 20:
                        newDestination = 1
                        print("Hedef goruldu takip ediliyor...")
                        ucanAracimizinAdi.simple_goto(newLocation)
                        print(newRemainingDistance)
                    else:
                        ucanAracimizinAdi.simple_goto(catchTarget)
                elif cX > 320 and cY > 240:
                    newLocation = get_location_metres(
                        ucanAracimizinAdi.location.global_relative_frame, -2, 2)
                    newRemainingDistance = get_distance_metres(
                        catchTarget, newLocation)
                    if newRemainingDistance < 20:
                        newDestination = 1
                        print("Hedef goruldu takip ediliyor...")
                        ucanAracimizinAdi.simple_goto(newLocation)
                        print(newRemainingDistance)
                    else:
                        ucanAracimizinAdi.simple_goto(catchTarget)
                elif cX < 320 and cY > 240:
                    newLocation = get_location_metres(
                        ucanAracimizinAdi.location.global_relative_frame, -2, -2)
                    newRemainingDistance = get_distance_metres(
                        catchTarget, newLocation)
                    if newRemainingDistance < 20:
                        newDestination = 1
                        print("Hedef goruldu takip ediliyor...")
                        ucanAracimizinAdi.simple_goto(newLocation)
                        print(newRemainingDistance)
                    else:
                        ucanAracimizinAdi.simple_goto(catchTarget)
            else:
                if newDestination == 1:
                    ucanAracimizinAdi.simple_takeoff(12)
                    ucanAracimizinAdi.simple_goto(catchTarget)
                    print("Geri Donuluyor...")

                    catchRemaining = get_distance_metres(
                        ucanAracimizinAdi.location.global_relative_frame, catchTarget)
                    if catchRemaining <= 3:
                        newDestination = 0
                        i = 0
                else:
                    ucanAracimizinAdi.simple_goto(targetLocation)

                    remainingDistance = get_distance_metres(
                        ucanAracimizinAdi.location.global_relative_frame, targetLocation)
                    print("Hedefe Kalan Uzaklik: ",
                          remainingDistance)
                    if remainingDistance <= 3:
                        print("Rotaya varildi.")
                        break

            if cv2.waitKey(1) == ord('q'):
                break
        else:
            ucanAracimizinAdi.simple_goto(targetLocation)

            remainingDistance = get_distance_metres(
                ucanAracimizinAdi.location.global_relative_frame, targetLocation)
            print("Hedefe Kalan Uzaklik: ",
                  remainingDistance)
            if remainingDistance <= 3:
                print("Rotaya varildi.")
                break
        counter += 1


set_arm()

print("LAND mod ayarlaniyor...")
ucanAracimizinAdi.mode = VehicleMode("LAND")

print("Cihaz Disconnect ediliyor...")

cam.release()
cv2.destroyAllWindows()
ucanAracimizinAdi.close()
sys.exit()
