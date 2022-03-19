import math
import random

from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
from adafruit_servokit import ServoKit
from water_pump import WaterPump
from detection import Detection

import sys

class Drone:
    def __init__(self, iha, su_alma_noktasi_lat, su_alma_noktasi_lon, su_alma_irtifa, sim=False):
        self.iha = iha
        self.sim = sim
        self.komut = iha.commands
        self.su_alma_noktasi_lat = su_alma_noktasi_lat
        self.su_alma_noktasi_lon = su_alma_noktasi_lon
        self.su_alma_irtifa = su_alma_irtifa

        self.su_alindi = False
        self.su_birakildi = False
        self.reset()


    def reset(self):
        self.su_alindi = False
        self.su_birakildi = False

    def takeoff(self, irtifa):
        while self.iha.is_armable is not True:
            print("İHA arm edilebilir durumda değil.")
            time.sleep(1)

        print("İHA arm edilebilir.")

        self.iha.mode = VehicleMode("GUIDED")

        self.iha.armed = True

        while self.iha.armed is not True:
            print("İHA arm ediliyor...")
            time.sleep(0.5)

        print("İHA arm edildi.")

        self.iha.simple_takeoff(irtifa)

        while self.iha.location.global_relative_frame.alt < irtifa * 0.9:
            print("İha hedefe yükseliyor.")
            time.sleep(1)

    def mod_degis(self,mod_adi: str = ""):
        while self.iha.mode != VehicleMode(mod_adi):
            print(f"iha {mod_adi} moda alınıyor.")
            self.iha.mode = VehicleMode(mod_adi)
            time.sleep(1)

    def su_alma_noktasi_bekle(self):
        su_alma_wp = wp(self.su_alma_noktasi_lat, self.su_alma_noktasi_lon, self.su_alma_irtifa)
        su_alma_wp_handler = wp_handler(iha.commands, su_alma_wp)

        while 1:
            if su_alma_noktasi_wp == self.komut.next - 1 and self.su_alindi is False:
                self.mod_degis("GUIDED")
                tmpWp = LocationGlobalRelative(su_alma_wp.lat, su_alma_wp.lon, su_alma_wp.alt)
                self.iha.simple_goto(tmpWp)
                while su_alma_wp_handler.get_distance_meters() > 0.5:
                    print("Gorev devam ediyor")
                    time.sleep(1)
                print("Su alma noktasina varildi.")
                return True
            else:
                return False

    def alcal(self,alt):
        while self.iha.location.global_relative_frame.alt > alt * 0.8:
            tmpLoc = LocationGlobalRelative(
                self.iha.location.global_relative_frame.lat,
                self.iha.location.global_relative_frame.lon,
                alt)

            self.iha.simple_goto(tmpLoc)
            time.sleep(1)
            print("alçalıyor..")
            if self.iha.location.global_relative_frame.alt < alt + 0.3:
                return True

    def yuksel(self,alt):
        while self.iha.location.global_relative_frame.alt < alt * 0.8:
            tmpLoc = LocationGlobalRelative(
                self.iha.location.global_relative_frame.lat,
                self.iha.location.global_relative_frame.lon,
                alt)

            self.iha.simple_goto(tmpLoc)
            time.sleep(1)
            print("yukseliyor..")
            if self.iha.location.global_relative_frame.alt > alt - 0.3:
                return True

    def su_al(self):
        # su motoru objesi oluşturduk
        water_pump = WaterPump(test=self.sim)

        # todo : modu guided moda al ve alçal

        # alçaldıktan sonra su almak için motoru çalıştır.
        water_pump.forward(20)
        # su alma islemi bitti.iha.mode = VehicleMode("GUIDED")
        # su pompasını kapat
        water_pump.stop()
        water_pump.cleanup()

        # dronekit ile dronu autoya al su bırakma noktasına gitsin.
        self.su_alindi = True

    def servo_calistir(self):
        if self.sim:
            time.sleep(random.randint(0, 4))
            print("Servo çalışması simule ediliyor.")
            return True

        if self.su_alindi and not self.su_birakildi:
            myKit = ServoKit(channels=16)
            myKit.servo[15].angle = 0
            time.sleep(2)
            myKit.servo[15].angle = 135
            time.sleep(1)
            self.su_birakildi = True
            return True

    def failsafe(self):
        if self.iha.mode == VehicleMode("POSHOLD"):
            print("görevden çıkılıyor...")
            sys.exit(1)

    def dronu_ortala(self):
        print("drone ortalanıyor")
        time.sleep(2)
        return True

class wp:
   def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt

class wp_handler:
    def __init__(self, mission, waypoint):
        self.mission = mission
        self.waypoint = waypoint

    def get_wp(self):
        return LocationGlobalRelative(self.waypoint.lat, self.waypoint.lon, self.waypoint.alt)

    def get_distance_meters(self):
        """
        returns the distance between drone's current position and target position
        """
        dlat = iha.location.global_relative_frame.lat - self.waypoint.lat
        dlong = iha.location.global_relative_frame.lon - self.waypoint.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

def goruntu_isleme_calistir(drone):
    det = Detection(drone.sim)
    while det.run() is not True:
        drone.failsafe()
        time.sleep(.2)
    return True



iha = connect("/dev/ttyACM0", wait_ready=True, baud=115200) # reel
# iha = connect("127.0.0.1:14550", wait_ready=True) #sim

su_alma_noktasi_lat = float(input("Su alma noktası (lat) : "))
su_alma_noktasi_lon = float(input("Su alma noktası (lon) : "))
su_alma_irtifa = float(input("Su alma irtifa : "))
su_alma_noktasi_wp = int(input("su alma noktasi wp : "))


if su_alma_noktasi_lat == 0 or "" and su_alma_noktasi_lon == 0 or "":
    print("su alma noktasi girmedin....")
    sys.exit(0)

print(f"su alma noktası lat = {su_alma_noktasi_lat} , lon = {su_alma_noktasi_lon}, alt = {su_alma_irtifa} olarak ayarlandı...")

drone = Drone(iha, su_alma_noktasi_lat, su_alma_noktasi_lon, su_alma_irtifa, sim=False)

drone.takeoff(13)
drone.mod_degis("AUTO")

while 1:
    drone.failsafe()
    try:
        if not drone.su_alindi:
            while not drone.su_alma_noktasi_bekle():
                drone.failsafe()
                time.sleep(.2)

            time.sleep(1)
            drone.su_al()
            drone.yuksel(13)
            time.sleep(2)
            drone.mod_degis("AUTO")

            su_birakma_tespit_edildi = goruntu_isleme_calistir(drone)

            if su_birakma_tespit_edildi:
                drone.mod_degis("GUIDED")
                drone.dronu_ortala()
                drone.alcal(4)
                drone.servo_calistir()
                time.sleep(1)
                drone.mod_degis("AUTO")

    except StopIteration as e:
        print(f"{e}")
    time.sleep(.2)
    if drone.su_alindi and drone.su_birakildi:
        print("Mission Completed. AUTO da devam edecek.")
        break