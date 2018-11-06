from time import sleep
from threading import Thread
from sys import exit
from random import randint

from pydbus import SessionBus
from gi.repository import GLib

from pydbus.generic import signal

service_name = 'com.ravenops.rviz.LockStep'

loop = GLib.MainLoop()

class LockStep(object):
    __doc__ = """
        <node>
            <interface name='{}'>
            <method name='Kill'/>
            <method name='Seek'>
                <arg type='d' name='atTimeSeconds'    direction='in'/>
                <arg type='d' name='totalTimeSeconds' direction='out'/>
            </method>
            <method name='Read'>
                <arg type='d' name='durationSeconds' direction='in'/>
                <arg type='d' name='lastTimeSeconds' direction='out'/>
                <arg type='s' name='errors'          direction='out'/>
            </method>
            <property name="CurrentTime" type="d" access="read">
                <annotation name="org.freedesktop.DBus.Property.EmitsChangedSignal" value="true"/>
            </property>
            </interface>
        </node>
    """.format(service_name) 

    def Kill(self):
        loop.quit()

    def Seek(self, atTimeSeconds):
        self._currentTime = min(atTimeSeconds, self._currentTime)
        return self._totalTime
    
    def Read(self, durationSeconds):
        start = self._currentTime
        end = min(self._currentTime + durationSeconds, self._totalTime)

        #Fake messages
        event_count = randint(1,100)
        print('Playing ros bag time from {} to {} with {} messages'.format(start,end, event_count))
        duration = end - start
        time_slice = duration / event_count

        lastTime = start
        for i in range(event_count):
            lastTime = i*time_slice + start
            # self.PropertiesChanged(service_name, {"CurrentTime": self.CurrentTime}, [])
            print('Processed event {} at {}'.format(i, lastTime))
            sleep(time_slice) # just to see in the viewer
        self._currentTime = end # cause there might be a gap between last event time and actually window end.

        print('Finished ros bag time from {} to {}'.format(start,end))
        return lastTime #last event time

    def __init__(self):
        self._currentTime = 0
        self._totalTime = randint(5,120)

    # @property
    # def CurrentTime(self):
    #     return self._currentTime

    # @CurrentTime.setter
    # def CurrentTime(self, value):
    #     print('Current time hard set from {} to {}'.format(self._currentTime, value))
    #     self._currentTime = value
    #     self.PropertiesChanged(service_name, {"CurrentTime": self.CurrentTime}, [])

    PropertiesChanged = signal()

lock_step = LockStep()

def startService():
    session_bus = SessionBus()
    session_bus.publish(service_name, lock_step)
    loop.run()
    print "finished service thread"

def main():
    thread = Thread(target = startService)
    thread.start()

    thread.join()
    print "finished force set...exiting"
    exit(0)

if __name__ == "__main__":
    main()
    # dbus-send --session --print-reply --type=method_call --dest='com.ravenops.rviz.LockStep' '/com/ravenops/rviz/LockStep' com.ravenops.rviz.LockStep.PlayFrameWidth double:3 double:5.125
    # dbus-send --session --print-reply --type=method_call --dest='com.ravenops.rviz.LockStep' '/com/ravenops/rviz/LockStep' com.ravenops.rviz.LockStep.Kill
