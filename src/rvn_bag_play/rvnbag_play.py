#! /usr/bin/env python2.7
"""
Raven[ player ]

Iain Brookshaw
2018.10.30
Copyright (c) 2018, Raven Ops Inc., All Rights Reserved

Raven Rosbag player, using D-Bus to communicate to the RvnRviz renderer

RVN::FIX: This has to become a ROS Package, because ROS!!!
"""

# General Python Imports
import time
import argparse
import threading
from enum import Enum

# ROS Imports
import rosbag
import rospy

# D-Bus imports
from pydbus         import SessionBus
from gi.repository  import GLib
from pydbus.generic import signal

# define the dbus name and ROS publication node names
service_name  = 'com.ravenops.rviz.LockStep'
ros_node_name = 'rvn_bag_play'

class ExitStatus(Enum):
    OK           = 0
    BAD_DESERIAL = 3
    BAD_BAG      = 4
    PUB_FAIL     = 5



# ----------------------------------------------------------------------------------------------------------------------
class MessageContainer(object):
    """
    It is much more convenient to have a message holder class than try and untangle the 
    tupple supplied
    """
    def __init__(self, topic, raw_msg=None, t=rospy.Time()):
        self.topic           = topic
        self.msg_class       = raw_msg[-1]
        self.raw_msg_data    = raw_msg[1]
        self.time_of_bagging = t


    def deserialize( self ):
        """ create an instance of the message class and return it """
        if not self.raw_msg_data or not self.msg_class:
            raise Exception("either raw msg or class are invalid.\n\traw_msg: {}\n\tmsg_class: {}".format(self.raw_msg_data,self.msg_class))
            return

        try:
            cls = self.msg_class()
        except Exception as e:
            raise Exception("Unable to create an instance of the message class: {}".format(e))

        try:
            msg = cls.deserialize(self.raw_msg_data)
        except Exception as e:
            raise Exception("Unable to deserialize raw message data into message: {}".format(e))

        return msg


    def msg_in_time_window( self, start, end ):
        """ returns t/f if the message bagging timestamp within the window """
        if start <= self.time_of_bagging and self.time_of_bagging <= end:
            return True
        else:
            return False


# # ----------------------------------------------------------------------------------------------------------------------

# OLD FRAME
# class Frame(object):
#     """ All the messages in a single frame -- storage struct. Messages are MessageContainer classes"""

#     def __init__( self, start, end ):
#         self.all_msgs   = [] # all message containers
#         self.last_clock = None
#         self.start      = start
#         self.end        = end
#         print "have frame from {} to {}".format(self.start.to_sec(), self.end.to_sec())


#     def add_msg(self, topic, raw_msg, t):
#         msg_cont = MessageContainer(topic, raw_msg=raw_msg, t=t)
#         self.all_msgs.append(msg_cont)
#         if topic == "/clock":
#             self.last_clock = msg_cont



# ----------------------------------------------------------------------------------------------------------------------
class Frame:
    def __init__( self ):
        self._lock = threading.Lock()
        self._msgs = []
        self._complete = False

    def add_msgs( self, msg ):
        self._lock.acquire()
        self._msgs.append(msg)
        self._lock.release()

    def clear( self ):
        self._lock.acquire()
        self._msgs = []
        self._lock.release()

    def set_complete( self ):
        self._lock.acquire()
        self._complete = True
        self._lock.release()

    def is_complete( self ):
        comp = False
        self._lock.acquire()
        comp = self._complete
        self._lock.release()
        return comp

    def get_next_msg( self ):
        msg = None
        self._lock.acquire()
        for m in self._msgs:
            msg = m
            self._lock.release()
            yield msg




# # ----------------------------------------------------------------------------------------------------------------------
# OLD BAG READER
#
# class BagReader(object):
#     """ Responsible for loading the bag."""

#     def __init__( self, bagfile ):
#         """
#         :params bagfile: str --- full path to the desired ROS bagfile
#         """
#         try:
#             self.bag   = rosbag.Bag(bagfile ,'r')
#         except Exception as e:
#             raise Exception("Could not open bag '{}': {}".format(bagfile,e))

#         self.now = rospy.Time(self.bag.get_start_time())
#         self.end = rospy.Time(self.bag.get_end_time())


#     def get_the_next_frame( self, delta ):
#         """
#         returns the frame defined by now + delta 
#         :params delta:   rospy.Duration -- frame duration
#         """
#         # RVN::TODO: Although this is a generator, it probably isn't all that efficient, but this
#         # is intended exclusively for the clips, which are short bags
#         # RVN::FIX: need to be able to stop, start this... 
#         frame = Frame(self.now, self.now+delta)
#         if self.end < frame.end:
#             raise Exception("you have walked off the end of the bag!")

#         for topic, raw_msg, t in self.bag.read_messages(raw=True, start_time=frame.start, end_time=frame.end):
#             frame.add_msg(topic, raw_msg, t)
#             # print "adding message of topic {} that occured at: {}".format(topic, t.to_sec())

#         self.now = frame.end
#         return frame


#     def seek( self, seek_point ):
#         """resets the 'now' to seek_point, abandoning the last search"""
#         # RVN::FIX: This is in conflict with what we planned -- need to reconcile the loop break with read_messages generator
#         self.now = seek_point



# ----------------------------------------------------------------------------------------------------------------------
class BagReader():
    """ Responsible for loading the bag. Will yield a generator for all messages within a given time window """
    #### RVN::FIX: This is going to have to be a seperate thread! Arrrgh, I don't understand this!!!

    def __init__( self, bagfile ):
        """
        :params bagfile: str --- full path to the desired ROS bagfile
        :params delta:   rospy.Duration -- frame duration
        """
        try:
            self.bag   = rosbag.Bag(bagfile)
        except Exception as e:
            raise Exception("Unable to load bagfile from path '{}': {}".format(bagfile,e))

        self._reset_fields(None)
        threading.Thread.__init__(self)


    def _reset_fields(self,frame):
        self._frame          = frame
        self._read_new_frame = False
        self._terminate      = False
        self._frame_end      = rospy.Time(self.bag.get_end_time())
        self._generator      = self.bag.read_messages( raw=True, start_time=seek_point, end_time=self._frame_end )


    def _wait_for_next_frame_request( self ):
        while not self._read_new_frame and not self._terminate:
            time.sleep(0.01)


    def _reached_frame_end( self, t ):
        if t < self._frame_end:
            return False
        else:
            self._read_new_frame=False
            return True


    def get_next_frame( self, duration, frame ):
        """ returns the next frame, defined by duration seconds """
        if type(duration) is float:
            duration = rospy.Duration(duration)

        self._frame_end      = self._frame_end + duration
        self._read_new_frame = True
        self._frame          = frame


    def reseek( self, seek_point, frame ):
        """ stops the current itteration and resets the read loop criteria """
        if type(seek_point) is float:
            seek_point = rospy.Time(seek_point)

        self._init_fields( frame )


    def kill( self ):
        selt._terminate = True


    def run( self ):
        """ loop over all messages in bag. Loop start defined by 'reseek()' """
        
        # RVN::TODO: Although this is a generator, it probably isn't all that efficient, but this
        # is intended exclusively for the clips, which are short bags
        # RVN::FIX: need to be able to stop, start this... 
        for topic, raw_msg, t in self._generator:

            if self._terminate:
                break

            print "waiting on frame request.."
            self._wait_for_next_frame_request()
            
            if self._reached_frame_end(t): 
                self._frame.set_complete()
                continue

            print "adding msg of topic {} to frame".format(topic)
            self._frame.add_msgs( MessageContainer(topic, raw_msg=raw_msg, t=t) )


        self._frame.set_complete()        





# ----------------------------------------------------------------------------------------------------------------------
class PublicationControl(object):
    __doc__ = """
        <node>
            <interface name='{}'>
            <method name='kill'/>
            <method name='seek'>
                <arg type='d' name='at_time_seconds' direction='in'/>
            </method>
            <method name='read'>
                <arg type='d' name='duration_seconds'   direction='in'/>
                <arg type='d' name='last_ros_clock_msg' direction='out'/>
            </method>
            <property name="current_time" type="d" access="read">
            <annotation name="org.freedesktop.DBus.Property.EmitsChangedSignal" value="true"/>                          # RVN::FIX: Implement
            </property>
            </interface>
        </node>
    """.format(service_name)

    def __init__( self, bagfile):

        self.terminate       = False  # RVN::NB: This is critical, as this flag is used to terminate the main loop
        self.exit_status     = ExitStatus.OK
        self._all_publishers = {}
        self._last_ros_clock_msg = None

        try:
            rospy.init_node( ros_node_name, anonymous=True )
        except Exception as e:
            raise Exception("Unable to start the publication node: {}".format(e))

        # This publication must use simulation time!
        rospy.set_param("use_sim_time", True)

        rospy.loginfo("Opening ROS bag at '{}' for reading...".format(bagfile))
        try:
            self.bag_reader = BagReader( bagfile )
            self.bag_reader.start()
        except Exception as e:
            self._set_terminate( ExitStatus.BAD_BAG, "Unable to create BagReader: {}".format(e))
        rospy.loginfo("...Done")


    def _create_publisher( self, msg_cont ):

        try:
            new_pub = rospy.Publisher(msg_cont.topic, msg_cont.msg_class, queue_size=100) # RVN::FIX: queue_size=???)
        except Exception as e:
            self._set_terminate( ExitStatus.PUB_FAIL, "Cannot create publisher for '{}' topic: {}".format(msg_cont.topic,e))

        self._all_publishers[msg_cont.topic] = new_pub


    def _publish_msg( self, msg_cont ):
        
        if msg_cont.topic not in self._all_publishers:
            self._create_publisher(msg_cont)

        try:
            msg = msg_cont.deserialize()
        except Exception as e:
            self._set_terminate( ExitStatus.BAD_DESERIAL, "Unable to publish msg to '{}' topic, cannot deserialize: {}".format(msg_cont.topic,e))

        self._all_publishers[msg_cont.topic].publish(msg)
        # RVN::FIX: Publish the bagged time as "/clock"


    def _set_terminate( self, code, ex_str="" ):
        """sets termination flag, error code status and raises exception (if set)"""
        self.exit_status = code
        self.terminate   = True
        self.bag_reader.kill()
        self.bag_reader.join()
        if ex_str:
            raise Exception(ex_str)



    def kill( self ):
        """ d-bus method to kill the player """
        self.bag_reader.kill()
        self.bag_reader.join()
        self._set_terminate( ExitStatus.OK, "recieved 'kill' d-bus signal" )


    def seek( self, t ):
        """ d-bus method to terminate the read loop and seek to t (or as close as possible) """
        rospy.loginfo("recieved 'seek' signal. Seeking to {}".format(t))
        ## RVN::FIX: Implement the seek -- is this a null concept with the generator??


    def read( self, duration ):
        """
        d-bus method to read and publish the frame over duration. 
        generates a frame over the period of duration, which is then published to the ROS graph. 
        returns the last \clock message as seconds float. returns a -ve number if in error
        """
        print "Recieved Read Command for frame of width: {}".format(duration)
        start_read = time.time()
        out_msg   = ""
        out_clock = -1.0

        # ROS OK check
        if rospy.is_shutdown():
            print "Error: tried to read frame when ROS is shutdown!"
            return -1.0

        print "recieved the 'read' d-bus signal. Publishing frame of {} seconds".format(duration)
        try:
            frame = self.bag_reader.get_the_next_frame( duration )
        except Exception as e:
            return out_clock, str(e)
        print "Frame extraction took: {} sec", time.time() - start_read

        for msg in frame.all_msgs:
            self._publish_msg(msg)

        if frame.last_clock:
            out_clock = frame.last_clock.deserialize()
       
        print "Frame contained {} messages, last /clock = {}".format(len(frame.all_msgs),out_clock)
        return out_clock, out_msg



# ----------------------------------------------------------------------------------------------------------------------
def main():

    verbose = False
    bagfile = ""
    parser  = argparse.ArgumentParser() # RVN::FIX: usage=print_usage())
    parser.add_argument( '-b', '--bagfile', required=True, default="", help="complete bagfile path")
    # parser.add_argument( '-v', '--verbose', help="select verbose output", action='store_true')

    args = parser.parse_args()
    if args.bagfile: bagfile = args.bagfile
    # if args.verbose: verbose = True

    # We need to change the time -- ROS cannot use wall-time for this publication
    # RVN::FIX: check and confirm this is ok

    try:
        pub = PublicationControl( bagfile )
    except Exception as e:
        rospy.logerr("{}".format(e))
        return ExitStatus.PUB_FAIL

    rospy.loginfo("starting d-bus service...")
    session_bus = SessionBus()
    loop        = GLib.MainLoop()
    session_bus.publish( service_name, pub )
    loop.run()
    rospy.loginfo("...finished d-bus service")

    return pub.exit_status



# ----------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    exit( main() )