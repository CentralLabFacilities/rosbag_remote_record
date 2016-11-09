"""

Copyright(c) <Florian Lier>

This file may be licensed under the terms of the
GNU Lesser General Public License Version 3 (the ``LGPL''),
or (at your option) any later version.

Software distributed under the License is distributed
on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
express or implied. See the LGPL for the specific language
governing rights and limitations.

You should have received a copy of the LGPL along with this
program. If not, go to http://www.gnu.org/licenses/lgpl.html
or write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.

The development of this software was supported by the
Excellence Cluster EXC 277 Cognitive Interaction Technology.
The Excellence Cluster EXC 277 is a grant of the Deutsche
Forschungsgemeinschaft (DFG) in the context of the German
Excellence Initiative.

Authors: Florian Lier
<flier>@techfak.uni-bielefeld.de

"""

# STD IMPORTS
import sys
import time
import signal
import psutil
import threading
import subprocess
from optparse import OptionParser

# RSB
RSB_SUPPORT = True
try:
    import rsb
except ImportError:
    RSB_SUPPORT = False

# ROS IMPORTS
ROS_SUPPORT = True
try:
    import rospy
    import roslib
    from std_msgs.msg import Bool
except ImportError:
    ROS_SUPPORT = False


class ROSRecordConnector(threading.Thread):

    def __init__(self, _filename, _inscope, _triggerscope):
        threading.Thread.__init__(self)
        self.is_running = True
        self.filename = _filename.strip()
        self.listen_topic = _triggerscope
        self.inscope = _inscope
        self.is_recording = False
        self.recordprocess = None

    def record_callback(self, ros_data):
        if (ros_data.data and self.is_recording or
                not ros_data.data and not self.is_recording):
            return  # already recording or already stopped
        else:
            self.is_recording = ros_data.data
        print ">>> [ROS] Record status: %s" % self.is_recording
        if self.is_recording is True:
            self.recordprocess = RecordBAG(self.filename, self.inscope)
            self.recordprocess.start()
        else:
            if self.recordprocess is not None:
                if self.recordprocess.stop():
                    self.recordprocess = None

    def run(self):
        print ">>> [ROS] Initializing ROSBAG REMOTE RECORD of: %s" % self.inscope.strip()
        ros_subscriber = rospy.Subscriber(self.listen_topic, Bool, self.record_callback, queue_size=1)
        print ">>> [ROS] Waiting for start on topic : %s" % self.listen_topic
        while self.is_running is True:
            time.sleep(0.05)
        if self.recordprocess is not None:
            self.recordprocess.stop()
        ros_subscriber.unregister()
        print ">>> [ROS] Stopping ROSBAG REMOTE RECORD %s" % self.inscope.strip()


class RSBRecordConnector(threading.Thread):

    def __init__(self, _filename, _inscope, _triggerscope):
        threading.Thread.__init__(self)
        self.is_running = True
        self.filename = _filename.strip()
        self.listen_scope = _triggerscope
        self.inscope = _inscope.strip()
        self.is_recording = False
        self.recordprocess = None

    def record_callback(self, event):
        if (event.data and self.is_recording or
                not event.data and not self.is_recording):
            return  # already recording or already stopped
        else:
            self.is_recording = event.data
        print ">>> [RSB] Record status: %s" % self.is_recording
        if self.is_recording is True:
            self.recordprocess = RecordBAG(self.filename, self.inscope)
            self.recordprocess.start()
        else:
            if self.recordprocess is not None:
                if self.recordprocess.stop():
                    self.recordprocess = None

    def run(self):
        print ">>> [RSB] Initializing ROSBAG REMOTE RECORD of: %s" % self.inscope.strip()
        rsb_subscriber = rsb.createListener(self.listen_scope)
        rsb_subscriber.addHandler(self.record_callback)
        while self.is_running is True:
            time.sleep(0.05)
        if self.recordprocess is not None:
            self.recordprocess.stop()
        rsb_subscriber.deactivate()
        print ">>> [RSB] Stopping ROSBAG REMOTE RECORD %s" % self.inscope.strip()


class RecordBAG(threading.Thread):
    def __init__(self, _name, _scope):
        threading.Thread.__init__(self)
        self.name = _name.strip()
        self.scope = _scope.strip()
        self.is_recording = False
        self.process = None

    def stop(self):
        print ">>> Received STOP"
        try:
            p = psutil.Process(self.process.pid)
            try:
                for sub in p.get_children(recursive=True):
                    sub.send_signal(signal.SIGINT)
                    self.process.send_signal(signal.SIGINT)
                return True
            except AttributeError:
                # newer psutils
                for sub in p.children(recursive=True):
                    sub.send_signal(signal.SIGINT)
                    self.process.send_signal(signal.SIGINT)
                return True
        except Exception as e:
            print ">>> Maybe the process is already dead? %s" % str(e)
            return False

    def run(self):
        print ">>> Recording: %s now" % self.scope
        print ">>> Filename:  %s-%s.bag %s" % (self.name, str(time.time()), self.scope)
        self.process = subprocess.Popen("rosbag record -O %s-%s.bag %s" % (self.name, str(time.time()), self.scope), shell=True)
        self.process.wait()
        print ">>> Recording: %s stopped" % self.scope


def signal_handler(sig, frame):
    """
    Callback function for the signal handler, catches signals
    and gracefully stops the application, i.e., exit subscribers
    before killing the main thread.
    :param sig the signal number
    :param frame frame objects represent execution frames.
    """
    print ">>> Exiting (signal %s)..." % str(sig)
    r.is_running = False
    print ">>> Bye!"
    sys.exit(0)


if __name__ == '__main__':

    parser = OptionParser(usage="Usage: %prog [options]")
    parser.add_option("-m", "--middleware",
                      action="store",
                      dest="middleware",
                      help="Set the middleware [ros|rsb]")
    parser.add_option("-i", "--inscope",
                      action="store",
                      dest="inscope",
                      help="Set the scopes/topics to record")
    parser.add_option("-t", "--triggerscope",
                      action="store",
                      dest="triggerscope",
                      help="Set the scope/topic for remote control (default:/meka/rosbagremote/record)")
    parser.add_option("-f", "--filename",
                      action="store",
                      dest="filename",
                      help="The name of the file that is saved")

    (options, args) = parser.parse_args()

    if not options.filename or not options.inscope or not options.middleware:
        print ">>> No inscope, filename given or middleware provided --> see help."
        sys.exit(1)
    if not options.triggerscope:
        options.triggerscope = "/meka/rosbagremote/record"
    if options.middleware.lower() == "ros":
        if ROS_SUPPORT:
            rospy.init_node('rosbag_remote_record', anonymous=True)
            r = ROSRecordConnector(options.filename, options.inscope, options.triggerscope)
            r.start()
        else:
            print ">>> ROS import failed, ros option cannot be used."
            sys.exit(-1)
    else:
        if RSB_SUPPORT:
            r = RSBRecordConnector(options.filename, options.inscope, options.triggerscope)
            r.start()
        else:
            print ">>> RSB import failed, rsb option cannot be used."
            sys.exit(-1)

    signal.signal(signal.SIGINT, signal_handler)

    while True:
        time.sleep(0.2)
