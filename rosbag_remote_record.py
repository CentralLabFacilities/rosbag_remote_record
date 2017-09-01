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
import os
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
    from std_msgs.msg import Bool, String
except ImportError:
    ROS_SUPPORT = False


class ROSRecordConnector(threading.Thread):

    def __init__(self, _filename, _inscope, _triggerscope, _msglimit, _directory=None):
        threading.Thread.__init__(self)
        self.is_running = True
        self.filename = _filename.strip()
        if _directory is None:
            self.directory = ""
        else:
            self.directory = _directory.strip()
            if self.directory != "":
                self.directory = self.directory + "/"
        self.listen_topic = _triggerscope
        self.inscope = _inscope
        self.msglimit = _msglimit
        self.recordprocess = None

    def record_string_callback(self, ros_data):
        if ros_data.data.lower().endswith(":start"):
            # extract filename
            idx = ros_data.data.lower().rfind(":start")
            newfilename = ros_data.data[0:idx]
            if len(newfilename) > 0 and newfilename.find(' ') == -1:
                self.record_handling(True, self.directory + newfilename)
            else:
                print ">>> [ROS] Record filename malformed: '%s'. Should not be empty or contain spaces, using default name" % newfilename
                self.record_handling(True, self.directory + self.filename)
        else:
            if ros_data.data.lower().endswith(":stop"):
                self.record_handling(False)
            else:
                # invalid message
                print ">>> [ROS] Record request invalid: %s. Should contain filename:start or :stop" % ros_data.data
                return

    def record_bool_callback(self, ros_data):
        self.record_handling(ros_data.data, self.filename)

    def record_handling(self, record, filename=None):
        if (record and self.recordprocess is not None and self.recordprocess.is_recording):
          return False # already recording 

        if (not record and self.recordprocess is None):
            return False  # already stopped
        if self.recordprocess is not None:
          print ">>> [ROS] Record status: %s" % str(self.recordprocess.is_recording)
        else:
          print ">>> [ROS] Record status: %s" % False
          
        if record and filename is not None:
            self.recordprocess = RecordBAG(filename, self.inscope, self.msglimit)
            self.recordprocess.start()
            return True
        else:
            if self.recordprocess is not None:
                if self.recordprocess.stop():
                    self.recordprocess = None
                    return True
                else:
                    return False
            return True

    def run(self):
        print ">>> [ROS] Initializing ROSBAG REMOTE RECORD of: %s" % self.inscope.strip()
        ros_subscriber = rospy.Subscriber(self.listen_topic, Bool, self.record_bool_callback, queue_size=1)
        ros_subscriber2 = rospy.Subscriber(self.listen_topic + "/named", String, self.record_string_callback, queue_size=1)
        print ">>> [ROS] Waiting for Bool (true for start) on topic : %s" % self.listen_topic
        print ">>> [ROS] Waiting for filename:start on topic : %s" % self.listen_topic + "/named"
        while self.is_running is True:
            time.sleep(0.05)
        if self.recordprocess is not None:
            self.recordprocess.stop()
        ros_subscriber.unregister()
        ros_subscriber2.unregister()
        print ">>> [ROS] Stopping ROSBAG REMOTE RECORD %s" % self.inscope.strip()


class RSBRecordConnector(threading.Thread):

    def __init__(self, _filename, _inscope, _triggerscope, _msglimit):
        threading.Thread.__init__(self)
        self.is_running = True
        self.filename = _filename.strip()
        self.listen_scope = _triggerscope
        self.inscope = _inscope.strip()
        self.msglimit = _msglimit
        self.recordprocess = None

    def record_callback(self, event):
        if (event.data and self.recordprocess is not None and self.recordprocess.is_recording):
          return False # already recording 

        if (not event.data and self.recordprocess is None):
            return False  # already stopped
        if self.recordprocess is not None:
          print ">>> [RSB] Record status: %s" % str(self.recordprocess.is_recording)
        else:
          print ">>> [RSB] Record status: %s" % False
          
        if event.data:
            self.recordprocess = RecordBAG(self.filename, self.inscope, self.msglimit)
            self.recordprocess.start()
            return True
        else:
            if self.recordprocess is not None:
                if self.recordprocess.stop():
                    self.recordprocess = None
                    return True
                else:
                    return False
            return True

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
    def __init__(self, _name, _scope, _msg_limit=None):
        threading.Thread.__init__(self)
        self.name = _name.strip()
        self.scope = _scope.strip()
        self.is_recording = False
        self.process = None
        self.msg_limit = _msg_limit

    def stop(self):
        print ">>> Received STOP"
        if not self.is_recording:
          print ">>> Already stopped"
          return True
        self.is_recording = False
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
        rosbag_filename = self.name + "-" + str(time.time()) + ".bag"
        print ">>> Filename:  %s" % rosbag_filename
        if self.msg_limit is not None:
          print ">>> stopping after %s messages" % str(self.msg_limit)
          self.process = subprocess.Popen("rosbag record -l %s -O %s %s" % (str(self.msg_limit), rosbag_filename, self.scope), shell=True)
        else:
          self.process = subprocess.Popen("rosbag record -O %s %s" % (rosbag_filename, self.scope), shell=True)
        self.is_recording = True
        self.process.wait()
        print ">>> Recording: %s stopped" % self.scope
        self.is_recording = False


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
                      help="Set the root scope/topic for remote control (default:/meka/rosbagremote/record)\
                       another topic <root>/named provides a way to query a new bag file name while starting")
    parser.add_option("-f", "--filename",
                      action="store",
                      dest="filename",
                      help="The name of the file that is saved")
    parser.add_option("-d", "--directory",
                      action="store",
                      dest="directory",
                      help="(only for named trigger) the base directory in which the files should be saved")
    parser.add_option("-l", "--limit",
                      action="store",
                      dest="msglimit",
                      help="The number of message to record before automatically stopping")

    (options, args) = parser.parse_args()

    if not options.filename or not options.inscope or not options.middleware:
        print ">>> No inscope, filename given or middleware provided --> see help."
        sys.exit(1)
    if not options.triggerscope:
        options.triggerscope = "/meka/rosbagremote/record"
    if options.middleware.lower() == "ros":
        if ROS_SUPPORT:
            rospy.init_node('rosbag_remote_record', anonymous=True)
            r = ROSRecordConnector(options.filename, options.inscope, options.triggerscope, options.msglimit, options.directory)
            r.start()
        else:
            print ">>> ROS import failed, ros option cannot be used."
            sys.exit(-1)
    else:
        if RSB_SUPPORT:
            r = RSBRecordConnector(options.filename, options.inscope, options.triggerscope, options.msglimit)
            r.start()
        else:
            print ">>> RSB import failed, rsb option cannot be used."
            sys.exit(-1)

    signal.signal(signal.SIGINT, signal_handler)

    while True:
        time.sleep(0.2)
