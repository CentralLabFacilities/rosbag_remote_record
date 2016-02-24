"""

This file is part of FINITE STATE MACHINE BASED TESTING.

Copyright(c) <Florian Lier>
http://opensource.cit-ec.de/fsmt

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

Authors: Florian Lier, Norman Koester
<flier>@techfak.uni-bielefeld.de

"""

from setuptools import setup

version = "0.01"
filename = "0.01"

setup(name="rosbag_remote_record",

      version=filename,

      description="Trigger ROSBAG recordings remotely",

      long_description="Trigger ROSBAG recordings remotely",

      author="Florian Lier",

      author_email="flier[at]techfak.uni-bielefeld.de",

      url="https://github.com/CentralLabFacilities/rosbag_remote_record",

      download_url="https://github.com/CentralLabFacilities/rosbag_remote_record",

      scripts=["rosbag_remote_record.py"],

      keywords=['ROSBAG', 'ROS', 'Remote Recording'],

      license="LGPLv3",

      classifiers=[
          'Development Status :: Beta',
          'Environment :: Console',
          'Environment :: Robotics',
          'Intended Audience :: Developers',
          'License :: OSI Approved :: GNU Library or ' +
          'Lesser General Public License (LGPL)',
          'Operating System :: OS Independent',
          'Programming Language :: Python',
          'Topic :: Text Processing :: Markup :: XML'
      ],

      install_requires=['psutil'],

)
