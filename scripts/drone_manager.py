#!/usr/bin/env python

# #{ imports

import rospy
import csv

# include messages
from mrs_msgs.srv import TrajectoryReferenceSrv
from mrs_msgs.msg import ControlManagerDiagnostics
from mrs_msgs.msg import Reference

from mrs_msgs.srv import PathSrv
from mrs_msgs.srv import PathSrvRequest as PathSrvRequest

from mrs_msgs.srv import TrajectoryReferenceSrv
from mrs_msgs.srv import TrajectoryReferenceSrvRequest as TrajectoryReferenceSrvRequest

from std_srvs.srv import Trigger as TriggerSrv
from std_srvs.srv import TriggerRequest as TriggerSrvRequest
from std_srvs.srv import TriggerResponse as TriggerSrvResponse

# #} end of imports

class DroneManager:

    # #{ __init__()

    def __init__(self):

        rospy.init_node('drone_manager', anonymous=True)

        ## | --------------- initialize global variables -------------- |

        self.control_manager_diag_ = ControlManagerDiagnostics()
        self.is_initialized_ = False
        self.finished_ = False
        self.started_ = False

        ## | --------------------- load parameters -------------------- |

        self.land_duration_ = rospy.get_param('~land_duration')

        self.goto_x_       = rospy.get_param('~goto_destination/x')
        self.goto_y_       = rospy.get_param('~goto_destination/y')
        self.goto_z_       = rospy.get_param('~goto_destination/z')
        self.goto_heading_ = rospy.get_param('~goto_destination/heading')

        self.trajectory_sampling_rate_ = rospy.get_param('~trajectory_sampling_rate')

        self.reference_frame_ = rospy.get_param('~reference_frame')

        self.trajectory_file_ = rospy.get_param('~trajectory_file')

        # TODO: !!! check if the params have been loaded successfully

        ## | ---------------- load trajectory from file --------------- |

        # the trajectory object for publishing
        self.trajectory_ = TrajectoryReferenceSrvRequest()

        self.trajectory_.trajectory.header.frame_id = self.reference_frame_
        self.trajectory_.trajectory.fly_now = False # cause we will call goto
        self.trajectory_.trajectory.dt = 1.0/self.trajectory_sampling_rate_
        self.trajectory_.trajectory.use_heading = True

        with open(self.trajectory_file_, newline='') as csvfile:

            reader = csv.reader(csvfile, delimiter=' ')

            for row in reader:

                point = Reference()

                point.position.x = float(row[0])
                point.position.y = float(row[1])
                point.position.z = float(row[2])
                point.heading    = float(row[3])

                self.trajectory_.trajectory.points.append(point)

        ## | --------------------- service servers -------------------- |

        self.ss_start_ = rospy.Service('~start_in', TriggerSrv, self.callbackStart)

        ## | --------------------- service clients -------------------- |

        self.sc_land_           = rospy.ServiceProxy('~land_out', TriggerSrv)
        self.sc_takeoff_        = rospy.ServiceProxy('~takeoff_out', TriggerSrv)
        self.sc_trajectory_     = rospy.ServiceProxy('~trajectory_out', TrajectoryReferenceSrv)
        self.sc_path_           = rospy.ServiceProxy('~path_out', PathSrv)
        self.sc_goto_start_     = rospy.ServiceProxy('~goto_start_out', TriggerSrv)
        self.sc_start_tracking_ = rospy.ServiceProxy('~start_tracking_out', TriggerSrv)

        ## | ----------------------- subscribers ---------------------- |

        rospy.Subscriber("~control_manager_diagnostics_in", ControlManagerDiagnostics, self.callbackControlManagerDiag)

        ## | ------------------------- timers ------------------------- |

        rospy.Timer(rospy.Duration(1.0), self.timerMain)

        ## | --------------------- finish the init -------------------- |

        self.is_initialized_ = True
        rospy.loginfo('node initialized')

        ## | -------------------------- spin -------------------------- |

        rospy.spin()

    # #} end of __init__()

## | ------------------------ callbacks ----------------------- |

    # #{ callbackControlManagerDiag()

    def callbackControlManagerDiag(self, data):

        if not self.is_initialized_:
            return

        rospy.loginfo_once("getting control manager diag")

        self.control_manager_diag_ = data

    # #} end of callbackControlManagerDiag()

    # #{ callbackStart()

    def callbackStart(self, req):

        if not self.is_initialized_:
            return

        self.started_ = True

        rospy.loginfo('activated')

        response = TriggerSrvResponse()

        response.success = True
        response.message = "started"

        return response

# #} end of callbackStart()

## | ------------------------- timers ------------------------- |

    # #{ timerMain()

    def timerMain(self, event):

        if not self.is_initialized_:
            return

        if self.finished_:
            return

        if not self.started_:
            rospy.loginfo('waiting for activation, call the "drone_manager/start" service to start')
            return

        rospy.loginfo_once("timerMain() spinning")

        # --------------------------------------------------------------
        # |        the state machine should be placed in here  \/      |
        # --------------------------------------------------------------

        while not self.freeToCommand():
            rospy.sleep(0.1)
            rospy.loginfo('waiting for the UAV to be ready')

        self.setTrajectory(self.trajectory_)

        self.gotoStart()

        while not self.freeToCommand():
            rospy.sleep(0.1)
            rospy.loginfo('waiting for the UAV to stop')

        self.startTrajectoryTracking()

        while not self.freeToCommand():
            rospy.sleep(0.1)
            rospy.loginfo('waiting for the UAV to stop')

        self.gotoXYZH(self.goto_x_, self.goto_y_, self.goto_z_, self.goto_heading_)

        while not self.freeToCommand():
            rospy.sleep(0.1)
            rospy.loginfo('waiting for the UAV to stop')

        self.land()

        rospy.loginfo('sleeping for additional {} s'.format(self.land_duration_))
        rospy.sleep(self.land_duration_)

        while not self.landed():
            rospy.sleep(0.1)
            rospy.loginfo('waiting for landing')

        self.takeoff()

        while not self.freeToCommand():
            rospy.sleep(0.1)
            rospy.loginfo('waiting for takeoff')

        self.finished_ = True

        rospy.loginfo('FINISHED')

        # --------------------------------------------------------------
        # |        the state machine should be placed in here  /\      |
        # --------------------------------------------------------------

# #} end of timerMain()

## | ------------------------- methods ------------------------ |

## service wrappers

    # #{ land()

    def land(self):

        land = TriggerSrvRequest()

        rospy.loginfo('calling for landing')

        resp = self.sc_land_.call(land)

        # TODO: check the return code and decide what to do if it fails

        rospy.sleep(1.0)

    # #} end of land()

    # #{ gotoStart()

    def gotoStart(self):

        goto_start = TriggerSrvRequest()

        rospy.loginfo('calling for goto trajectory start')

        resp = self.sc_goto_start_.call(goto_start)

        # TODO: check the return code and decide what to do if it fails

        rospy.sleep(1.0)

    # #} end of gotoStart()

    # #{ startTrajectoryTracking()

    def startTrajectoryTracking(self):

        start_tracking = TriggerSrvRequest()

        rospy.loginfo('calling for start trajectory tracking')

        resp = self.sc_start_tracking_.call(start_tracking)

        # TODO: check the return code and decide what to do if it fails

        rospy.sleep(1.0)

    # #} end of startTrajectoryTracking()

    # #{ takeoff()

    def takeoff(self):

        takeoff = TriggerSrvRequest()

        rospy.loginfo('calling for takeoff')

        resp = self.sc_takeoff_.call(takeoff)

        # TODO: check the return code and decide what to do if it fails

        rospy.sleep(1.0)

    # #} end of takeoff()

    # #{ setTrajectory()

    def setTrajectory(self, trajectory):

        rospy.loginfo('setting trajectory')

        resp = self.sc_trajectory_.call(trajectory)

        # TODO: check the return code and decide what to do if it fails

        rospy.sleep(1.0)

    # #} end of setTrajectory()

    # #{ gotoXYZH()

    def gotoXYZH(self, x, y, z, heading):

        rospy.loginfo('calling goto')

        path = PathSrvRequest()
        path.path.header.stamp = rospy.Time.now()
        path.path.header.frame_id = self.reference_frame_

        path.path.fly_now = True
        path.path.use_heading = True

        point = Reference()

        point.position.x       = x
        point.position.y       = y
        point.position.z       = z
        point.heading = heading

        path.path.points.append(point)

        resp = self.setPath(path)

        # TODO: check the return code and decide what to do if it fails

        rospy.sleep(1.0)

    # #} end of gotoXYZH()

    # #{ setPath()

    def setPath(self, path):

        rospy.loginfo('setting path')

        resp = self.sc_path_.call(path)

        # TODO: check the return code and decide what to do if it fails

        rospy.sleep(1.0)

    # #} end of setPath()

## getters

    # #{ freeToCommand()

    def freeToCommand(self):

        return self.control_manager_diag_.flying_normally and not self.control_manager_diag_.tracker_status.have_goal

    # #} end of freeToCommand()

    # #{ landed()

    def landed(self):

        return self.control_manager_diag_.active_tracker == "NullTracker"

    # #} end of landed()

if __name__ == '__main__':
    try:
        drone_manager = DroneManager()
    except rospy.ROSInterruptException:
        pass
