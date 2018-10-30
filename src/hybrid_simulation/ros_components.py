import rospy
from hybrid_simulation.msg import VehicleStatusArray
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel


class RosComponents:

    def __init__(self):

        rospy.init_node('sumo_interface', anonymous=True)
        self.use_gazebo = rospy.get_param('~use_gazebo', True)
        self.control_ego_vehicle = rospy.get_param('~control_ego_vehicle', True)
        self.control_from_gazebo = rospy.get_param('~control_from_gazebo', True)
        self.lane_change = rospy.get_param('~lane_change', False)
        self.ego_vehicle_id = rospy.get_param('~ego_vehicle_name', "prius")

        if self.use_gazebo is True:
            rospy.loginfo("Waiting for gazebo services...")
            rospy.wait_for_service("/gazebo/delete_model")
            rospy.wait_for_service("/gazebo/spawn_sdf_model")
            self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
            self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.vehicle_status_pub = rospy.Publisher('vehicles_status', VehicleStatusArray, queue_size=1)


