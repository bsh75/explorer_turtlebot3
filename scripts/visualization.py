from visualization_msgs.msg import Marker, MarkerArray
import rospy

class CustomisedMarker(Marker):
    def __init__(self, x, y, marker_id, size, shape, color, duration, frame_id="map"):
        """
        Initializes a Marker object with customizable size, shape, color, and ID.

        Args:
            x (float): X position in meters.
            y (float): Y position in meters.
            marker_id (int): Unique ID for the marker.
            size (float): Size of the marker in meters.
            shape (int, optional): Shape type (e.g., Marker.CUBE, Marker.SPHERE). Defaults to Marker.CUBE.
            color (tuple, optional): RGBA color tuple (r, g, b, a), each value between 0.0 and 1.0. 
                                     Defaults to green (0.0, 1.0, 0.0, 1.0).
            frame_id (str, optional): Frame ID of the marker. Defaults to "map".
        """
        super().__init__()  # Call the constructor of the parent Marker class
        self.header.frame_id = frame_id
        self.header.stamp = rospy.Time.now()
        self.ns = "exploration_goals"
        self.id = marker_id
        self.type = shape
        self.action = Marker.ADD
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = 0
        self.pose.orientation.w = 1.0
        self.scale.x = size
        self.scale.y = size
        self.scale.z = size
        self.color.r, self.color.g, self.color.b, self.color.a = color
        self.lifetime = rospy.Duration(duration) 

class MarkerArrayDisplay(MarkerArray):
    def __init__(self, possible_locations, goal, resolution, duration):
        super().__init__()  # Call the constructor of the parent Marker class
        self.possible_locations = possible_locations
        self.goal = goal
        self.resolution = resolution
        self.marker_pub = rospy.Publisher('/frontiers', MarkerArray, queue_size=100)
        self.duration = duration

    def refresh_targets_and_goal(self):
        # Create array of potential targets
        markers_msg = []  # Clearing
        location_size = self.resolution
        goal_size = location_size*2
        location_color = (0.0, 1.0, 0.0, 0.5)
        goal_color = (1.0, 1.0, 0.0, 1.0)

        for i, location in enumerate(self.possible_locations, start=1):
            marker = CustomisedMarker(*location, i, Marker.CUBE, location_size, location_color, self.duration)
            markers_msg.append(marker)
        # Add the goal marker
        goal_marker = CustomisedMarker(*self.goal, i+1, Marker.SPHERE, goal_size, goal_color, self.duration)
        markers_msg.append(goal_marker)
        self.marker_pub.publish(self.markers_msg)