from visualization_msgs.msg import Marker, MarkerArray
import rospy

class CustomMarker(Marker):
    """
    Represents a ROS visualization marker with customizable properties.
    """

    def __init__(self, x, y, marker_id, size, color, shape=Marker.CUBE, duration=0, frame_id="map"):
        """
        Initializes a CustomMarker object.

        Args:
            x (float): X position of the marker (in meters).
            y (float): Y position of the marker (in meters).
            marker_id (int): Unique ID for the marker.
            size (float): Size of the marker (in meters).
            color (tuple): RGBA color of the marker (values between 0.0 and 1.0). Default: (0.0, 1.0, 0.0, 1.0) (green).
            shape (int, optional): Shape type (e.g., Marker.CUBE, Marker.SPHERE). Defaults to Marker.CUBE.
            duration (float, optional): Lifetime of the marker in seconds. Defaults to 0 (persistent).
            frame_id (str, optional): Frame ID of the marker. Defaults to "map".
        """
        super().__init__()
        self.header.frame_id = frame_id
        self.header.stamp = rospy.Time.now()
        self.ns = "exploration_goals"
        self.id = marker_id
        self.shape = shape
        self.action = Marker.ADD
        self.pose.position.x, self.pose.position.y = x, y
        self.pose.orientation.w = 1.0  # No rotation
        self.scale.x = self.scale.y = self.scale.z = size
        # Correct way to assign colors to the Marker's color attribute
        self.color.r, self.color.g, self.color.b, self.color.a = color  
        self.lifetime = rospy.Duration(duration)



class GoalMarkerArray:
    """
    Manages and publishes an array of markers to visualize exploration targets and goals.
    """

    def __init__(self, topic_name='/frontiers', queue_size=100):
        """
        Initializes the GoalMarkerArray.

        Args:
            topic_name (str, optional): Name of the topic to publish the marker array. Defaults to '/frontiers'.
            queue_size (int, optional): Size of the publisher queue. Defaults to 100.
        """
        self.marker_pub = rospy.Publisher(topic_name, MarkerArray, queue_size=queue_size)

    def display(self, possible_locations, goal, resolution, duration=0):
        """
        Publishes markers for the possible goal locations and the current goal.

        Args:
            possible_locations (list): List of (x, y) coordinates of possible goal locations in meters.
            goal (tuple): (x, y) coordinates of the current goal in meters.
            resolution (float): Resolution of the map in meters per cell.
            duration (float, optional): Lifetime of the markers in seconds. Defaults to 0 (persistent).
        """
        markers_msg = MarkerArray()

        # Target Markers (cubes)
        for i, location in enumerate(possible_locations, start=1):
            marker = CustomMarker(*location, i, resolution, (0, 1, 0, 0.5), duration=duration)  # Green with transparency
            markers_msg.markers.append(marker)

        # Goal Marker (sphere)
        marker = CustomMarker(*goal, len(possible_locations) + 1, 2 * resolution, (1, 1, 0, 1), Marker.SPHERE, duration=duration) # Yellow
        markers_msg.markers.append(marker)

        self.marker_pub.publish(markers_msg)
