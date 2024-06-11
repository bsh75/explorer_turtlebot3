class StateController:
    def __init__(self):
        self.current_state = "exploring"  # Initial state
        self.recovery_attempts = 0
        self.robot_pose = None
        self.prev_robot_pose = None

    def update_pose(self, pose_msg):
        # store both the current and previouse poses
        self.prev_robot_pose = self.robot_pose 
        self.robot_pose = pose_msg

    def get_robot_position_and_yaw(self):


    def update_feedback(self, feedback_msg):
        # ... (logic to update state based on feedback)

    def update_result(self, status, result_msg):
        # ... (logic to update state based on result)

    def update_status(self, status_msg):
        # ... (logic to update state based on recovery status)

    def trigger_recovery(self):
        # ... (logic to initiate recovery behavior)

    def get_state(self):
        return self.current_state