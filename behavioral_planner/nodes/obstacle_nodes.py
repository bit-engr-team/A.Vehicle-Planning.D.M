import py_trees

class IsObstacleAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface, threshold=1.0):
        super().__init__(name)
        self.rdf = rdf_interface
        self.threshold = threshold  # Mesafe eşiği (metre)

    def update(self):
        distance = self.rdf.get_obstacle_distance()
        if distance is not None and distance <= self.threshold:
            return py_trees.common.Status.SUCCESS  # Engel çok yakın → engel var
        else:
            return py_trees.common.Status.FAILURE

class StopVehicle(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_publisher_interface):
        super().__init__(name)
        self.publisher = ros_publisher_interface

    def update(self):
        self.publisher.publish_stop_command()
        return py_trees.common.Status.SUCCESS
