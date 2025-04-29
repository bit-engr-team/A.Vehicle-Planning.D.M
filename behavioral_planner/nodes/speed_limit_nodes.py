import py_trees

class IsSpeedLimitRespected(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.is_speed_within_limit() else py_trees.common.Status.FAILURE

class AdjustSpeedToLimit(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_publisher_interface):
        super().__init__(name)
        self.publisher = ros_publisher_interface

    def update(self):
        self.publisher.publish_adjust_speed_command()
        return py_trees.common.Status.SUCCESS
