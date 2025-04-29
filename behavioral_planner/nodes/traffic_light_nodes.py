import py_trees

class IsTrafficLightAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.is_traffic_light_detected() else py_trees.common.Status.FAILURE

class IsTrafficLightRed(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_traffic_light_color() == "RED" else py_trees.common.Status.FAILURE

class IsTrafficLightYellow(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_traffic_light_color() == "YELLOW" else py_trees.common.Status.FAILure

class IsTrafficLightGreen(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_traffic_light_color() == "GREEN" else py_trees.common.Status.FAILURE

class SlowDownAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_publisher_interface):
        super().__init__(name)
        self.publisher = ros_publisher_interface

    def update(self):
        self.publisher.publish_slow_down_command()
        return py_trees.common.Status.SUCCESS

class ProceedNormally(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_publisher_interface):
        super().__init__(name)
        self.publisher = ros_publisher_interface

    def update(self):
        self.publisher.publish_normal_drive_command()
        return py_trees.common.Status.SUCCESS
