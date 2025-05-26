import py_trees

# Genel Trafik İşareti Algılama
class IsTrafficSignAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.is_traffic_sign_detected() else py_trees.common.Status.FAILURE

# Trafik İşareti Tipini Getirme
class GetTrafficSignType(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface
        self.last_detected_sign = None

    def update(self):
        self.last_detected_sign = self.rdf.get_detected_traffic_sign_type()
        return py_trees.common.Status.SUCCESS if self.last_detected_sign is not None else py_trees.common.Status.FAILURE

# 23 Trafik İşareti İçin Koşul Node'ları

class IsPedestrianCrossingAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "PEDESTRIAN_CROSSING" else py_trees.common.Status.FAILURE

class IsRoundaboutAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "ROUNDABOUT" else py_trees.common.Status.FAILURE

class IsTrafficLightSignAhead(py_trees.behaviour.Behaviour):  # Trafik ışığı işareti için
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "TRAFFIC_LIGHT" else py_trees.common.Status.FAILURE

class IsNoRightTurnAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "NO_RIGHT_TURN" else py_trees.common.Status.FAILURE

class IsNoLeftTurnAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "NO_LEFT_TURN" else py_trees.common.Status.FAILURE

class IsNoEntryAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "NO_ENTRY" else py_trees.common.Status.FAILURE

class IsKeepRightAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "KEEP_RIGHT" else py_trees.common.Status.FAILURE

class IsKeepLeftAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "KEEP_LEFT" else py_trees.common.Status.FAILURE

class IsMandatoryTurnRightAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "MANDATORY_TURN_RIGHT" else py_trees.common.Status.FAILURE

class IsMandatoryTurnLeftAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "MANDATORY_TURN_LEFT" else py_trees.common.Status.FAILURE

class IsMandatoryGoStraightAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "MANDATORY_GO_STRAIGHT" else py_trees.common.Status.FAILURE

class IsMandatoryGoStraightOrRightAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "MANDATORY_STRAIGHT_OR_RIGHT" else py_trees.common.Status.FAILURE

class IsMandatoryGoStraightOrLeftAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "MANDATORY_STRAIGHT_OR_LEFT" else py_trees.common.Status.FAILURE

class IsMandatoryTurnRightAfterAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "MANDATORY_TURN_RIGHT_AFTER" else py_trees.common.Status.FAILURE

class IsMandatoryTurnLeftAfterAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "MANDATORY_TURN_LEFT_AFTER" else py_trees.common.Status.FAILURE

class IsLaneMergeRightAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "LANE_MERGE_RIGHT" else py_trees.common.Status.FAILURE

class IsLaneMergeLeftAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "LANE_MERGE_LEFT" else py_trees.common.Status.FAILURE

class IsTwoWayTrafficAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "TWO_WAY_TRAFFIC" else py_trees.common.Status.FAILURE

class IsNoParkingAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "NO_PARKING" else py_trees.common.Status.FAILURE

class IsParkingAreaAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "PARKING_AREA" else py_trees.common.Status.FAILURE

class IsTunnelAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "TUNNEL" else py_trees.common.Status.FAILURE

class IsBusStopAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "BUS_STOP" else py_trees.common.Status.FAILURE

class IsStopSignAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name, rdf_interface):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        return py_trees.common.Status.SUCCESS if self.rdf.get_detected_traffic_sign_type() == "STOP" else py_trees.common.Status.FAILURE
