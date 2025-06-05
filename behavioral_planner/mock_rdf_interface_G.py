class MockRDFInterface_G:
    def __init__(self, detected_types=None, obstacle_on_crosswalk=False, traffic_light_color="RED"):
        # Algılanan işaret türleri (örnek: ["STOP", "PEDESTRIAN_CROSSING"])
        self._detected_types = detected_types or []
        # Yaya geçidinde engel var mı?
        self._obstacle_on_crosswalk = obstacle_on_crosswalk
        # Trafik ışığının rengi nedir?
        self._traffic_light_color = traffic_light_color

    def get_detected_types(self):
        return self._detected_types

    def is_obstacle_on_crosswalk(self):
        return self._obstacle_on_crosswalk

    def get_traffic_light_color(self):
        return self._traffic_light_color
