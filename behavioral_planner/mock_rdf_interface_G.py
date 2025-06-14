class MockRDFInterface_G:
    def __init__(self, detected_types=None, obstacle_on_crosswalk=False, traffic_light_color="RED"):
        # Algılanan işaret türleri (örnek: ["STOP", "PEDESTRIAN_CROSSING"])
        self._detected_types = detected_types or []
        # Yaya geçidinde engel var mı?
        self._obstacle_on_crosswalk = obstacle_on_crosswalk
        # Trafik ışığının rengi nedir?
        self._traffic_light_color = traffic_light_color
        # Araç pozisyonu (varsayılan değerler)
        self._vehicle_position = (0, 0)
        # Sonraki waypoint pozisyonu (varsayılan değerler)
        self._next_waypoint_position = (5, 5)
        # Aktif waypoint durumu
        self._has_active_waypoint = False

    def get_detected_types(self):
        return self._detected_types

    def is_obstacle_on_crosswalk(self):
        return self._obstacle_on_crosswalk

    def get_traffic_light_color(self):
        return self._traffic_light_color

    def ask_query(self, query):
        # Basit bir mock implementasyonu
        # Gerçek SPARQL sorgusu yerine, algılanan türlere göre yanıt döndürür
        if "Obstacle" in query:
            return self._obstacle_on_crosswalk
        elif "TrafficSign" in query or "TrafficLight" in query or "Crosswalk" in query:
            return len(self._detected_types) > 0
        elif "Vehicle" in query and "hasActiveWaypoint" in query:
            return self._has_active_waypoint
        return False

    def select_query(self, query):
        # Basit bir mock implementasyonu
        # Gerçek SPARQL sorgusu yerine, algılanan türlere göre yanıt döndürür
        class MockResult:
            def __init__(self, type_value):
                self.type = type_value

        results = []
        if "?type" in query:
            for type_name in self._detected_types:
                results.append(MockResult(f":{type_name}"))
        return results

    def get_vehicle_position(self):
        return self._vehicle_position

    def get_next_waypoint_position(self):
        return self._next_waypoint_position

    def load_rdf_data(self, rdf_turtle_string):
        # Mock implementasyon - gerçek RDF verisi yüklemez
        pass

    def set_vehicle_position(self, position):
        self._vehicle_position = position

    def set_next_waypoint_position(self, position):
        self._next_waypoint_position = position

    def set_active_waypoint(self, has_active):
        self._has_active_waypoint = has_active
