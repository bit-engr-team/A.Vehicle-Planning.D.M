from rdflib import Graph, Literal, Namespace
from rdflib.namespace import RDF, XSD
import json
from typing import Dict, List, Union, Optional

class PerceptionConverter:
    def __init__(self):
        # RDF namespace tanımlamaları
        self.ns = Namespace("http://example.org/")
        self.graph = Graph()
        self.graph.bind("", self.ns)

    def _create_base_rdf(self) -> str:
        """Temel RDF prefix'lerini içeren başlangıç string'ini oluşturur"""
        return """
        @prefix : <http://example.org/> .
        @prefix xsd: <http://www.w3.org/2001/XMLSchema#> .
        """

    def _convert_timestamp(self, timestamp: float) -> str:
        """Timestamp'i RDF formatına dönüştürür"""
        return f":timestamp {timestamp}^^xsd:float ."

    def _convert_vehicle_position(self, position: List[float]) -> str:
        """Araç pozisyonunu RDF formatına dönüştürür"""
        if len(position) >= 2:
            return f"""
            :Vehicle :posX {position[0]}^^xsd:float ;
                    :posY {position[1]}^^xsd:float .
            """
        return ""

    def _convert_obstacles(self, objects: List[Dict]) -> str:
        """Engelleri RDF formatına dönüştürür"""
        obstacle_triples = []
        for i, obj in enumerate(objects):
            if obj.get("type") in ["dynamic", "static"]:
                distance = obj.get("distance", 0.0)
                obstacle_triples.append(f"""
                :Obstacle{i} a :Obstacle ;
                            :distance {distance}^^xsd:float ;
                            :isDetected true ;
                            :onCrosswalk false .
                """)
        return "\n".join(obstacle_triples)

    def _convert_traffic_lights(self, objects: List[Dict]) -> str:
        """Trafik ışıklarını RDF formatına dönüştürür"""
        traffic_light_triples = []
        for i, obj in enumerate(objects):
            if obj.get("class") == "traffic_light":
                color = "red"  # Varsayılan değer
                traffic_light_triples.append(f"""
                :TrafficLight{i} a :TrafficLight ;
                                :hasColor "{color}" ;
                                :isDetected true .
                """)
        return "\n".join(traffic_light_triples)

    def _convert_lane_info(self, lane_data: Dict) -> str:
        """Şerit bilgilerini RDF formatına dönüştürür"""
        lane_triples = []
        if "lane_centerlines" in lane_data:
            for i, centerline in enumerate(lane_data["lane_centerlines"]):
                if "points" in centerline and len(centerline["points"]) > 0:
                    # İlk noktayı waypoint olarak kullan
                    point = centerline["points"][0]
                    lane_triples.append(f"""
                    :Waypoint{i} a :Waypoint ;
                                :posX {point[0]}^^xsd:float ;
                                :posY {point[1]}^^xsd:float ;
                                :taskType "lane_following" .
                    """)
        return "\n".join(lane_triples)

    def convert_perception_data(self, perception_data: Dict) -> str:
        """
        Perception modülünden gelen JSON verisini RDF formatına dönüştürür
        
        Args:
            perception_data: Perception modülünden gelen JSON verisi
                {
                    "timestamp": float,
                    "lane_lines": [...],
                    "lane_centerlines": [...],
                    "drivable_area": {...},
                    "objects": [...]
                }
        
        Returns:
            str: Turtle formatında RDF verisi
        """
        # RDF verisini oluşturmaya başla
        rdf_parts = [self._create_base_rdf()]

        # Timestamp ekle
        if "timestamp" in perception_data:
            rdf_parts.append(self._convert_timestamp(perception_data["timestamp"]))

        # Araç pozisyonunu ekle (örnek olarak)
        # Not: Gerçek uygulamada bu veri başka bir kaynaktan gelmelidir
        rdf_parts.append(self._convert_vehicle_position([0.0, 0.0]))

        # Engelleri ekle
        if "objects" in perception_data:
            rdf_parts.append(self._convert_obstacles(perception_data["objects"]))
            rdf_parts.append(self._convert_traffic_lights(perception_data["objects"]))

        # Şerit bilgilerini ekle
        if "lane_centerlines" in perception_data:
            rdf_parts.append(self._convert_lane_info(perception_data))

        # Tüm RDF verisini birleştir
        return "\n".join(rdf_parts)

    def validate_rdf(self, rdf_string: str) -> bool:
        """
        Oluşturulan RDF verisinin geçerli olup olmadığını kontrol eder
        
        Args:
            rdf_string: Turtle formatında RDF verisi
        
        Returns:
            bool: RDF verisi geçerli ise True, değilse False
        """
        try:
            self.graph.parse(data=rdf_string, format="turtle")
            return True
        except Exception as e:
            print(f"RDF validation error: {e}")
            return False

# Kullanım örneği
if __name__ == "__main__":
    # Örnek perception verisi
    sample_perception_data = {
        "timestamp": 1234567890.123,
        "lane_lines": [
            {"points": [[0, 0], [1, 1]]},
            {"points": [[0, 1], [1, 2]]}
        ],
        "lane_centerlines": [
            {"points": [[0.5, 0.5], [1.5, 1.5]]}
        ],
        "drivable_area": {
            "boundary_points": [[0, 0], [1, 0], [1, 1], [0, 1]],
            "area": 1,
            "mask_shape": [100, 100]
        },
        "objects": [
            {
                "class": "car",
                "distance": 10.5,
                "type": "dynamic"
            },
            {
                "class": "traffic_light",
                "distance": 5.0,
                "type": "static"
            }
        ]
    }

    # Converter'ı oluştur
    converter = PerceptionConverter()
    
    # Veriyi dönüştür
    rdf_data = converter.convert_perception_data(sample_perception_data)
    
    # RDF verisini doğrula
    if converter.validate_rdf(rdf_data):
        # Behavioral Planner'a gönder
        behavioral_planner.update_rdf(rdf_data)
        print("RDF conversion successful!")
        print("\nGenerated RDF:")
        print(rdf_data)
    else:
        print("RDF conversion failed!") 