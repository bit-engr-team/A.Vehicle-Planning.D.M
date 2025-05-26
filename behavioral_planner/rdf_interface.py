from rdflib import Graph
from std_msgs.msg import String

class RDFInterface:
    def __init__(self, node):
        self.graph = Graph() # RDF verisini tutacak nesne, veritabani gibi
        self.node = node
        self.latest_rdf_data = None

        self.subscriber = self.node.create_subscription(
            String,                 #mesaj tipi
            "/perception/rdf",      #topic ismi
            self.callback,          #veri geldiginde ne yapilacak
            10                      #queue size
        )

    def callback(self, msg):
        self.latest_rdf_data = msg.data
        self.graph = Graph() #graph yeniden baslatilir, rdf verilerinin surekli yenilenmesi icin
        try:
            self.graph.parse(data=self.latest_rdf_data, format="ttl")   #Bu satır RDF verisini belleğe yükler.
        except Exception as e:
            self.node.get_logger().warn(f"RDF parse hatası: {e}")

    def get_obstacle_distance(self):
        """
        Algılanan ilk engelin mesafesini (metre) döndürür.
        """
        query = '''
        SELECT ?d WHERE {
            ?o a :Obstacle ;
            :distance ?d ;
            :isDetected true .
        } LIMIT 1
        '''
        results = self.graph.query(query)
        for row in results:
            try:
                return float(row.d.toPython())  # Literal → float dönüşümü
            except:
                return None
        return None

    def is_traffic_light_detected(self):
        query = '''
        ASK WHERE {
            ?l a :TrafficLight ; :isDetected true .
        }
        '''
        return bool(self.graph.query(query))

    def get_traffic_light_color(self):
        query = '''
        SELECT ?color WHERE {
            ?l a :TrafficLight ; :hasColor ?color ; :isDetected true .
        } LIMIT 1
        '''
        results = self.graph.query(query)
        for row in results:
            return str(row.color)
        return None

    def is_traffic_sign_detected(self):
        query = '''
        ASK WHERE {
            ?s a :TrafficSign ; :isDetected true .
        }
        '''
        return bool(self.graph.query(query))

    def get_detected_traffic_sign_type(self):
        query = '''
        SELECT ?type WHERE {
            ?s a :TrafficSign ; :hasType ?type ; :isDetected true .
        } LIMIT 1
        '''
        results = self.graph.query(query)
        for row in results:
            return str(row.type)
        return None

    def is_within_lane(self):
        query = '''
        ASK WHERE {
            ?v a :Vehicle ; :isWithinLane true .
        }
        '''
        return bool(self.graph.query(query))

    def is_speed_within_limit(self):
        query = '''
        ASK WHERE {
            ?v a :Vehicle ; :isSpeedWithinLimit true .
        }
        '''
        return bool(self.graph.query(query))

    def is_wrong_way_detected(self):
        query = '''
        ASK WHERE {
            ?v a :Vehicle ; :isWrongWay true .
        }
        '''
        return bool(self.graph.query(query))

    def is_parking_area_detected(self):
        query = '''
        ASK WHERE {
            ?p a :ParkingArea ; :isDetected true .
        }
        '''
        return bool(self.graph.query(query))

    def is_at_waypoint(self):
        query = '''
        ASK WHERE {
            ?v a :Vehicle ; :isAtWaypoint true .
        }
        '''
        return bool(self.graph.query(query))