from rdflib import Graph

class RDFInterface_G:
    def __init__(self):
        self.graph = Graph()

    def load_rdf_data(self, rdf_string, format="ttl"):
        """
        RDF verisini yükler ve grafi günceller.
        
        Args:
            rdf_string (str): Turtle formatında RDF verisi
            format (str): RDF formatı (varsayılan: "ttl")
        """
        try:
            # Mevcut grafi temizle
            self.graph = Graph()
            # Yeni veriyi yükle
            self.graph.parse(data=rdf_string, format=format)
            return True
        except Exception as e:
            print(f"[RDF] Parse hatası: {e}")
            return False

    def ask_query(self, query):
        try:
            return bool(self.graph.query(query))
        except Exception as e:
            print(f"[RDF] ASK sorgu hatası: {e}")
            return False

    def select_query(self, query):
        try:
            return self.graph.query(query)
        except Exception as e:
            print(f"[RDF] SELECT sorgu hatası: {e}")
            return []

    def get_obstacle_distance(self):
        query = """
        SELECT ?d WHERE {
            ?o a :Obstacle ; :distance ?d ; :isDetected true .
        }
        ORDER BY ASC(?d)
        LIMIT 1
        """
        results = self.select_query(query)
        for row in results:
            try:
                return float(row.d.toPython())
            except:
                return None
        return None

    def get_vehicle_position(self):
        query = """
        SELECT ?x ?y WHERE {
            ?v a :Vehicle ; :posX ?x ; :posY ?y .
        } LIMIT 1
        """
        results = self.select_query(query)
        for row in results:
            try:
                x = float(row.x.toPython())
                y = float(row.y.toPython())
                return (x, y)
            except:
                return None
        return None

    def get_next_waypoint_position(self):
        query = """
        SELECT ?x ?y ?t WHERE {
            ?w a :Waypoint ; :posX ?x ; :posY ?y .
            OPTIONAL { ?w :taskType ?t }
        } LIMIT 1
        """
        results = self.select_query(query)
        for row in results:
            try:
                x = float(row.x.toPython())
                y = float(row.y.toPython())
                t = str(row.t) if row.t else "default"
                return (x, y), t
            except:
                return None
        return None

    def get_traffic_light_color(self):
        query = """
        SELECT ?color WHERE {
            ?l a :TrafficLight ; :hasColor ?color ; :isDetected true .
        } LIMIT 1
        """
        results = self.select_query(query)
        for row in results:
            return str(row.color)
        return None

    def is_obstacle_on_crosswalk(self):
        query = """
        ASK WHERE {
            ?o a :Obstacle ; :onCrosswalk true ; :isDetected true .
        }
        """
        return self.ask_query(query)

    def get_graph_state(self):
        """
        Mevcut RDF grafının durumunu döndürür.
        """
        return self.graph.serialize(format="turtle")
