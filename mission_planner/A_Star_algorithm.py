import osmnx as ox
import math
import heapq # priority queue için heapq modülünü kullanıyoruz (priority_dict ifadesinde hata aldım)
#import json


# Heuristik fonksiyon: Euclidean mesafesini hesaplar
def distance_heuristic(state_key, goal_key, node_data):
    n1 = node_data[state_key]  # Başlangıç vertex'inin verilerini al
    n2 = node_data[goal_key]  # Hedef vertex'inin verilerini al

    long1 = n1['x'] * math.pi / 180.0
    lat1 = n1['y'] * math.pi / 180.0
    long2 = n2['x'] * math.pi / 180.0
    lat2 = n2['y'] * math.pi / 180.0

    r = 6371000  # Dünya'nın yarıçapı, metre cinsinden (6371 km * 1000)

    x1 = r * math.cos(lat1) * math.cos(long1)
    y1 = r * math.cos(lat1) * math.sin(long1)
    z1 = r * math.sin(lat1)

    x2 = r * math.cos(lat2) * math.cos(long2)
    y2 = r * math.cos(lat2) * math.sin(long2)
    z2 = r * math.sin(lat2)

    d = ((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2) ** 0.5
    return d


# A* algoritması implementasyonu
def a_star_search(origin_key, goal_key, graph):
    open_queue = []  # Açık kuyruk (heap kullanarak öncelikli kuyruk oluşturacağız)
    heapq.heappush(open_queue, (0, origin_key))  # Başlangıç noktasını ekle (f_score, node)

    closed_dict = {}  # Kapalı liste (işlenmiş düğümler)
    predecessors = {}  # Önceki düğüm bilgisi
    costs = {origin_key: 0.0}  # Başlangıçtan şu ana kadar olan gerçek mesafe

    node_data = graph.nodes(True)

    goal_found = False
    while open_queue:
        f_cost, u = heapq.heappop(open_queue)  # En düşük f_cost'u pop et

        if u == goal_key:  # Eğer hedefe ulaşılmışsa
            goal_found = True
            break

        # Ulaşılabilir komşular üzerinde işlem yap
        for edge in graph.out_edges([u], data=True):
            if edge[1] in closed_dict:  # Eğer komşu vertex zaten kapalı listede ise
                continue

            length = edge[2]['length']  # Kenarın uzunluğu

            g_cost = costs[u] + length  # Geçilen yolun maliyeti (g(n))
            h_cost = distance_heuristic(edge[1], goal_key, node_data)  # Hedefe olan tahmini mesafe (h(n))
            f_cost = g_cost + h_cost  # f(n) = g(n) + h(n)

            if edge[1] in costs:
                # Eğer yeni yol daha kısa ise, güncelle
                if g_cost < costs[edge[1]]:
                    costs[edge[1]] = g_cost
                    heapq.heappush(open_queue, (f_cost, edge[1]))
                    predecessors[edge[1]] = u
            else:
                # Komşuyu açık kuyrukta değilse, ekle
                costs[edge[1]] = g_cost
                heapq.heappush(open_queue, (f_cost, edge[1]))
                predecessors[edge[1]] = u

        # Şu anki vertex'i kapalı listeye ekle
        closed_dict[u] = 0

    if not goal_found:
        raise ValueError("Goal not found in search.")

    # Predecessor (öncül) göstergelerini kullanarak yolu oluştur
    return get_path(origin_key, goal_key, predecessors)


# Bu fonksiyon, predecessor (öncül) göstergelerini takip ederek
# başlangıç noktasından hedefe kadar olan yolu bir vertex anahtarları listesi olarak döndürür
def get_path(origin_key, goal_key, predecessors):
    key = goal_key
    path = [goal_key]

    # Hedefe ulaşıldığında, origin_key (başlangıç noktası) ile karşılaşana kadar
    # predecessors sözlüğünden her bir vertex'in öncülünü takip ederiz.
    while key != origin_key:
        key = predecessors[key]  # Şu anki vertex'in öncülünü al
        path.insert(0, key)  # Yeni vertex'i yolun başına ekle

    return path

# JSON'la grid yapısı kullanımı
#json_data = '''{
 # "grid": [
 #  [0, 1, 0, 0, 0],
 # [0, 1, 0, 1, 0],
 # [0, 0, 0, 1, 0],
   # [0, 1, 1, 1, 0],
   # [0, 0, 0, 0, 0]
  # ],
  #"start": [0, 0],
  #"goal": [4, 4]
#}'''

# JSON'la ayrıştırma
#data = json.loads(json_data)

# Gridleri ve noktaları al
#grid = data["grid"]
#start = tuple(data["start"])
#goal = tuple(data["goal"])

#Some explanations:
#The grid represents the environment. The 0s are walkable cells(walkable cells refer to locations in the grid or environment that the algorithm can traverse.
 These cells are typically open or clear of obstacles,
# meaning the algorithm can move through them to find a path from the start point to the goal.), and the 1s are blocked cells.
#The start and goal are the points where the A* algorithm will begin and aim to reach.


# Örnek harita ve nokta verileri
map_graph = ox.graph_from_point(center_point=(39.78394, 32.80378), dist=2500, network_type='drive')
origin = ox.nearest_nodes(map_graph, Y=39.780240, X=32.816827)  # Başlangıç noktası
destination = ox.nearest_nodes(map_graph, Y=39.785540, X=32.790327)  # Hedef noktası

# A* algoritması ile en kısa yolu bul
path = a_star_search(origin, destination, map_graph)

# Yolu harita üzerinde çiz
fig, ax = ox.plot_graph_route(map_graph, path)



