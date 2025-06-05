from mock_rdf_interface_G import MockRDFInterface_G
from nodes.traffic_sign_nodes_G import UpdateBehaviorFromSign_G

# Sahte RDF verisi tanımlanıyor
mock_rdf = MockRDFInterface_G(
    detected_types=["STOP", "PEDESTRIAN_CROSSING", "TRAFFIC_LIGHT"],
    obstacle_on_crosswalk=True,
    traffic_light_color="GREEN"
)

# Node oluşturuluyor
update_node = UpdateBehaviorFromSign_G(rdf_interface=mock_rdf)

# update() fonksiyonu bir kez çağrılıyor (bt tick gibi)
status = update_node.update()

# Çıktı yazdırılıyor
print(f"Node durumu: {status}")
