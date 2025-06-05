import py_trees

from behavioral_planner_G.nodes_G.waypoint_nodes_G import (
    IsWaypointAvailable_G,
    RequestWaypointFromMissionPlanner_G,
    CreateTemporaryWaypoint_G
)
from behavioral_planner_G.nodes_G.dynamic_obstacle_nodes_G import (
    IsObstacleAhead_G,
    IsObstacleClose_G,
    StopVehicle_G,
    SlowDown_G
)
from behavioral_planner_G.nodes_G.traffic_sign_nodes_G import (
    IsTrafficSignDetected_G,
    GetTrafficSignType_G,
    UpdateBehaviorFromSign_G
)
from behavioral_planner_G.nodes_G.action_executor_nodes_G import (
    IsAtNextWaypoint_G,
    MoveToWaypoint_G,
    IsBusStop_G,
    PickupPassenger_G,
    IsParkingArea_G,
    StopForMission_G,
    FinishMission_G,
    UpdateToNextWaypoint_G
)

def create_behavior_tree(rdf_interface, decision_callback=None):
    root = py_trees.composites.Sequence("Davranış Planlayıcı")

    # === 1. Waypoint Durumu (Fallback) ===
    waypoint_fallback = py_trees.composites.Fallback("Waypoint Durumu")
    waypoint_fallback.add_children([
        IsWaypointAvailable_G(rdf_interface),
        RequestWaypointFromMissionPlanner_G(decision_callback=decision_callback),
        CreateTemporaryWaypoint_G(decision_callback=decision_callback)
    ])

    # === 2. Çevresel Kontroller (Parallel) ===
    env_parallel = py_trees.composites.Parallel("Çevresel Kontroller", policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    # 2.1 Dinamik Engel Kontrolü (Sequence içinde Fallback)
    obstacle_sequence = py_trees.composites.Sequence("Dinamik Engel Kontrolü")
    obstacle_sequence.add_children([
        IsObstacleAhead_G(rdf_interface),
        py_trees.composites.Fallback("Engel Varsa")
    ])
    obstacle_fallback = obstacle_sequence.children[1]
    obstacle_fallback.add_children([
        py_trees.composites.Sequence("Yakın Engel Kontrolü", children=[
            IsObstacleClose_G(rdf_interface),
            StopVehicle_G(decision_callback=decision_callback)
        ]),
        SlowDown_G(decision_callback=decision_callback)
    ])

    # 2.2 Trafik İşaretleri (Sequence)
    traffic_sequence = py_trees.composites.Sequence("Trafik İşaretleri")
    traffic_sequence.add_children([
        IsTrafficSignDetected_G(rdf_interface),
        GetTrafficSignType_G(rdf_interface),
        UpdateBehaviorFromSign_G(rdf_interface=rdf_interface, decision_callback=decision_callback)
    ])

    env_parallel.add_children([obstacle_sequence, traffic_sequence])

    # === 3. Aksiyon Alma (Sequence) ===
    action_sequence = py_trees.composites.Sequence("Aksiyon Alma")

    # 3.1 Hedefe hareket (Fallback)
    move_fallback = py_trees.composites.Fallback("Waypoint Hareket Kontrolü")
    move_fallback.add_children([
        IsAtNextWaypoint_G(rdf_interface=rdf_interface),
        MoveToWaypoint_G(decision_callback=decision_callback)
    ])

    # 3.2 Görev Yürüt (Parallel)
    task_parallel = py_trees.composites.Parallel("Görevler", policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    # 3.2.1 Yolcu Alma (Sequence)
    pickup_sequence = py_trees.composites.Sequence("Yolcu Alma")
    pickup_sequence.add_children([
        IsBusStop_G(rdf_interface, (0, 0), (0, 0)),  # Geçici pozisyonlar
        StopForMission_G(decision_callback=decision_callback),
        PickupPassenger_G(decision_callback=decision_callback)
    ])

    # 3.2.2 Park Etme (Sequence)
    park_sequence = py_trees.composites.Sequence("Park Etme")
    park_sequence.add_children([
        IsParkingArea_G(rdf_interface, (0, 0), (0, 0)),
        StopForMission_G(decision_callback=decision_callback),
        FinishMission_G(decision_callback=decision_callback)
    ])

    task_parallel.add_children([pickup_sequence, park_sequence])

    action_sequence.add_children([
        py_trees.composites.Parallel("Paralel Hareket ve Görev", policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
                                     children=[move_fallback, task_parallel]),
        UpdateToNextWaypoint_G(decision_callback=decision_callback)
    ])

    # === Tüm ana düğümleri kök düğüme ekle ===
    root.add_children([waypoint_fallback, env_parallel, action_sequence])

    return root
