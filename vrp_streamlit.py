import streamlit as st
import random, time
from datetime import datetime, timedelta
from geopy.distance import geodesic
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

st.title("🚚 VRP Simulation App")

# ---------------- PARAMETERS ---------------- #
num_vehicles = st.sidebar.number_input("Number of Vehicles", min_value=1, max_value=10, value=3)
vehicle_speed = st.sidebar.number_input("Vehicle Speed (km/h)", min_value=10, max_value=5000, value=3000)
max_orders_per_vehicle = st.sidebar.number_input("Max Orders per Vehicle", min_value=1, max_value=10, value=3)
initial_orders = st.sidebar.number_input("Initial Orders", min_value=1, max_value=20, value=5)
sim_speed = st.sidebar.number_input("Simulation Speed (sec)", min_value=1, max_value=10, value=1)

DEPOT_LOCATION = (12.9716, 77.5946)

# ---------------- SESSION STATE ---------------- #
if "all_orders" not in st.session_state:
    st.session_state.all_orders = []
if "available_virtual_vehicles" not in st.session_state:
    st.session_state.available_virtual_vehicles = []
if "out_for_delivery" not in st.session_state:
    st.session_state.out_for_delivery = []
if "delivery_log" not in st.session_state:
    st.session_state.delivery_log = []
if "vehicle_pool" not in st.session_state:
    st.session_state.vehicle_pool = [
        {"id": f"V{i+1}", "max_vol": 100, "max_weight": 200, "available_at": datetime.now(), "trip_count": 0}
        for i in range(num_vehicles)
    ]

# ---------------- HELPERS ---------------- #
def travel_time_km(loc1, loc2):
    distance_km = geodesic(loc1, loc2).km
    return (distance_km / vehicle_speed) * 60  # minutes

def generate_random_order(i):
    lat = round(random.uniform(12.95, 12.99), 5)
    lon = round(random.uniform(77.55, 77.65), 5)
    priority = random.choice([1, 2])
    wait_time = 30 if priority == 1 else 60
    return {
        "id": f"O{i}",
        "location": (lat, lon),
        "volume": random.randint(10, 30),
        "weight": random.randint(5, 20),
        "priority": priority,
        "wait_time": wait_time,
        "arrival_time": datetime.now()
    }

def can_reach_in_time(order, depot, vehicle_available_at):
    travel_minutes = travel_time_km(depot, order["location"])
    latest_delivery = order["arrival_time"] + timedelta(minutes=order["wait_time"])
    expected_delivery = vehicle_available_at + timedelta(minutes=travel_minutes)
    return expected_delivery <= latest_delivery

# ---------------- VRP SOLVER ---------------- #
def assign_orders(orders, vehicles):
    if not orders or not vehicles:
        return []
    
    all_locations = [DEPOT_LOCATION] + [o["location"] for o in orders]
    distance_matrix = [
        [int(geodesic(loc1, loc2).km*1000) for loc2 in all_locations]
        for loc1 in all_locations
    ]
    volumes = [o["volume"] for o in orders]
    weights = [o["weight"] for o in orders]

    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), len(vehicles), 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        return distance_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]
    
    routing.SetArcCostEvaluatorOfAllVehicles(routing.RegisterTransitCallback(distance_callback))

    def demand_callback(from_index):
        node = manager.IndexToNode(from_index)
        return 0 if node==0 else 1
    
    routing.AddDimensionWithVehicleCapacity(
        routing.RegisterUnaryTransitCallback(demand_callback),
        0,
        [max_orders_per_vehicle]*len(vehicles),
        True,
        "OrderCount"
    )

    solution = routing.SolveWithParameters(pywrapcp.DefaultRoutingSearchParameters())
    assignments = []
    if solution:
        for v_id in range(len(vehicles)):
            index = routing.Start(v_id)
            route = []
            while not routing.IsEnd(index):
                node = manager.IndexToNode(index)
                if node!=0:
                    route.append(orders[node-1])
                index = solution.Value(routing.NextVar(index))
            if route:
                assignments.append((vehicles[v_id], route))
    return assignments

# ---------------- SIMULATION ---------------- #
if st.button("Start Simulation"):
    # Generate initial orders
    for i in range(initial_orders):
        st.session_state.all_orders.append(generate_random_order(i))

    order_id = initial_orders
    for step in range(10):  # simulate 10 steps
        st.write(f"### Step {step+1}")
        # Random new order
        if random.random() < 0.5:
            new_order = generate_random_order(order_id)
            st.session_state.all_orders.append(new_order)
            st.write(f"New Order: {new_order['id']} | Priority: {new_order['priority']}")
            order_id += 1
        
        # Assign available vehicles
        available_vehicles = [v for v in st.session_state.vehicle_pool if v["available_at"] <= datetime.now() and v["trip_count"]<3]
        valid_orders = [o for o in st.session_state.all_orders if can_reach_in_time(o, DEPOT_LOCATION, datetime.now())]
        assignments = assign_orders(valid_orders, available_vehicles)

        for v, orders_assigned in assignments:
            total_route_minutes = sum(travel_time_km(DEPOT_LOCATION, o["location"]) for o in orders_assigned)
            v["available_at"] = datetime.now() + timedelta(minutes=total_route_minutes)
            v["trip_count"] += 1
            st.write(f"🚚 Vehicle {v['id']} dispatched: {[o['id'] for o in orders_assigned]}")
        
        time.sleep(sim_speed)
