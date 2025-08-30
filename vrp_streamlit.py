import streamlit as st
import time, random
from datetime import datetime, timedelta
from geopy.distance import geodesic
import folium
from streamlit_folium import st_folium
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# ---------------- CONFIG ---------------- #
DEPOT_LOCATION = (12.9716, 77.5946)
NUM_VEHICLES = 3
REFRESH_INTERVAL = 5  # seconds between updates
VEHICLE_SPEED = 30  # km/h (for ETA simulation)

# ---------------- SESSION STATE ---------------- #
if "orders" not in st.session_state:
    st.session_state.orders = []
if "logs" not in st.session_state:
    st.session_state.logs = []
if "vehicles" not in st.session_state:
    st.session_state.vehicles = [
        {"id": i, "status": "Available", "route": [], "eta": None}
        for i in range(NUM_VEHICLES)
    ]

# ---------------- HELPERS ---------------- #
def create_random_order():
    return {
        "id": len(st.session_state.orders) + 1,
        "lat": DEPOT_LOCATION[0] + random.uniform(-0.05, 0.05),
        "lon": DEPOT_LOCATION[1] + random.uniform(-0.05, 0.05),
        "time": datetime.now()
    }

def build_distance_matrix(orders, depot):
    locations = [depot] + [(o["lat"], o["lon"]) for o in orders]
    n = len(locations)
    matrix = [[0]*n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i != j:
                matrix[i][j] = geodesic(locations[i], locations[j]).km
    return matrix

def solve_vrp(distance_matrix, num_vehicles, depot=0):
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicles, depot)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        return int(distance_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)] * 1000)

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_parameters)
    routes = []
    if solution:
        for vehicle_id in range(num_vehicles):
            index = routing.Start(vehicle_id)
            route = []
            while not routing.IsEnd(index):
                route.append(manager.IndexToNode(index))
                index = solution.Value(routing.NextVar(index))
            routes.append(route)
    return routes

def estimate_eta(route, distance_matrix):
    total_km = 0
    for i in range(len(route)-1):
        total_km += distance_matrix[route[i]][route[i+1]]
    hours = total_km / VEHICLE_SPEED
    return datetime.now() + timedelta(hours=hours)

# ---------------- SIMULATION STEP ---------------- #
# Random new orders
if random.random() < 0.4:  # 40% chance new order
    new_order = create_random_order()
    st.session_state.orders.append(new_order)
    st.session_state.logs.append({"time": datetime.now(), "action": f"New order {new_order['id']} created"})

# Update vehicle statuses
for v in st.session_state.vehicles:
    if v["status"] == "Out for Delivery" and v["eta"] and datetime.now() >= v["eta"]:
        v["status"] = "Returning"
        v["eta"] = datetime.now() + timedelta(minutes=2)  # 2 min return time
        st.session_state.logs.append({"time": datetime.now(), "action": f"Vehicle {v['id']} finished delivery, returning"})

    elif v["status"] == "Returning" and v["eta"] and datetime.now() >= v["eta"]:
        v["status"] = "Available"
        v["route"] = []
        v["eta"] = None
        st.session_state.logs.append({"time": datetime.now(), "action": f"Vehicle {v['id']} is now available"})

# Assign new routes if available vehicles + pending orders
available_vehicles = [v for v in st.session_state.vehicles if v["status"] == "Available"]
if st.session_state.orders and available_vehicles:
    dist_matrix = build_distance_matrix(st.session_state.orders, DEPOT_LOCATION)
    routes = solve_vrp(dist_matrix, NUM_VEHICLES)

    for v, route in zip(st.session_state.vehicles, routes):
        if v["status"] == "Available" and len(route) > 1:
            v["route"] = route
            v["status"] = "Out for Delivery"
            v["eta"] = estimate_eta(route, dist_matrix)
            st.session_state.logs.append({"time": datetime.now(), "action": f"Vehicle {v['id']} dispatched with route {route}"})

# ---------------- DISPLAY ---------------- #
st.title("🚚 Dynamic VRP Simulation with Vehicle States")

# Map
m = folium.Map(location=DEPOT_LOCATION, zoom_start=13)
folium.Marker(DEPOT_LOCATION, tooltip="Depot", icon=folium.Icon(color="red")).add_to(m)

for o in st.session_state.orders:
    folium.Marker((o["lat"], o["lon"]), tooltip=f"Order {o['id']}").add_to(m)

colors = ["blue", "green", "purple"]
for v in st.session_state.vehicles:
    if v["route"]:
        coords = [DEPOT_LOCATION] + [(st.session_state.orders[i-1]["lat"], st.session_state.orders[i-1]["lon"]) for i in v["route"] if i != 0]
        folium.PolyLine(coords, color=colors[v["id"] % len(colors)], weight=3).add_to(m)

st_folium(m, width=700, height=500)

# Vehicle table
st.subheader("🚛 Vehicle Status")
st.table([{"Vehicle": v["id"], "Status": v["status"], "ETA": v["eta"]} for v in st.session_state.vehicles])

# Orders
st.subheader("📦 Orders")
st.write(st.session_state.orders)

# Logs
st.subheader("📝 Logs")
st.write(st.session_state.logs)

# Auto-refresh
time.sleep(REFRESH_INTERVAL)
st.experimental_rerun()
