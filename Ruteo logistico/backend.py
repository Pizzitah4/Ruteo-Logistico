from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List
import ortools.constraint_solver.routing_enums_pb2 as routing_enums_pb2
from ortools.constraint_solver import pywrapcp

app = FastAPI()

# Configura CORS para desarrollo
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

class RoutingRequest(BaseModel):
    locations: List[dict]  # [{lat: x, lng: y, id:?}, ...]
    vehicle_capacity: int = None  # Para problemas de capacidad
    time_windows: List[tuple] = None  # Para restricciones de tiempo

def create_distance_matrix(locations):
    # Ejemplo simplificado (sin API externa)
    matrix = []
    for i in range(len(locations)):
        row = []
        for j in range(len(locations)):
            if i == j:
                row.append(0)
            else:
                # Distancia aproximada (en metros)
                lat1, lng1 = locations[i]["lat"], locations[i]["lng"]
                lat2, lng2 = locations[j]["lat"], locations[j]["lng"]
                distance = ((lat2 - lat1)**2 + (lng2 - lng1)**2)**0.5 * 111319  # 1 grado ≈ 111 km
                row.append(int(distance))
        matrix.append(row)
    return matrix

@app.post("/optimize-route")
async def optimize_route(request: RoutingRequest):
    try:
        distance_matrix = create_distance_matrix(request.locations)
        
        manager = pywrapcp.RoutingIndexManager(
            len(distance_matrix), 1, 0)  # 1 vehículo, depósito en 0
        routing = pywrapcp.RoutingModel(manager)
        
        def distance_callback(from_index, to_index):
            return distance_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]
        
        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        search_parameters.time_limit.seconds = 5
        
        solution = routing.SolveWithParameters(search_parameters)
        
        if solution:
            index = routing.Start(0)
            route = []
            while not routing.IsEnd(index):
                node = manager.IndexToNode(index)
                route.append(request.locations[node])
                index = solution.Value(routing.NextVar(index))
            
            return {"status": "success", "route": route}
        else:
            return {"status": "error", "message": "No solution found"}
            
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
