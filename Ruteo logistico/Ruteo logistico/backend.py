from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List
import ortools.constraint_solver.routing_enums_pb2 as routing_enums_pb2
from ortools.constraint_solver import pywrapcp

app = FastAPI()

# Configura CORS para tu frontend en Netlify
app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://singular-raindrop-329837.netlify.app"],  # Cambia por tu URL real
    allow_methods=["*"],
    allow_headers=["*"],
)

class RoutingRequest(BaseModel):
    locations: List[dict]
    distance_matrix: List[List[float]]  # Ahora recibimos la matriz desde el frontend

@app.post("/optimize-route")
async def optimize_route(request: RoutingRequest):
    try:
        manager = pywrapcp.RoutingIndexManager(
            len(request.distance_matrix), 1, 0)  # 1 vehículo, depósito en 0
        routing = pywrapcp.RoutingModel(manager)
        
        def distance_callback(from_index, to_index):
            return request.distance_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]
        
        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        search_parameters.time_limit.seconds = 3
        
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
            return {"status": "error", "message": "No se encontró solución"}
            
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))