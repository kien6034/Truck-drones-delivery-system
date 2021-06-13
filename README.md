# Truck-drones-delivery-system
APCS heuristic, time-dependent model

Paper:
The time-depenent multiple Flying sidekicks travelling salesman problem: Parcel Delivery with Traffic congestion 

Problem:

Using the system of 1 truck and multiple drones (UAV) for the last-mile delivery operation 
 - Truck and UAV can work concurrently (While truck go delivery, UAV do the same and vice versa. Truck and UAV shouldnt wait (do nothing) if not necessary)
 - UAV can be retrieved at customer nodes or at the depots
 - When being retrival, UAV then can be charged by truck
 - There are some customer nodes that can be served by truck, UAV are not allowed to deliver there. 
 
 - Important: Truck's route is affected by traffic congestion. (Handel this, we can avoid UAV's crash and optimize the makespan - whole delivery's operation time)
 
Objectives 
-> Miniminze make span


Data: 
Get data from Open Street Map (real data)


Algorithm
- Ant-pair colony system
- Label setting with traffic congestion 

Run:
python main.py 
