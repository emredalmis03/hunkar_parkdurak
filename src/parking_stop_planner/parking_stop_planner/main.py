import math
from models import Pose2D
from planners.dubins_planner import plan

def main():
    print("Starting Dubins Planner Test...")
    
    start = Pose2D(x=0.0, y=0.0, yaw=0.0)
    
    #dumduz gitme
    #goal = Pose2D(x=10.0, y=0.0, yaw=0.0)
    #capraz 90 derece
    goal = Pose2D(x=10.0, y=10.0, yaw=math.pi/2)
    
    waypoints = plan(start, goal)
    
    print(f"Ggenerated {len(waypoints)} waypoints")
    
    for i, wp in enumerate(waypoints):
        print(f"Waypoint {i}: x={wp.x}, y={wp.y}, yaw={wp.yaw/math.pi*180}")
    

if __name__ == "__main__":
    #print("yoo")
    main()