#!/usr/bin/env python
import rospy
import math
import tf
import random
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time

# =============================
# PID Controller
# =============================
class PID:
    def __init__(self, kp=0.2, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error)/dt if dt>0 else 0
        self.prev_error = error
        return self.kp*error + self.ki*self.integral + self.kd*derivative

# =============================
# Robot simulation
# =============================
class Robot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.last_time = rospy.Time.now()
        self.br = tf.TransformBroadcaster()
        self.path_pub = rospy.Publisher("/robot_path", Path, queue_size=10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "world"

    def step(self, v, w):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        if dt<=0: dt=0.01
        self.last_time = now

        self.x += v*math.cos(self.yaw)*dt
        self.y += v*math.sin(self.yaw)*dt
        self.yaw += w*dt

        # publish TF
        self.br.sendTransform(
            (self.x, self.y, 0),
            tf.transformations.quaternion_from_euler(0,0,self.yaw),
            now,
            "body",
            "world"
        )

        # publish path
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "world"
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0
        qx,qy,qz,qw = tf.transformations.quaternion_from_euler(0,0,self.yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)
            
                       
    def run_episode(self, pid_v, pid_w, gx, gy, max_time=10.0):
        start_time = time.time()
        max_overshoot = 0
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            dx = gx - self.x
            dy = gy - self.y
            rho = math.sqrt(dx**2 + dy**2)
            angle2goal = math.atan2(dy, dx)
            dyaw = angle2goal - self.yaw
            dyaw = (dyaw+math.pi) % (2*math.pi) - math.pi
            
            
            # PID compute
            v = pid_v.compute(rho, 0.02) * rho
            w = pid_w.compute(dyaw, 0.02) * dyaw

            # clamp speeds
            v = max(min(v,0.4),-0.4)
            w = max(min(w,0.6),-0.6)

            self.step(v,w)

            if rho>0.1:
                max_overshoot = max(max_overshoot, rho)

            if rho<0.1 or (time.time()-start_time)>max_time:
            	
                break
                
                
            rate.sleep()

        settling_time = time.time() - start_time
        final_error = rho
        overshoot = max_overshoot
        return settling_time, final_error, overshoot

# =============================
# GA Tuner with continuous state
# =============================
class GA_Tuner:
    def __init__(self, robot, pop_size=8, generations=10):
        self.pop_size = pop_size
        self.generations = generations
        self.robot = robot

    def random_individual(self):
        return {
            'Kp_v': random.uniform(0.0,1.0),
            'Ki_v': random.uniform(0.0,0.1),
            'Kd_v': random.uniform(0.0,0.2),
            'Kp_w': random.uniform(0.0,1.0),
            'Ki_w': random.uniform(0.0,0.1),
            'Kd_w': random.uniform(0.0,0.2),
        }

    def crossover(self, a,b):
        return {k: a[k] if random.random()<0.5 else b[k] for k in a}

    def mutate(self, ind):
        for k in ind:
            if random.random()<0.3:
                ind[k] *= (1 + random.uniform(-0.2,0.2))
                ind[k] = max(0,ind[k])
        return ind

    def fitness(self, ind, gx, gy):
        pid_v = PID(ind['Kp_v'], ind['Ki_v'], ind['Kd_v'])
        pid_w = PID(ind['Kp_w'], ind['Ki_w'], ind['Kd_w'])
        settling, final_error, overshoot = self.robot.run_episode(pid_v, pid_w, gx, gy)
        fitness_val = settling + 20*final_error + 5*max(0, overshoot-0.1)
        return fitness_val

    def run(self, gx, gy):
        pop = [self.random_individual() for _ in range(self.pop_size)]
        for gen in range(self.generations):
            scored = [(self.fitness(ind, gx, gy), ind) for ind in pop]
            scored.sort(key=lambda x:x[0])
            pop = [ind for (_,ind) in scored[:2]]  # elites
            while len(pop)<self.pop_size:
                a,b = random.sample(pop,2)
                child = self.mutate(self.crossover(a,b))
                pop.append(child)
        best = scored[0][1]
        return best

# =============================
# Main
# =============================
if __name__=='__main__':
    rospy.init_node('robot_pid_ga')

    gx = rospy.get_param('~goal_x', 1.0)
    gy = rospy.get_param('~goal_y', 1.0)

    robot = Robot()
    ga = GA_Tuner(robot)

    best_pid = ga.run(gx, gy)

    print("\n===== GA Tuned PID =====")
    for k,v in best_pid.items():
        print(f"{k} = {v:.3f}")
    print("========================\n")

    # Use the best PID to move to goal
    pid_v = PID(best_pid['Kp_v'], best_pid['Ki_v'], best_pid['Kd_v'])
    pid_w = PID(best_pid['Kp_w'], best_pid['Ki_w'], best_pid['Kd_w'])
    robot.run_episode(pid_v, pid_w, gx, gy)
