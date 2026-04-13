from __future__ import annotations
import numpy as np
import argparse
import cv2
from .planner import AStarPlanner
from .city_map import CityMap, MapParams
from .car import RaceCar
from .vision import CarVision
from .simulation import Simulation


class CityCarPracticeApp:
    def __init__(self, args):
        self.args = args
        self.simulation = Simulation(gui=args.gui)
        self.city = CityMap(MapParams(cell=args.cell))
        self.car = RaceCar()
        self.vision = CarVision()
        self.planner = AStarPlanner()

    def setup(self):
        self.simulation.connect()
        self.city.build_world()

        sx, sy = self.args.start
        gx, gy = self.args.goal

        start = self.city.nearest_free(sx, sy) or self.city.world_to_grid(sx, sy)
        goal = self.city.nearest_free(gx, gy) or self.city.world_to_grid(gx, gy)

        x0, y0 = self.city.grid_to_world(*start)
        xg, yg = self.city.grid_to_world(*goal)

        self.car.spawn(x0, y0, yaw=0.0)
        self.car.stop()
        self.simulation.add_goal_marker(xg, yg)

        if self.args.gui:
            self.simulation.set_topdown_camera(self.city.params.plane_size)

        return start, goal, (x0, y0), (xg, yg)

    def try_astar(self, start, goal, car_world_xy):
        self.path = self.planner.a_star(self.city.grid, start, goal, allow_diag=True)
        if self.path is not None:
            self.simulation.add_info_text(
                "A* implementado: integra aqui el movimiento del coche",
                car_world_xy[0],
                car_world_xy[1],
                color=(0, 1, 0),
                life_time=3.0,
            )

    def move_car(self):
        for gx, gy in self.path:
            wx, wy = self.city.grid_to_world(gx, gy)

            while True:
                # Propiedades Coche
                THROTTLE = 30
                MAX_STEER = 0.6
                DISTANCE_TOLERANCE = 0.5
                pos, orn, yaw = self.car.get_pose()
                car_x, car_y, _ = pos

                # Distancia entre el coche y el siguiente punto
                delta_x = wx - car_x
                delta_y = wy - car_y
                distance = np.sqrt(delta_x**2 + delta_y**2)

                # Obtener siguiente posicion
                if distance < DISTANCE_TOLERANCE :
                    break

                # Angulo respecto al siguiente punto
                angle_to_point = np.arctan2(delta_y, delta_x)
                delta_angle_car_yaw = angle_to_point - yaw 
                normal_delta_angle = (delta_angle_car_yaw + np.pi) % (2 * np.pi) - np.pi
                        
                steering = float(np.clip(normal_delta_angle, -MAX_STEER, MAX_STEER))
                
                # Mover coche
                self.car.set_control(steering, THROTTLE)
                self.simulation.step() 

        self.car.stop()
        
    def run(self):
        start, goal, car_world_xy, _ = self.setup()
        self.try_astar(start, goal, car_world_xy)
        
        # Dibujar el camino del A*
        last_wx, last_wy = self.city.grid_to_world(start[0], start[1])
        for gx, gy in self.path:
            wx, wy = self.city.grid_to_world(gx, gy)
            self.city.draw_path(last_wx, last_wy, wx, wy)
            last_wx, last_wy = wx, wy

        self.move_car()

        while self.simulation.is_connected():
            self.simulation.step()
            self.car.stop()

            if self.args.show_cv and self.car.car_id is not None:
                img = self.vision.get_car_camera_image(self.car.car_id, 320, 240)
                _, dbg = self.vision.steering_from_yellow_line(img)
                cv2.imshow("car_cam_debug", dbg)
                if cv2.waitKey(1) & 0xFF == 27:
                    break

        if self.args.show_cv:
            cv2.destroyAllWindows()


def build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--start", type=float, nargs=2, default=[-15.0, -15.0], help="Start x y (world)")
    ap.add_argument("--goal", type=float, nargs=2, default=[+15.0, +15.0], help="Goal x y (world)")
    ap.add_argument("--cell", type=float, default=0.25)
    ap.add_argument("--show_cv", action="store_true", help="Show OpenCV debug window")
    return ap


def main():
    parser = build_argparser()
    args = parser.parse_args()
    app = CityCarPracticeApp(args)
    app.run()


if __name__ == "__main__":
    main()
