Plantilla POO para práctica de A* con PyBullet
=============================================

Contenido
---------
- practica_astar_poo/planner.py      -> clase AStarPlanner con NotImplementedError
- practica_astar_poo/city_map.py     -> clase CityMap y parámetros del mapa
- practica_astar_poo/car.py          -> clase RaceCar
- practica_astar_poo/vision.py       -> clase CarVision
- practica_astar_poo/simulation.py   -> clase Simulation
- practica_astar_poo/app.py          -> clase principal CityCarPracticeApp
- main.py                            -> punto de entrada

Comportamiento actual
---------------------
- El entorno se abre y se visualiza correctamente.
- El coche aparece en la carretera más cercana al punto inicial.
- El objetivo se marca visualmente.
- El coche NO se mueve.
- La parte de A* está vacía a propósito y lanza NotImplementedError.

Ejemplo de ejecución
--------------------
python main.py --gui --start -18 -18 --goal 18 18

O también:
python -m practica_astar_poo.app --gui --start -18 -18 --goal 18 18

Qué debe hacer el alumnado
--------------------------
Implementar en planner.py:
- heuristic(...)
- get_neighbors(...)
- reconstruct_path(...)
- astar(...)

Después, opcionalmente, se puede ampliar app.py para:
- dibujar la ruta encontrada
- mover el coche siguiendo la ruta
