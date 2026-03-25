from __future__ import annotations
from typing import List, Optional
import numpy as np
import heapq


class AStarPlanner:
    """
    Algoritmo A* para calcular la ruta más rápida de un punto 'A' a un punto 'B'.
    Toma en cuenta los obstaculos (marcados con 'x' en el grid) y el coste de los nodos.
    """

    def _heuristic(self, origen: tuple[int, int], destino: tuple[int, int], allow_diag: bool = False) -> float:
        """
        _Calcula la heuristica haciendo uso de la Distancia Manhattan o de 
        la distancia Euclidiana (allow_diag = True)._

        Args:
            origen (tuple[int, int]): _Coordenadas nodo actual_
            destino (tuple[int, int]): _Coordenadas meta_
            allow_diag (bool, optional): _description_. Defaults to False.

        Returns:
            float: _Distancia calculada_
        """

        x = abs(origen[0] - destino[0])
        y = abs(origen[1] - destino[1])

        # Distancia Euclidiana
        if allow_diag: 
            return np.sqrt(x^2 + y^2)

        # Distancia Manhattan
        return x + y 

    def _get_neighbors(self, current_node: Node, grid: np.ndarray, allow_diag: bool = False) -> list[tuple[int, int]]:
        """
        _Obtiene las casillas adyacentes al nodo dado.
        Si allow_diag es True, incluye también los vecinos diagonales._

        Args:
            current_node (Node):
            grid (np.ndarray): _Matriz del mapa._
            allow_diag (bool, optional): _Permitir vecinos diagonales._

        Returns:
            list([tuple[int, int]]): Lista de nodos vecino validados.
        """

        x, y = current_node.position
        grid_width, grid_height = grid.shape
        real_neighbours = []

        neighbours = [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1)
        ]

        if allow_diag:
            neighbours += [
                (x + 1, y + 1),
                (x + 1, y - 1),
                (x - 1, y + 1),
                (x - 1, y - 1)
            ]

        for neighbour_x, neighbour_y in neighbours:
            if (0 <= neighbour_x < grid_width) and (0 <= neighbour_y < grid_height) and (grid[neighbour_x][neighbour_y] != "x"):    
                coste_nodo = (current_node.g * np.sqrt(2)) if (neighbour_x != x and neighbour_y != y) else current_node.g # Si cambian tanto x como y el vecino es diagonal -> Aplicar pitagoras
                
                vecino_posicion = (neighbour_x, neighbour_y)
                g =  coste_nodo + int(grid[neighbour_x][neighbour_y])
                h = self._heuristic(vecino_posicion, self.goal)

                vecino = self.Node(vecino_posicion, g, h)
                real_neighbours.append(vecino)
        
        return real_neighbours

    def _reconstruct_path(self, last_node: Node) -> list[tuple[int, int]]:
        """
        _Obtiene el camino final desde el último nodo._ \n 
        _Se reconstruye el camino haciendo uso de la propiedad
        'padre' de la clase Node._

        Args:
            last_node (Node): _Nodo final desde el que comienza la reconstrucción del camino._

        Returns:
            list([tuple[int, int]]): _Lista de coordenadas del camino encontrado
            desde el nodo inicial hasta el nodo final._
        """
        final_path = []

        while True:
            final_path.append(last_node.position)

            if not last_node.padre:
                break

            last_node = last_node.padre

        return final_path[::-1]

    def a_star(self, grid: np.ndarray, start: tuple[int, int], goal: tuple[int, int], allow_diag: bool = False) -> list[tuple[int, int]]  | None:
        """
        _Calcula el camino más corto entre dos puntos haciendo uso de A*_

        Args:
            grid (np.ndarray): _Matriz númerica que representa el mapa con los costes de cada nodo. 'x' si el nodo es un obstaculo_
            start (tuple[int, int]): _Coordenadas (fila, columna) del nodo inicial_
            goal (tuple[int, int]): _Coordenadas (fila, columna) del nodo final_
            allow_diag (bool, optional): _Para permitir movimientos en diagonal_. Defaults to False.

        Returns:
           list([tuple[int, int]]): _Lista de coordenadas de lo nodos del camino encontrado
            de princpio a fin._ Devuelve 'None' si no es posible encontrar un camino.
        """

        num_iteraciones = 0
        open_list = []
        close_list = {} # Posiciones de los nodos como claves
        self.goal = goal

        h = self._heuristic(start, goal, allow_diag)
        nodo = self.Node(start, 0, h, None)

        heapq.heappush(open_list, nodo)
   
        while open_list:
            num_iteraciones += 1

            # Buscar nodo de menor coste
            current_node = heapq.heappop(open_list)

            # Marcar nodo como explorado (mover al diccionario)
            close_list[current_node.position] = current_node

            if current_node.position == goal:
                print(f"Se ha encontrado la meta. \n Número de iteraciones {num_iteraciones}")
                return self._reconstruct_path(current_node)
            
            # Encontrar vecinos
            neighbours = self._get_neighbors(current_node, grid, allow_diag)

            for neighbour in neighbours:
                # Comprobar si un vecino ya esta explorado (closed list)
                if neighbour.position in close_list:
                    old_neighbour = close_list[neighbour.position]

                    # Comprobar si el nuevo padre tiene una ruta mas corta (g)
                    if neighbour.g < old_neighbour.g:
                        old_neighbour.g = neighbour.g
                        old_neighbour.f = neighbour.g + old_neighbour.h
                        old_neighbour.padre = current_node
                    
                        # Como se ha modificado (no explorado) se mueve a la open list otra vez
                        del close_list[neighbour.position]
                        heapq.heappush(open_list, old_neighbour)           

                # Comprobar si un vecino ya esta en la open list
                else:
                    neighbour_in_open_list = next((x for x in open_list if neighbour.position == x.position), None)

                    if neighbour_in_open_list is None:
                        neighbour.padre = current_node
                        heapq.heappush(open_list, neighbour)

                    # Comprobar si el nuevo padre tiene una ruta mas corta (g)
                    elif neighbour.g < neighbour_in_open_list.g:
                        neighbour_in_open_list.g = neighbour.g
                        neighbour_in_open_list.f = neighbour.g + neighbour_in_open_list.h
                        neighbour_in_open_list.padre = current_node
                        heapq.heapify(open_list)

        print(f"No es posible acceder al nodo. Número de iteraciones {num_iteraciones}")
        return None # Imposible acceder

    class Node:
        """
        _Representa un nodo del mapa._

        _Cada nodo almacena su posición (x, y), el coste acumulado
        desde el inicio (g), el valor heurístico hasta el objetivo (h) y
        el coste total (f = g + h)._
        
        _También se guarda un puntero/referencia al padre para poder reconstruir el camino._
        """
        def __init__(self, position: tuple[int, int], g: int, h: int, padre: object = None):
            
            self.position = position
            self.g: int = g     # Distancia recorrida + coste del nodo padre a este
            self.h: int = h     # Valor heuristica
            self.f = self.g + self.h
            self.padre = padre

        def __lt__(self, other): 
            """
            _Para establecer el criterio de comparacion entre nodos al usar el operador '<'_

            _Primero se compara el coste total (f), luego la heuristica (h) y,
            en caso de empate, la posición. La posición al ser unica permite 
            desempatar en caso de que tengan los mismos costes_

            Args:
                other (Node): _El otro nodo con el que se compara_

            Returns:
                bool: _True si este nodo (self) tiene menor prioridad (menor coste) que el otro (other)._
            """
            if self.f != other.f:
                return self.f < other.f
            elif self.h != other.h:
                return self.h < other.h
            else: # Desempate por posicion
                return self.position < other.position
        
def main():
    start = (0, 0)
    goal = (4,2)

    grid = np.array([
        [1,  1,  1,  1,  1],
        [1, "x", 1, "x", 1],
        [1,  1,  2,  1,  1],
        [1, "x", 1, "x", 1],
        [1,  1,  1,  1,  1]
    ])

    pathfinding = AStarPlanner()
    path = pathfinding.a_star(grid, start, goal, allow_diag=True)
    print(path)

if __name__ == "__main__":
    main()