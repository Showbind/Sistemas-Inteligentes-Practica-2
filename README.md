# Práctica 2: CONDUCCIÓN AUTÓNOMA CON A*

> **Sistemas Inteligentes**
> <br>
> Práctica de planificación de rutas usando el algoritmo A* en un entorno de simulación 3D con PyBullet.
> 
## Descripción

Este proyecto implementa un simulador de conducción autónoma donde un coche se desplaza por el mapa usando el algoritmo **A\*** para calcular la ruta más óptima. 
<br><br>El entorno se visualiza en tiempo real usando **PyBullet**.

### Características principales

- **Algoritmo A\*:** Cálculo de rutas óptimas con costes variables y uso opcional de desplazamiento con diagonales
- **Simulación 3D:** Física de vehículos con PyBullet
- **Control automático del vehículo**: Dirección y aceleración 
- **Mundo dinámico:** Mapa con carreteras, intersecciones y obstáculos

## Estructura del Proyecto

```
├── main.py                   # Ejecutable
├── requirements.txt          # Dependencias
├── README.md                  
└── src/
    ├── __init__.py
    ├── planner.py            # Algoritmo A* 
    ├── car.py                # Simulador del vehículo 
    ├── city_map.py           # Generación del mapa
    ├── simulation.py         # Motor de simulación 
    ├── vision.py             # Procesamiento de visión 
    └── app.py                # Aplicación principal 
```

## Componentes Principales

### 1. **AStarPlanner** (`src/planner.py`)

Implementa el algoritmo A* completo con:

- **Heurística configurable**: Manhattan o Euclidiana (según `allow_diag`)
- **Obtención de vecinos**: Soporte para movimientos ortogonales y diagonales
- **Reconstrucción de rutas**: Traceback desde el nodo objetivo al inicio
- **Costes variables**: Toma en cuenta el coste de cada nodo y los obstáculos

```python
class Node:
    position      # (row, col) en el mapa
    g            # Coste acumulado desde el inicio
    h            # Heurística hasta el objetivo
    f = g + h    # Coste total estimado
    padre        # Referencia al nodo anterior (para reconstruir la ruta)
```

### 2. **RaceCar** (`src/car.py`)

Controla la física y movimiento del vehículo:

- **Spawning**: Crea el vehículo en coordenadas específicas
- **Control**: Dirección y aceleración via `set_control(steering, throttle)`
- **Lectura de posición**: Obtiene posición, orientación y ángulo yaw
- **Fricción realista**: Configuración de fricción lateral y giro

### 3. **CityMap** (`src/city_map.py`)

Genera el mundo y mantiene la representación en cuadrículas:

- **Mapa**: Carreteras horizontales y verticales, intersecciones y bordes
- **Conversiones**: `world_to_grid()` y `grid_to_world()` para cambio de coordenadas.
- **Visualización**: `draw_path()` dibuja el camino en la simulación
- **Grilla de costes**: Marca obstáculos (-1) y caminos válidos (>0)

### 4. **Simulation** (`src/simulation.py`)

Motor de simulación y renderizado:

- **Conexión con PyBullet**: Uso de GUI
- **Visualización**: Cámara top-down sincronizada con el mapa


### 5. **CityCarPracticeApp** (`src/app.py`)

Integra todos los componentes:

1. **Setup**: Inicializa la simulación, el mapa y vehículo
2. **Planificación**: Calcula ruta A* del inicio al objetivo
3. **Visualización**: Dibuja líneas del camino
4. **Movimiento**: El vehículo sigue la ruta punto a punto
5. **Loop principal**: Mantiene la simulación activa

## Dependencias

```
opencv-python      # Visión computacional 
pybullet           # Motor de físicas
```

## Instalación

### Requisitos
- Python 3.9+
- pip

### Pasos

1. **Clonar el repositorio**
```bash
git clone <repositorio>
cd "Sistemas-Inteligentes-Practica-2"
```

2. **Crear & Activar entorno virtual (recomendado)**
```bash
python -m venv .venv

# En Windows:
.venv\Scripts\activate

# En Linux/MacOS:
source .venv/bin/activate
```

3. **Instalar dependencias**
```bash
pip install -r requirements.txt
```



4. **Ejecutar**

```bash
python main.py --gui --start -15 -15 --goal 15 15
```

### Parámetros de ejecución en CLI

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `--gui` | flag | Habilitar visualización gráfica |
| `--start` | int, int | Coordenadas iniciales en el mundo (x, y) |
| `--goal` | int, int | Coordenadas objetivo en el mundo (x, y) |
| `--cell` | float | Tamaño de celda en metros (default: 0.25) |
| `--show_cv` | flag | Mostrar ventana OpenCV con debug de cámara |

### Ejemplos de uso

```bash
# Simulación con GUI, inicio en (-15, -15), objetivo en (15, 15)
python main.py --gui --start -15 -15 --goal 15 15

# Con visualización de cámara del vehículo
python main.py --gui --start -10 -10 --goal 10 10 --show_cv

# Cambiar tamaño de las celdas (celdas más pequeñas = más precisión pero cálculo más lento de A*)
python main.py --gui --cell 0.2 --start -15 -15 --goal 15 15

# Sin GUI (simulación en background)
python main.py --start -15 -15 --goal 15 15
```

## Configuración Avanzada

### Parámetros del vehículo (`src/app.py`)

```python
THROTTLE = 30              # Velocidad forward (unidades PyBullet)
MAX_STEER = 0.6           # Ángulo máximo de dirección (radianes)
DISTANCE_TOLERANCE = 0.5  # Distancia antes de cambiar al 
```

### Parámetros del mapa (`src/city_map.py`)

```python
@dataclass
class MapParams:
    plane_size: float = 40.0            # Tamaño total del mapa
    road_half_width: float = 5.5        # Ancho de carreteras
    intersection_half: float = 7.5      # Tamaño de intersecciones
    cell: float = 0.25                  # Resolución de la grilla
```

## Cómo Funciona el Algoritmo A*

1. **Inicialización**: Se crea un nodo raíz con g=0 y h=heurística(inicio, objetivo)

2. **Bucle principal**:
   - Extrae nodo con menor f de la lista abierta (open list)
   - Si es el objetivo, reconstruye el camino.
   - Obtiene todos los vecinos válidos
   - Para cada vecino:
     - Si está en la lista cerrada y nuevo g es mejor, se cambia el padre y se mueve a la lista abierta 
     - Si no está en ninguna lista, se añade a la list abierta
     - Si está en la lista abierta y nuevo g es mejor, se actualiza su padre

3. **Reconstrucción**: Desde el nodo objetivo, sigue los punteros padre hasta el nodo inicio extrayendo las posiciones de estos nodos.
    
    **Costes**: 
   - Movimiento ortogonal: `1.0 * coste_celda`
   - Movimiento diagonal: `√2 * coste_celda`
