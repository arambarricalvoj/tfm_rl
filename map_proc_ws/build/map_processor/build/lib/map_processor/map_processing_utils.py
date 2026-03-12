# map_processor/map_processing.py
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import threading
from typing import Optional, Tuple, Sequence

UNKNOWN = -1
OCCUPIED = 100
FREE = 0

class MapProcessor:
    """
    Clase para mantener y procesar un OccupancyGrid.
    - map_data: numpy array shape (height, width) dtype=int (values -1,0..100)
    - width, height: en celdas
    - resolution: metros por celda
    - origin: geometry_msgs/Pose (x,y,yaw) or tuple (x,y,yaw)
    - atributos calculados: free_count, occupied_count, unknown_count, coverage, etc.
    """

    def __init__(self):
        # mapa y metadatos
        self.map_data: Optional[np.ndarray] = None
        self.width: Optional[int] = None
        self.height: Optional[int] = None
        self.resolution: Optional[float] = None
        self.origin: Optional[Tuple[float, float, float]] = None  # (x, y, yaw) si se proporciona
        self.robot_pose: Optional[Tuple[float, float, float]] = None

        # métricas
        self.free_count: int = 0
        self.occupied_count: int = 0
        self.unknown_count: int = 0
        self.total_count: int = 0

        self.free_percent: float = 0.0
        self.occupied_percent: float = 0.0
        self.unknown_percent: float = 0.0

        self.free_ratio_known: float = 0.0
        self.coverage: float = 0.0  # free + occupied / (free + occupied + unkwown)
        self.mean_occupancy: float = 0.0  # media de valores (ignorando unknown)
        self.size_m2: Optional[float] = None  # area en m^2 (width*height*resolution^2)

        # bounding box de celdas ocupadas: (min_row, min_col, max_row, max_col) en índices de matriz
        self.occupied_bbox: Optional[Tuple[int, int, int, int]] = None

    # ---------- Actualización del mapa ----------
    def update_from_occupancy_grid(self, msg) -> None:
        """
        Actualiza el mapa a partir de un nav_msgs/OccupancyGrid.
        msg.data: lista de ints (row-major, width*height)
        msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin
        """
        data = msg.data
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        # origin: intentar extraer x,y,yaw si está disponible
        origin_pose = None
        try:
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y
            q = msg.info.origin.orientation
            # calcular yaw de quaternion
            yaw = self._quat_to_yaw(q)
            origin_pose = (ox, oy, yaw)
        except Exception:
            origin_pose = None

        self.update_from_data(data, width, height, resolution, origin_pose)

    def update_from_data(self,
                         data: Sequence[int],
                         width: int,
                         height: int,
                         resolution: float,
                         origin: Optional[Tuple[float, float, float]] = None) -> None:
        """
        Actualiza el mapa desde datos crudos.
        - data: secuencia de length width*height (row-major)
        - width, height: enteros
        - resolution: float (m/celda)
        - origin: opcional (x,y,yaw)
        """
        arr = np.array(data, dtype=int)
        if arr.size != width * height:
            raise ValueError("Tamaño de data no coincide con width*height")

        # reshape a (height, width) en row-major (ROS OccupancyGrid usa row-major)
        self.map_data = arr.reshape((height, width))
        self.width = width
        self.height = height
        self.resolution = float(resolution)
        self.origin = origin

        # recalcular métricas
        self._compute_stats()

    # ---------- Cálculos internos ----------
    def _compute_stats(self) -> None:
        if self.map_data is None:
            # reset métricas
            self.free_count = self.occupied_count = self.unknown_count = self.total_count = 0
            self.free_percent = self.occupied_percent = self.unknown_percent = 0.0
            self.coverage = 0.0
            self.mean_occupancy = 0.0
            self.size_m2 = None
            self.occupied_bbox = None
            return

        flat = self.map_data.ravel()
        self.total_count = flat.size
        self.free_count = int((flat == FREE).sum())
        self.occupied_count = int((flat >= 50).sum())  # umbral: >=50 como ocupado
        self.unknown_count = int((flat == UNKNOWN).sum())

        # porcentajes
        self.free_percent = self.free_count / self.total_count * 100.0
        self.occupied_percent = self.occupied_count / self.total_count * 100.0
        self.unknown_percent = self.unknown_count / self.total_count * 100.0

        # coverage: fracción de celdas conocidas que están libres
        known = self.free_count + self.occupied_count
        self.coverage = (self.free_count / known) if known > 0 else 0.0

        # media de ocupación (ignorando unknown)
        known_vals = flat[flat != UNKNOWN]
        self.mean_occupancy = float(known_vals.mean()) if known_vals.size > 0 else 0.0

        # tamaño en metros cuadrados
        self.size_m2 = (self.width * self.height) * (self.resolution ** 2)

        # bounding box de ocupadas (si existen)
        occ_indices = np.argwhere(self.map_data >= 50)
        if occ_indices.size == 0:
            self.occupied_bbox = None
        else:
            rows = occ_indices[:, 0]
            cols = occ_indices[:, 1]
            self.occupied_bbox = (int(rows.min()), int(cols.min()), int(rows.max()), int(cols.max()))

    # ---------- Métodos de consulta útiles ----------
    def get_free_cells(self) -> int:
        return self.free_count

    def get_occupied_cells(self) -> int:
        return self.occupied_count

    def get_unknown_cells(self) -> int:
        return self.unknown_count

    def get_coverage(self) -> float:
        return self.coverage

    def get_size_m2(self) -> Optional[float]:
        return self.size_m2

    def get_occupied_bbox(self) -> Optional[Tuple[int, int, int, int]]:
        return self.occupied_bbox

    def get_map_copy(self) -> Optional[np.ndarray]:
        return None if self.map_data is None else self.map_data.copy()
    
        # ---------- Métodos de pose del robot ----------
    def set_robot_pose(self, x: float, y: float, yaw: Optional[float] = None) -> None:
        """
        Establece la pose del robot en coordenadas del frame map.
        robot_pose se guarda como (x, y, yaw) donde yaw puede ser None.
        """
        self.robot_pose = (float(x), float(y), float(yaw) if yaw is not None else None)

    def set_robot_pose_from_pose_msg(self, pose_msg) -> None:
        # Normalizar al objeto Pose (tiene .position y .orientation)
        if hasattr(pose_msg, 'pose') and hasattr(pose_msg.pose, 'pose'):
            # PoseWithCovarianceStamped
            p = pose_msg.pose.pose
        elif hasattr(pose_msg, 'pose') and hasattr(pose_msg.pose, 'position'):
            # PoseWithCovariance
            p = pose_msg.pose
        elif hasattr(pose_msg, 'position'):
            # Pose directo
            p = pose_msg
        else:
            raise TypeError("Tipo de mensaje de pose no reconocido")

        pos = p.position
        ori = p.orientation
        yaw = self._quat_to_yaw(ori)
        self.set_robot_pose(pos.x, pos.y, yaw)

    def world_to_map_indices(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """
        Convierte coordenadas (x,y) en el frame map a índices (row, col).
        Devuelve (row, col) o None si está fuera de rango o mapa no inicializado.
        Tiene en cuenta origin (ox,oy,oyaw) y resolution.
        """
        if self.map_data is None or self.width is None or self.height is None or self.resolution is None:
            return None

        ox, oy, oyaw = self.origin or (0.0, 0.0, 0.0)
        res = self.resolution
        h = self.height
        w = self.width

        # trasladar punto al origen del mapa
        dx = x - ox
        dy = y - oy

        # rotar por -oyaw (si origin tiene rotación)
        if oyaw:
            cos_r = np.cos(-oyaw)
            sin_r = np.sin(-oyaw)
            lx = cos_r * dx - sin_r * dy
            ly = sin_r * dx + cos_r * dy
        else:
            lx, ly = dx, dy

        # columna = x local / res, fila = y local / res
        col = int(np.floor(lx / res))
        row = int(np.floor(ly / res))

        # Ajuste de convención de filas
        # Asumimos que origin corresponde a la esquina inferior izquierda.
        # Si tu OccupancyGrid usa otra convención, invierte row: row = h - 1 - row
        if 0 <= row < h and 0 <= col < w:
            return (row, col)
        return None

    def get_robot_cell(self) -> Optional[Tuple[int, int]]:
        """
        Devuelve (row, col) de la celda donde está self.robot_pose o None.
        """
        if self.robot_pose is None:
            return None
        x, y, _ = self.robot_pose
        return self.world_to_map_indices(x, y)

    def get_robot_cell_value(self) -> Optional[int]:
        """
        Devuelve el valor de ocupación de la celda donde está el robot (0..100 o -1),
        o None si no hay mapa o la pose está fuera.
        """
        cell = self.get_robot_cell()
        if cell is None:
            return None
        row, col = cell
        return int(self.map_data[row, col])
    
    # requiere al principio del archivo:
    # import matplotlib.pyplot as plt
    # import matplotlib.colors as mcolors
    # import threading

    # Métodos de plotting interactivo
    def start_plot(self, figsize=(8, 8), title="Occupancy Grid"):
        """
        Inicia la ventana matplotlib en modo interactivo.
        Llamar una vez (por ejemplo en __init__ del nodo).
        """
        if getattr(self, "_plot_started", False):
            return
        try:
            plt.ion()
            self._fig, self._ax = plt.subplots(figsize=figsize)
            self._ax.set_title(title)
            self._im = None
            self._robot_scatter = None
            self._plot_lock = threading.Lock()
            # crear colormap: unknown=gray, free=white, occupied=black
            cmap = mcolors.ListedColormap(['gray', 'white', 'black'])
            bounds = [-1, 0.5, 50, 101]  # bins: unknown(-1), free(0..49), occ(50..100)
            self._norm = mcolors.BoundaryNorm(bounds, cmap.N)
            self._cmap = cmap
            self._plot_started = True
        except Exception as e:
            # si no hay entorno gráfico, marcar como no iniciado
            self._plot_started = False
            raise

    def stop_plot(self):
        """Cierra la ventana si está abierta."""
        if not getattr(self, "_plot_started", False):
            return
        try:
            plt.ioff()
            plt.close(self._fig)
        finally:
            self._plot_started = False
            self._im = None
            self._robot_scatter = None

    def update_plot(self, draw=True):
        """
        Actualiza la imagen del mapa y la pose del robot.
        Llamar desde la callback /map o /pose cada vez que haya cambios.
        Si draw=False solo actualiza los datos internos sin llamar a plt.pause.
        """
        if not getattr(self, "_plot_started", False):
            return

        with self._plot_lock:
            if self.map_data is None:
                # limpiar imagen si no hay mapa
                if self._im is not None:
                    self._im.set_data(np.zeros((1, 1)))
                    if draw:
                        plt.pause(0.001)
                return

            # preparar matriz para mostrar con tres categorías:
            # unknown -> 0, free -> 1, occupied -> 2
            disp = np.full(self.map_data.shape, 0, dtype=int)  # default unknown
            disp[self.map_data == FREE] = 1
            disp[self.map_data >= 50] = 2

            if self._im is None:
                self._im = self._ax.imshow(disp, cmap=self._cmap, norm=self._norm,
                                        origin='lower', interpolation='nearest')
                self._ax.set_xlim(-0.5, self.width - 0.5)
                self._ax.set_ylim(-0.5, self.height - 0.5)
                self._ax.set_xlabel('col')
                self._ax.set_ylabel('row')
            else:
                self._im.set_data(disp)
                # ajustar límites si el tamaño del mapa cambió
                self._im.set_extent((-0.5, self.width - 0.5, -0.5, self.height - 0.5))
                self._ax.set_xlim(-0.5, self.width - 0.5)
                self._ax.set_ylim(-0.5, self.height - 0.5)

            # dibujar o actualizar la pose del robot en coordenadas de celda
            if self.robot_pose is not None:
                cell = self.get_robot_cell()
                if cell is not None:
                    row, col = cell
                    # si no existe el scatter, crearlo; si existe, actualizar sus datos
                    if self._robot_scatter is None:
                        self._robot_scatter = self._ax.scatter([col], [row], c='red', s=50, marker='o', zorder=5)
                    else:
                        self._robot_scatter.set_offsets([[col, row]])
                else:
                    # si la pose está fuera del mapa, eliminar el scatter si existe
                    if self._robot_scatter is not None:
                        self._robot_scatter.remove()
                        self._robot_scatter = None
            else:
                if self._robot_scatter is not None:
                    self._robot_scatter.remove()
                    self._robot_scatter = None

            if draw:
                # pausa corta para que matplotlib procese eventos y actualice la ventana
                plt.pause(0.001)



    # ---------- utilidades ----------
    @staticmethod
    def _quat_to_yaw(q) -> float:
        """
        Convierte geometry_msgs/Quaternion-like a yaw (radians).
        q debe tener atributos x,y,z,w
        """
        import math
        x = getattr(q, 'x', 0.0)
        y = getattr(q, 'y', 0.0)
        z = getattr(q, 'z', 0.0)
        w = getattr(q, 'w', 1.0)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # Representación breve
    def __repr__(self):
        return (f"<MapProcessor size={self.width}x{self.height} res={self.resolution} "
                f"free={self.free_count} occ={self.occupied_count} unk={self.unknown_count} "
                f"coverage={self.coverage:.3f}>")
