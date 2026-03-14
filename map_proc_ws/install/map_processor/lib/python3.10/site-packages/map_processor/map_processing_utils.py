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
        self.robot_pose: Optional[Tuple[float, float, float]] = (0, 0, 0)

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
        # bounding box de zona explorada: (min_row, min_col, max_row, max_col) en índices de matriz
        self.explored_bbox: Optional[Tuple[int, int, int, int]] = None

        self.occupied_bbox_size: Optional[Tuple[int, int]] = None
        self.explored_bbox_size: Optional[Tuple[int, int]] = None

        self.prob_explored_bbox: Optional[Tuple[int, int, int, int]] = None
        self.prob_explored_bbox_size: Optional[Tuple[int, int]] = (24, 24)

        self.prob_neighborhood_scale = 1.0   # factor para ampliar la vecindad base
        self.prob_kernel_type = "uniform"    # "uniform" o "gaussian"
        self.prob_kernel = None              # si no es None, usar kernel 2D personalizado (se normaliza)
        self.prob_return_uint8 = False       # si True devuelve 0..255 uint8, si False devuelve float 0..1


        # ---------- Local Egocentric Map ----------
        # tamaño deseado de la LEM en celdas (H, W). Por defecto 24x24.
        self.lem_size: Tuple[int, int] = (50, 50)
        # almacenamiento de la LEM (numpy array HxW uint8 con valores 0,128,255) o None
        self.lem_map: Optional[np.ndarray] = None
        self.lem_mark_agent: bool = False
        self.lem_agent_marker_size: int = 1
        self.lem_agent_marker_value: int = 128

        self.lem_rotation_offset_deg = -90.0

        # ---------- plotting (3 panels) ----------
        self._fig = None
        self._axs = None
        self._im_list = [None, None, None]
        self._plot_lock = threading.Lock()


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
            self.occupied_bbox = None
            self.explored_bbox = None
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
            min_r, min_c, max_r, max_c = int(rows.min()), int(cols.min()), int(rows.max()), int(cols.max())
            self.occupied_bbox = (min_r, min_c, max_r, max_c)
            self.occupied_bbox_size = (max_r - min_r + 1, max_c - min_c + 1)
        # bounding box de la región explorada (celdas conocidas: != UNKNOWN)
        known_indices = np.argwhere(self.map_data != UNKNOWN)
        if known_indices.size == 0:
            self.explored_bbox = None
        else:
            k_rows = known_indices[:, 0]
            k_cols = known_indices[:, 1]
            k_min_r, k_min_c, k_max_r, k_max_c = int(k_rows.min()), int(k_cols.min()), int(k_rows.max()), int(k_cols.max())
            self.explored_bbox = (k_min_r, k_min_c, k_max_r, k_max_c)
            self.explored_bbox_size = (k_max_r - k_min_r + 1, k_max_c - k_min_c + 1)

            # construir explored_map: known -> 255, unknown -> 0
            sub = self.map_data[k_min_r:k_max_r + 1, k_min_c:k_max_c + 1]
            gem = np.zeros(sub.shape, dtype=np.uint8)
            gem[sub != UNKNOWN] = 255
            self.explored_map = gem

        self.compute_prob_explored_map(out_size=self.prob_explored_bbox_size,
                                   neighborhood_scale=self.prob_neighborhood_scale,
                                   kernel=None,
                                   kernel_type=self.prob_kernel_type,
                                   return_uint8=False)

        # Generar LEM automáticamente si hay pose del robot
        try:
            if self.robot_pose is not None:
                # generate_lem maneja comprobaciones internas y guarda en self.lem_map
                print("robot_pose:", self.robot_pose)   # x, y, yaw (radians)
                self.generate_lem(pad=0)
        except Exception:
            # no queremos que un fallo en LEM rompa el cálculo de stats
            self.lem_map = None


    def _map_values_to_lem(self, submap: np.ndarray) -> np.ndarray:
        """
        Convierte submap (valores -1,0..100) a LEM codificado (0,128,255).
        """
        out = np.full(submap.shape, 128, dtype=np.uint8)  # unknown -> 128
        out[submap == FREE] = 0
        out[submap >= 50] = 255
        out[(submap > FREE) & (submap < 50)] = 0
        return out
    
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
    
    def get_explored_bbox(self) -> Optional[Tuple[int, int, int, int]]:
        """
        Devuelve el bounding box de la región explorada (celdas conocidas: free u occupied)
        como (min_row, min_col, max_row, max_col) o None si no hay celdas conocidas.
        """
        return self.explored_bbox
    
    def get_occupied_bbox_size(self) -> Optional[Tuple[int, int]]:
        """
        Devuelve (height_rows, width_cols) en celdas del occupied_bbox, o None si no existe.
        """
        return self.occupied_bbox_size

    def get_explored_bbox_size(self) -> Optional[Tuple[int, int]]:
        """
        Devuelve (height_rows, width_cols) en celdas del explored_bbox, o None si no existe.
        """
        return self.explored_bbox_size

    def get_map_copy(self) -> Optional[np.ndarray]:
        return None if self.map_data is None else self.map_data.copy()

    def get_explored_map_copy(self, pad: int = 0) -> Optional[np.ndarray]:
        """
        Devuelve una copia del submapa correspondiente al explored_bbox.
        - pad: número de celdas a expandir alrededor del bbox (>=0).
        Retorna None si no hay mapa o no hay región explorada.
        """
        if self.map_data is None:
            return None
        if self.explored_bbox is None:
            return None

        min_r, min_c, max_r, max_c = self.explored_bbox

        # aplicar padding y recortar a los límites del mapa
        if pad < 0:
            pad = 0
        r0 = max(0, min_r - pad)
        c0 = max(0, min_c - pad)
        r1 = min(self.height - 1, max_r + pad)
        c1 = min(self.width - 1, max_c + pad)

        # slice inclusive: numpy usa r0:r1+1, c0:c1+1
        sub = self.map_data[r0:r1 + 1, c0:c1 + 1]
        return sub.copy()
    
    def get_lem_copy(self, generate_if_missing: bool = True, pad: int = 0) -> Optional[np.ndarray]:
        """
        Devuelve una copia de la LEM (HxW uint8). Si no existe y generate_if_missing=True,
        la genera con generate_lem(pad).
        """
        if self.lem_map is None and generate_if_missing:
            return self.generate_lem(pad=pad)
        return None if self.lem_map is None else self.lem_map.copy()


    # Método para computar y almacenar el mapa reducido de probabilidades
    def compute_prob_explored_map(self,
                                out_size: tuple | None = None,
                                neighborhood_scale: float | None = None,
                                kernel: np.ndarray | None = None,
                                kernel_type: str | None = None,
                                return_uint8: bool | None = None):
        """
        Calcula y guarda self.prob_explored_map_reduced y self.prob_explored_bbox.
        - out_size: (H_out, W_out). Si None usa self.prob_explored_bbox_size.
        - neighborhood_scale: factor multiplicador del tamaño base de vecindad.
        - kernel: kernel 2D personalizado (se normaliza). Si None se genera según kernel_type.
        - kernel_type: "uniform" o "gaussian".
        - return_uint8: si True guarda en 0..255 uint8, si False en float 0..1.
        Retorna el mapa reducido (H_out, W_out).
        """
        # parámetros por defecto desde atributos
        if out_size is None:
            out_size = self.prob_explored_bbox_size
        if neighborhood_scale is None:
            neighborhood_scale = self.prob_neighborhood_scale
        if kernel_type is None:
            kernel_type = self.prob_kernel_type
        if kernel is None:
            kernel = self.prob_kernel
        if return_uint8 is None:
            return_uint8 = self.prob_return_uint8

        if getattr(self, "map_data", None) is None:
            raise RuntimeError("map_data no disponible")

        H_in, W_in = self.map_data.shape
        H_out, W_out = out_size

        # máscara unknown: 1 si unknown (-1), 0 si conocido (0 o >=50)
        unknown_mask = (self.map_data == -1).astype(np.float32)

        # escala por celda reducida (celdas originales por celda reducida)
        scale_r = H_in / float(H_out)
        scale_c = W_in / float(W_out)
        # tamaño representativo (media simple)
        scale = max(1.0, 0.5 * (scale_r + scale_c))

        # tamaño base de vecindad en celdas (al menos 1)
        base_size = max(1, int(np.ceil(scale)))
        ksize = max(1, int(np.ceil(base_size * neighborhood_scale)))
        if ksize % 2 == 0:
            ksize += 1

        # construir kernel si no se pasa uno
        if kernel is None:
            if kernel_type == "uniform":
                kernel_local = np.ones((ksize, ksize), dtype=np.float32)
            elif kernel_type == "gaussian":
                sigma = max(0.5, ksize / 6.0)
                ax = np.linspace(-(ksize // 2), ksize // 2, ksize)
                xx, yy = np.meshgrid(ax, ax)
                kernel_local = np.exp(-(xx**2 + yy**2) / (2.0 * sigma**2)).astype(np.float32)
            else:
                raise ValueError(f"kernel_type desconocido: {kernel_type}")
        else:
            kernel_local = np.array(kernel, dtype=np.float32)
            kh, kw = kernel_local.shape
            if kh % 2 == 0 or kw % 2 == 0:
                pad_h = 1 if kh % 2 == 0 else 0
                pad_w = 1 if kw % 2 == 0 else 0
                kernel_local = np.pad(kernel_local, ((0,pad_h),(0,pad_w)), mode='constant', constant_values=0)

        # normalizar kernel
        s = kernel_local.sum()
        if s <= 0:
            kernel_local = np.ones_like(kernel_local, dtype=np.float32)
            s = kernel_local.sum()
        kernel_local = kernel_local / s

        # intentar usar scipy para convolución; si no, usar convolución simple (menos eficiente)
        try:
            from scipy.signal import convolve2d
            conv = convolve2d(unknown_mask, kernel_local, mode='same', boundary='fill', fillvalue=0)
        except Exception:
            # convolución manual por FFT sería mejor, pero implementamos un método directo para kernels pequeños
            kh, kw = kernel_local.shape
            pad_h = kh // 2
            pad_w = kw // 2
            padded = np.pad(unknown_mask, ((pad_h, pad_h), (pad_w, pad_w)), mode='constant', constant_values=0)
            conv = np.zeros_like(unknown_mask, dtype=np.float32)
            for r in range(H_in):
                r0 = r
                r1 = r + kh
                for c in range(W_in):
                    c0 = c
                    c1 = c + kw
                    window = padded[r0:r1, c0:c1]
                    conv[r, c] = float((window * kernel_local).sum())

        # ahora agregamos conv en bloques para reducir a out_size
        out = np.zeros((H_out, W_out), dtype=np.float32)
        for i in range(H_out):
            r0 = int(np.floor(i * scale_r))
            r1 = int(np.floor((i + 1) * scale_r))
            if r1 <= r0:
                r1 = r0 + 1
            r1 = min(H_in, r1)
            for j in range(W_out):
                c0 = int(np.floor(j * scale_c))
                c1 = int(np.floor((j + 1) * scale_c))
                if c1 <= c0:
                    c1 = c0 + 1
                c1 = min(W_in, c1)
                block = conv[r0:r1, c0:c1]
                if block.size == 0:
                    out[i, j] = 0.0
                else:
                    out[i, j] = float(block.mean())

        out = np.clip(out, 0.0, 1.0)

        # guardar bbox y mapa reducido en la instancia
        # prob_explored_bbox: si quieres limitar a una subregión, puedes calcular r0,c0,r1,c1; por defecto usamos todo el mapa
        self.prob_explored_bbox = (0, 0, H_in - 1, W_in - 1)
        self.prob_explored_map_reduced = (out * 255.0).astype(np.uint8) if return_uint8 else out
        self.prob_explored_bbox_size = (H_out, W_out)
        self.prob_neighborhood_scale = neighborhood_scale
        self.prob_kernel_type = kernel_type
        self.prob_kernel = kernel_local

        return self.prob_explored_map_reduced

    # Método getter que devuelve copia del mapa reducido (24x24)
    def get_prob_explored_map_copy(self, as_uint8: bool | None = None):
        """
        Devuelve una copia del mapa reducido (self.prob_explored_map_reduced).
        Si no existe, lo calcula con compute_prob_explored_map().
        """
        if as_uint8 is None:
            as_uint8 = self.prob_return_uint8
        if not hasattr(self, "prob_explored_map_reduced"):
            self.compute_prob_explored_map(return_uint8=as_uint8)
        out = self.prob_explored_map_reduced
        if as_uint8 and out.dtype != np.uint8:
            return (out * 255.0).astype(np.uint8).copy()
        if (not as_uint8) and out.dtype == np.uint8:
            return (out.astype(np.float32) / 255.0).copy()
        return out.copy()

    
    def generate_lem(self, pad: int = 0) -> Optional[np.ndarray]:
        """
        Genera y guarda en self.lem_map la Local Egocentric Map HxW centrada en la pose del robot
        y alineada con la orientación del robot (theta). Usa nearest-neighbor para rotación.
        - pad: celdas extra alrededor del centro antes de recortar al tamaño HxW.
        Retorna la LEM (numpy uint8 HxW) o None si no hay mapa o pose del robot.
        """
        if self.map_data is None or self.robot_pose is None:
            return None

        H, W = self.lem_size
        rc = self.get_robot_cell()
        if rc is None:
            return None
        center_r, center_c = rc
        yaw = self.robot_pose[2] if len(self.robot_pose) > 2 else 0.0

        # tamaño mínimo del buffer para rotar sin recorte: usar diagonal
        diag = int(np.ceil(np.sqrt(H * H + W * W)))
        side = max(H, W, diag) + 2 * max(0, pad)
        # asegurar side impar para centrar mejor (opcional)
        if side % 2 == 0:
            side += 1

        half_side = side // 2
        r0 = center_r - half_side
        c0 = center_c - half_side
        r1 = r0 + side - 1
        c1 = c0 + side - 1

        # recortar a límites del mapa y crear buffer relleno con UNKNOWN
        r0_clip = max(0, r0)
        c0_clip = max(0, c0)
        r1_clip = min(self.height - 1, r1)
        c1_clip = min(self.width - 1, c1)

        buf_h = (r1 - r0 + 1)
        buf_w = (c1 - c0 + 1)
        buffer = np.full((buf_h, buf_w), UNKNOWN, dtype=int)

        dst_r0 = r0_clip - r0
        dst_c0 = c0_clip - c0
        dst_r1 = dst_r0 + (r1_clip - r0_clip)
        dst_c1 = dst_c0 + (c1_clip - c0_clip)

        buffer[dst_r0:dst_r1 + 1, dst_c0:dst_c1 + 1] = self.map_data[r0_clip:r1_clip + 1, c0_clip:c1_clip + 1]

        # rotación: queremos que la orientación del robot (yaw) apunte hacia 'arriba' en la LEM.
        # Por tanto rotamos el buffer por -yaw (grados).
        #angle_deg = -np.degrees(yaw)
        angle_deg = self.lem_rotation_offset_deg - np.degrees(yaw)

        # intentar usar scipy.ndimage.rotate para rotación arbitraria (nearest neighbor)
        rotated = None
        try:
            from scipy.ndimage import rotate
            # rotate devuelve una matriz flotante si input int; usamos order=0 (nearest), cval=UNKNOWN
            rotated = rotate(buffer, angle=angle_deg, reshape=False, order=0, mode='constant', cval=UNKNOWN)
        except Exception:
            # fallback: rotación cuantizada a 90° con np.rot90
            # aproximamos angle_deg a múltiplos de 90
            print("WOWOWOWOWOWO")
            k = int(np.round((angle_deg % 360) / 90.0)) % 4
            rotated = np.rot90(buffer, k=k)

        # center-crop la rotación a HxW
        rh, rw = rotated.shape
        start_r = (rh - H) // 2
        start_c = (rw - W) // 2
        # si la rotación produjo una matriz más pequeña (raro), recortar con límites seguros
        if start_r < 0 or start_c < 0:
            # crear canvas HxW con UNKNOWN y centrar rotated dentro
            canvas = np.full((H, W), UNKNOWN, dtype=int)
            rr = max(0, -start_r)
            cc = max(0, -start_c)
            rcopy0 = max(0, start_r)
            ccopy0 = max(0, start_c)
            rcopy1 = min(rh, start_r + H)
            ccopy1 = min(rw, start_c + W)
            dst_r1 = rr + (rcopy1 - rcopy0)
            dst_c1 = cc + (ccopy1 - ccopy0)
            canvas[rr:dst_r1, cc:dst_c1] = rotated[rcopy0:rcopy1, ccopy0:ccopy1]
            center_window = canvas
        else:
            center_window = rotated[start_r:start_r + H, start_c:start_c + W]

        # mapear valores a 0,128,255
        lem = self._map_values_to_lem(center_window)

        # marcar la posición del agente en el centro si se desea
        if self.lem_mark_agent and self.lem_agent_marker_size > 0:
            D = max(1, int(self.lem_agent_marker_size))
            if D % 2 == 0:
                D -= 1
            dhalf = D // 2
            cr = H // 2
            cc = W // 2
            r0m = max(0, cr - dhalf)
            c0m = max(0, cc - dhalf)
            r1m = min(H, cr + dhalf + 1)
            c1m = min(W, cc + dhalf + 1)
            lem[r0m:r1m, c0m:c1m] = np.clip(self.lem_agent_marker_value, 0, 255)

        self.lem_map = lem.copy()
        return self.lem_map.copy()



    
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

    def set_lem_size(self, H: int, W: int) -> None:
        """
        Fija el tamaño de la LEM en celdas H x W.
        """
        if H <= 0 or W <= 0:
            raise ValueError("H y W deben ser enteros positivos")
        self.lem_size = (int(H), int(W))
        self.lem_map = None

    def set_lem_size_from_range(self, range_m: float, use_radius: bool = True) -> None:
        """
        Calcula y fija la LEM a partir de un rango en metros.
        - range_m: si use_radius True se interpreta como radio R; lado = 2*R.
                   si use_radius False se interpreta como lado total.
        Requiere self.resolution definida.
        """
        if self.resolution is None:
            raise RuntimeError("resolution no definida en el MapProcessor")
        if range_m <= 0:
            raise ValueError("range_m debe ser positivo")

        side_m = 2.0 * range_m if use_radius else range_m
        cells = int(np.ceil(side_m / self.resolution))
        H = W = max(1, cells)
        self.set_lem_size(H, W)


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

    # Métodos de plotting interactivo actualizados
    def start_plot(self, figsize=(12, 4), title="Map / ProbExplored / LEM"):
        if getattr(self, "_plot_started", False):
            return
        try:
            plt.ion()
            self._fig, axs = plt.subplots(1, 3, figsize=figsize)
            self._axs = axs
            self._axs[0].set_title("Map (original)")
            self._axs[1].set_title("ProbExplored (reducido)")
            self._axs[2].set_title("LEM (egocentric)")
            self._robot_scatter = None
            self._prob_robot_scatter = None

            # colormap para el mapa original (unknown=gray, free=white, occupied=black)
            cmap = mcolors.ListedColormap(['gray', 'white', 'black'])
            bounds = [-1, 0.5, 50, 101]
            self._norm = mcolors.BoundaryNorm(bounds, cmap.N)
            self._cmap = cmap

            # placeholders y estado para las imágenes
            self._im_list = [None, None, None]
            self._prob_colorbar = None

            for ax in self._axs:
                ax.set_xlabel('col')
                ax.set_ylabel('row')

            self._plot_started = True
        except Exception:
            self._plot_started = False
            raise


    def update_plot(self, draw=True):
        if not getattr(self, "_plot_started", False):
            return

        with self._plot_lock:
            # ---------------- Panel 0: mapa original ----------------
            if self.map_data is None:
                if self._im_list[0] is not None:
                    self._im_list[0].set_data(np.zeros((1, 1)))
                if draw:
                    plt.pause(0.001)
                return

            disp = np.full(self.map_data.shape, 0, dtype=int)
            disp[self.map_data == FREE] = 1
            disp[self.map_data >= 50] = 2

            if self._im_list[0] is None:
                self._im_list[0] = self._axs[0].imshow(disp, cmap=self._cmap, norm=self._norm,
                                                    origin='lower', interpolation='nearest')
                self._axs[0].set_xlim(-0.5, self.width - 0.5)
                self._axs[0].set_ylim(-0.5, self.height - 0.5)
            else:
                self._im_list[0].set_data(disp)
                self._im_list[0].set_extent((-0.5, self.width - 0.5, -0.5, self.height - 0.5))

            # ---------------- Panel 1: prob_explored_map reducido (heatmap) ----------------
            try:
                prob_map = self.get_prob_explored_map_copy(as_uint8=False)
            except Exception:
                prob_map = None

            if prob_map is None:
                prob_img = np.zeros(self.prob_explored_bbox_size, dtype=np.float32)
            else:
                prob_img = prob_map.copy()

            Hp, Wp = prob_img.shape

            if self._im_list[1] is None:
                self._im_list[1] = self._axs[1].imshow(prob_img, cmap='viridis', origin='lower',
                                                    vmin=0.0, vmax=1.0, interpolation='nearest')
                self._axs[1].set_xlim(-0.5, Wp - 0.5)
                self._axs[1].set_ylim(-0.5, Hp - 0.5)
                self._prob_colorbar = self._fig.colorbar(self._im_list[1], ax=self._axs[1], fraction=0.046, pad=0.04)
                self._prob_colorbar.set_label("P(unknown)")
            else:
                self._im_list[1].set_data(prob_img)
                self._im_list[1].set_extent((-0.5, Wp - 0.5, -0.5, Hp - 0.5))
                try:
                    self._prob_colorbar.update_normal(self._im_list[1])
                except Exception:
                    pass

            # ---------------- Panel 2: LEM ----------------
            lem_img = self.lem_map if self.lem_map is not None else np.full(self.lem_size, 128, dtype=np.uint8)
            if self._im_list[2] is None:
                self._im_list[2] = self._axs[2].imshow(lem_img, cmap='gray', vmin=0, vmax=255, origin='lower', interpolation='nearest')
                self._axs[2].set_xlim(-0.5, self.lem_size[1] - 0.5)
                self._axs[2].set_ylim(-0.5, self.lem_size[0] - 0.5)
            else:
                self._im_list[2].set_data(lem_img)
                self._im_list[2].set_extent((-0.5, self.lem_size[1] - 0.5, -0.5, self.lem_size[0] - 0.5))

            # ---------------- Actualizar marcador del robot en panel 0 y panel 1 (MISMO BLOQUE) ----------------
            # 1) obtener celda del robot en mapa global
            robot_cell = None
            if self.robot_pose is not None:
                try:
                    r_global, c_global = self.get_robot_cell()   # debe devolver (row, col) en mapa global
                    robot_cell = (int(r_global), int(c_global))
                except Exception:
                    try:
                        x, y, yaw = self.robot_pose
                        c_global = int((x - self.map_origin_x) / self.resolution)
                        r_global = int((y - self.map_origin_y) / self.resolution)
                        robot_cell = (r_global, c_global)
                    except Exception:
                        robot_cell = None

            # 2) actualizar scatter en panel 0 (mapa global)
            if robot_cell is not None:
                row, col = robot_cell
                if getattr(self, "_robot_scatter", None) is None:
                    self._robot_scatter = self._axs[0].scatter([col], [row], c='red', s=50, marker='o', zorder=5)
                else:
                    try:
                        self._robot_scatter.set_offsets([[col, row]])
                    except Exception:
                        try:
                            self._robot_scatter.remove()
                        except Exception:
                            pass
                        self._robot_scatter = self._axs[0].scatter([col], [row], c='red', s=50, marker='o', zorder=5)
            else:
                if getattr(self, "_robot_scatter", None) is not None:
                    try:
                        self._robot_scatter.remove()
                    except Exception:
                        pass
                    self._robot_scatter = None

            # 3) mapear celda global -> celda reducida y actualizar scatter en panel 1
            if robot_cell is not None:
                r_g, c_g = robot_cell
                H_in, W_in = self.map_data.shape
                H_out, W_out = Hp, Wp
                scale_r = H_in / float(H_out)
                scale_c = W_in / float(W_out)
                i_red = int(np.clip(np.floor(r_g / scale_r), 0, H_out - 1))
                j_red = int(np.clip(np.floor(c_g / scale_c), 0, W_out - 1))
                scatter_x = j_red
                scatter_y = i_red
            else:
                scatter_x = Wp / 2.0
                scatter_y = Hp / 2.0

            if getattr(self, "_prob_robot_scatter", None) is None:
                self._prob_robot_scatter = self._axs[1].scatter([scatter_x], [scatter_y], c='red', s=40, marker='o', zorder=6)
                try:
                    self._prob_robot_scatter._is_robot_marker = True
                except Exception:
                    pass
            else:
                try:
                    self._prob_robot_scatter.set_offsets([[scatter_x, scatter_y]])
                except Exception:
                    try:
                        self._prob_robot_scatter.remove()
                    except Exception:
                        pass
                    self._prob_robot_scatter = self._axs[1].scatter([scatter_x], [scatter_y], c='red', s=40, marker='o', zorder=6)
                    try:
                        self._prob_robot_scatter._is_robot_marker = True
                    except Exception:
                        pass

            # ---------------- Finalizar frame ----------------
            if draw:
                plt.pause(0.001)




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
