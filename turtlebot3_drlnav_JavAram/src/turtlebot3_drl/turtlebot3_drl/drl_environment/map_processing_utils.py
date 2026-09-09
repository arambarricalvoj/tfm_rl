# map_processing_utils.py
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from typing import Optional, Tuple, Sequence

UNKNOWN = -1
FREE = 0
OCCUPIED = 100  # valor exacto de SLAM Toolbox


class MapProcessor:
    """
    Procesa un OccupancyGrid de SLAM Toolbox y genera:
    - métricas globales
    - LEM (Local Egocentric Map) 24x24 con escala configurable
    - GEM (Global Exploration Map) 24x24 con marcador configurable
    - cnn_map: stack [LEM, GEM] → (2,24,24)
    """

    def __init__(self):
        # mapa y metadatos
        self.map_data = None
        self.width = None
        self.height = None
        self.resolution = None
        self.origin = None
        self.robot_pose = (0.0, 0.0, 0.0)

        # métricas
        self.free_count = 0
        self.occupied_count = 0
        self.unknown_count = 0
        self.total_count = 0

        self.free_percent = 0.0
        self.occupied_percent = 0.0
        self.unknown_percent = 0.0
        self.free_ratio_known = 0.0
        self.coverage = 0.0
        self.mean_occupancy = 0.0
        self.size_m2 = None

        # métricas GEM
        self.total_count_gem = 0
        self.known_count_gem = 0
        self.unknown_count_gem = 0

        self.known_percent_gem = 0.0
        self.unknown_percent_gem = 0.0
        self.coverage_gem = 0.0
        self.mean_occupancy_gem = 0.0

        self.explored_bbox_gem = None
        self.explored_bbox_size_gem = None

        # bounding box explorado
        self.explored_bbox = None
        self.explored_bbox_size = None

        # mapas derivados
        self.lem_size = (24, 24)
        self.gem_size = (24, 24)
        self.lem_scale = 4  # configurable externamente
        self.lem_map = None
        self.gem_map = None
        self.cnn_map = None

        # parámetros GEM
        self.gem_agent_marker_size = 3
        self.gem_agent_marker_value = 128

        # flags
        self.enable_lem = True
        self.enable_global_reduced_map = True

        # plotting
        self._fig = None
        self._axs = None
        self._im_list = [None, None, None]

        ### NUEVO ###
        self.robot_path = []   # lista de (row, col)
        self.start_cell = None
        self.goal_cell = None
        self.final_cell = None
        self._path_artist = None
        self._start_artist = None
        self._final_artist = None
        ### FIN NUEVO ###

    # ------------------------------------------------------------------
    #   ACTUALIZACIÓN DESDE ROS
    # ------------------------------------------------------------------
    def update_from_occupancy_grid(self, msg):
        data = msg.data
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        try:
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y
            q = msg.info.origin.orientation
            yaw = self._quat_to_yaw(q)
            origin_pose = (ox, oy, yaw)
        except:
            origin_pose = None

        self.update_from_data(data, width, height, resolution, origin_pose)

    def update_from_data(self, data, width, height, resolution, origin=None):
        arr = np.array(data, dtype=int)
        if arr.size != width * height:
            raise ValueError("Tamaño incorrecto")

        self.map_data = arr.reshape((height, width))
        self.width = width
        self.height = height
        self.resolution = float(resolution)
        self.origin = origin

        self._compute_stats()

        if self.enable_lem:
            self.generate_lem()

        if self.enable_global_reduced_map:
            self.compute_gem()

        self._update_cnn_map()

    # ------------------------------------------------------------------
    #   MÉTRICAS
    # ------------------------------------------------------------------
    def _compute_stats(self):
        flat = self.map_data.ravel()

        self.total_count = flat.size
        self.free_count = int((flat == FREE).sum())
        self.occupied_count = int((flat == OCCUPIED).sum())
        self.unknown_count = int((flat == UNKNOWN).sum())

        self.free_percent = 100 * self.free_count / self.total_count
        self.occupied_percent = 100 * self.occupied_count / self.total_count
        self.unknown_percent = 100 * self.unknown_count / self.total_count

        known = self.free_count + self.occupied_count
        self.free_ratio_known = self.free_count / known if known > 0 else 0.0
        self.coverage = self.free_ratio_known

        known_vals = flat[flat != UNKNOWN]
        self.mean_occupancy = float(known_vals.mean()) if known_vals.size > 0 else 0.0

        self.size_m2 = self.width * self.height * (self.resolution ** 2)

        # bbox explorado
        known_idx = np.argwhere(self.map_data != UNKNOWN)
        if known_idx.size == 0:
            self.explored_bbox = None
            self.explored_bbox_size = None
        else:
            rows = known_idx[:, 0]
            cols = known_idx[:, 1]
            min_r, min_c = int(rows.min()), int(cols.min())
            max_r, max_c = int(rows.max()), int(cols.max())
            self.explored_bbox = (min_r, min_c, max_r, max_c)
            self.explored_bbox_size = (max_r - min_r + 1, max_c - min_c + 1)
    
    def _compute_gem_stats(self):
        if self.gem_map is None:
            return

        # total de celdas
        self.total_count_gem = self.gem_map.size

        # desconocido = 0
        self.unknown_count_gem = int((self.gem_map == 0).sum())

        # conocido = 255 (explorado) + 128 (robot)
        known_mask = (self.gem_map == 255) | (self.gem_map == self.gem_agent_marker_value)
        self.known_count_gem = int(known_mask.sum())

        # porcentajes
        if self.total_count_gem > 0:
            self.known_percent_gem = 100.0 * self.known_count_gem / self.total_count_gem
            self.unknown_percent_gem = 100.0 * self.unknown_count_gem / self.total_count_gem
        else:
            self.known_percent_gem = 0.0
            self.unknown_percent_gem = 0.0

        # coverage = ratio de celdas conocidas
        self.coverage_gem = self.known_count_gem / self.total_count_gem if self.total_count_gem > 0 else 0.0

    # ------------------------------------------------------------------
    #   LEM (Local Egocentric Map) con escala configurable
    # ------------------------------------------------------------------
    def generate_lem(self):
        if self.map_data is None or self.robot_pose is None:
            self.lem_map = None
            return None

        H, W = self.lem_size
        scale = float(self.lem_scale)

        rc = self.get_robot_cell()
        if rc is None:
            self.lem_map = None
            return None

        center_r, center_c = rc

        # tamaño real del recorte (redondeado a entero)
        H_big = int(round(H * scale))
        W_big = int(round(W * scale))

        # recorte centrado EXACTO
        r0 = center_r - (H_big // 2)
        c0 = center_c - (W_big // 2)
        r1 = r0 + H_big - 1
        c1 = c0 + W_big - 1

        # clip
        r0c = max(0, r0)
        c0c = max(0, c0)
        r1c = min(self.height - 1, r1)
        c1c = min(self.width - 1, c1)

        # canvas grande
        canvas_big = np.full((H_big, W_big), UNKNOWN, dtype=int)
        sub = self.map_data[r0c:r1c + 1, c0c:c1c + 1]

        rr = r0c - r0
        cc = c0c - c0
        canvas_big[rr:rr + sub.shape[0], cc:cc + sub.shape[1]] = sub

        # downsample NN a 24×24
        lem = np.zeros((H, W), dtype=np.uint8)

        for i in range(H):
            for j in range(W):
                r0 = int(i * H_big / H)
                r1 = int((i+1) * H_big / H)
                c0 = int(j * W_big / W)
                c1 = int((j+1) * W_big / W)

                block = canvas_big[r0:r1, c0:c1]

                # max pooling real
                v = block.max()

                if v == OCCUPIED:
                    lem[i, j] = 255
                elif v == FREE:
                    lem[i, j] = 0
                else:
                    lem[i, j] = 128

        self.lem_map = lem
        return lem


    # ------------------------------------------------------------------
    #   GEM (Global Exploration Map) 24×24
    # ------------------------------------------------------------------
    def compute_gem(self):
        if self.map_data is None or self.explored_bbox is None:
            self.gem_map = None
            return None

        min_r, min_c, max_r, max_c = self.explored_bbox
        sub = self.map_data[min_r:max_r + 1, min_c:max_c + 1]

        # GEM base: conocido → 255, desconocido → 0
        gem_src = np.zeros_like(sub, dtype=np.uint8)
        gem_src[sub != UNKNOWN] = 255
        gem_src[sub == UNKNOWN] = 0

        H_in, W_in = gem_src.shape
        H_out, W_out = self.gem_size

        gem = np.zeros((H_out, W_out), dtype=np.uint8)

        # nearest neighbor
        for i in range(H_out):
            src_r = int((i + 0.5) * H_in / H_out)
            src_r = min(max(src_r, 0), H_in - 1)
            for j in range(W_out):
                src_c = int((j + 0.5) * W_in / W_out)
                src_c = min(max(src_c, 0), W_in - 1)
                gem[i, j] = gem_src[src_r, src_c]

        # marcar robot
        rc = self.get_robot_cell()
        if rc is not None:
            r_g, c_g = rc
            r_rel = r_g - min_r
            c_rel = c_g - min_c
            r_rel = min(max(r_rel, 0), H_in - 1)
            c_rel = min(max(c_rel, 0), W_in - 1)

            # mapeo input→output
            i_red = int(round((r_rel + 0.5) * H_out / H_in - 0.5))
            j_red = int(round((c_rel + 0.5) * W_out / W_in - 0.5))
            i_red = min(max(i_red, 0), H_out - 1)
            j_red = min(max(j_red, 0), W_out - 1)

            D = max(1, int(self.gem_agent_marker_size))
            if D % 2 == 0:
                D -= 1
            d = D // 2

            r0 = max(0, i_red - d)
            r1 = min(H_out - 1, i_red + d)
            c0 = max(0, j_red - d)
            c1 = min(W_out - 1, j_red + d)

            gem[r0:r1 + 1, c0:c1 + 1] = self.gem_agent_marker_value

        self.gem_map = gem
        self._compute_gem_stats()
        return gem

    # ------------------------------------------------------------------
    #   CNN MAP
    # ------------------------------------------------------------------
    def _update_cnn_map(self):
        if self.lem_map is None or self.gem_map is None:
            self.cnn_map = None
            return
        self.cnn_map = np.stack([self.lem_map, self.gem_map], axis=0)

    # ------------------------------------------------------------------
    #   POSE Y CELDAS
    # ------------------------------------------------------------------
    def set_robot_pose_from_pose_msg(self, pose_msg):
        if hasattr(pose_msg, 'pose') and hasattr(pose_msg.pose, 'pose'):
            p = pose_msg.pose.pose
        else:
            p = pose_msg.pose

        pos = p.position
        ori = p.orientation
        yaw = self._quat_to_yaw(ori)
        self.robot_pose = (pos.x, pos.y, yaw)

        ### NUEVO: registrar trayectoria ###
        rc = self.get_robot_cell()
        if rc is not None:
            if self.start_cell is None:
                self.start_cell = rc  # primera posición
            self.robot_path.append(rc)
        ### FIN NUEVO ###

    def get_robot_cell(self):
        if self.map_data is None or self.origin is None:
            return None

        x, y, _ = self.robot_pose
        ox, oy, _ = self.origin

        col = int((x - ox) / self.resolution)
        row = int((y - oy) / self.resolution)

        if 0 <= row < self.height and 0 <= col < self.width:
            return (row, col)
        return None

    def get_robot_cell_value(self):
        rc = self.get_robot_cell()
        if rc is None:
            return None
        r, c = rc
        return int(self.map_data[r, c])

    # ------------------------------------------------------------------
    #   VISUALIZACIÓN
    # ------------------------------------------------------------------
    def start_plot(self):
        if self._fig is not None:
            return

        self._fig, self._axs = plt.subplots(1, 3, figsize=(12, 4))
        titles = ["Global Map", "LEM", "GEM"]
        for ax, t in zip(self._axs, titles):
            ax.set_title(t)
            ax.axis('off')

        self._im_list = [None, None, None]
        plt.ion()
        plt.show()

    def update_plot(self):
        if self._fig is None or self.map_data is None:
            return

        # panel 0
        ax0 = self._axs[0]
        if self._im_list[0] is None:
            cmap, norm = self._get_global_cmap()
            self._im_list[0] = ax0.imshow(self.map_data, cmap=cmap, norm=norm, origin='lower')
        else:
            self._im_list[0].set_data(self.map_data)

        ### NUEVO: trayectoria + inicio + fin ###
        if self._path_artist is not None:
            self._path_artist.remove()
            self._path_artist = None

        if self._start_artist is not None:
            self._start_artist.remove()
            self._start_artist = None

        if self._final_artist is not None:
            self._final_artist.remove()
            self._final_artist = None

        ### NUEVO: dibujar trayectoria ###
        if len(self.robot_path) > 1:
            rows = [p[0] for p in self.robot_path]
            cols = [p[1] for p in self.robot_path]
            self._path_artist, = ax0.plot(cols, rows, color='red', linewidth=2)

        ### NUEVO: inicio ###
        if self.start_cell is not None:
            self._start_artist = ax0.scatter(self.start_cell[1], self.start_cell[0], c='blue', s=50)

        ### NUEVO: final ###
        if self.final_cell is not None:
            self._final_artist = ax0.scatter(self.final_cell[1], self.final_cell[0], c='yellow', s=50)
        ### FIN NUEVO ###

        # panel 1
        lem = self.lem_map
        if lem is not None:
            ax1 = self._axs[1]
            if self._im_list[1] is None:
                cmap_lem = mcolors.ListedColormap(['black', 'gray', 'white'])
                bounds = [0, 1, 129, 256]
                norm_lem = mcolors.BoundaryNorm(bounds, cmap_lem.N)
                self._im_list[1] = ax1.imshow(lem, cmap=cmap_lem, norm=norm_lem, origin='lower')
            else:
                self._im_list[1].set_data(lem)

        # panel 2
        gem = self.gem_map
        if gem is not None:
            ax2 = self._axs[2]
            if self._im_list[2] is None:
                cmap_gem = mcolors.ListedColormap(['black', 'gray', 'white'])
                bounds = [0, 1, 129, 256]
                norm_gem = mcolors.BoundaryNorm(bounds, cmap_gem.N)
                self._im_list[2] = ax2.imshow(gem, cmap=cmap_gem, norm=norm_gem, origin='lower')
            else:
                self._im_list[2].set_data(gem)

        self._fig.canvas.draw()
        self._fig.canvas.flush_events()

    def _get_global_cmap(self):
        cmap = mcolors.ListedColormap(['gray', 'white', 'black'])
        bounds = [-1.5, -0.5, 0.5, 150]
        norm = mcolors.BoundaryNorm(bounds, cmap.N)
        return cmap, norm

    # ------------------------------------------------------------------
    #   NUEVA GUI INDEPENDIENTE: MAPA + TRAYECTORIA
    # ------------------------------------------------------------------
    def show_path_window(self):
        if self.map_data is None:
            return

        fig, ax = plt.subplots(figsize=(6, 6))
        cmap, norm = self._get_global_cmap()
        ax.imshow(self.map_data, cmap=cmap, norm=norm, origin='lower')

        # trayectoria en rojo
        if len(self.robot_path) > 1:
            rows = [p[0] for p in self.robot_path]
            cols = [p[1] for p in self.robot_path]
            ax.plot(cols, rows, color='red', linewidth=2)

        # inicio en azul
        if self.start_cell is not None:
            ax.scatter(self.start_cell[1], self.start_cell[0], c='blue', s=60, label='Inicio')

        # fin en verde
        #if self.robot_path:
        #    last = self.robot_path[-1]
        #    ax.scatter(last[1], last[0], c='green', s=60, label='Fin')
        if self.final_cell is not None:
            ax0.scatter(self.final_cell[1], self.final_cell[0], c='green', s=50)


        ax.set_title("Trayectoria del robot sobre el mapa global")
        ax.axis('off')
        ax.legend(loc='upper right')
        plt.show()

    def set_final_cell(self):
        """Marca la última celda de la trayectoria como punto final."""
        if self.robot_path:
            self.final_cell = self.robot_path[-1]

    def reset_path(self):
        """Resetea trayectoria, inicio y fin."""
        self.robot_path = []
        self.start_cell = None
        self.final_cell = None

    def save_map_png(self, filename):
        """
        Guarda el mapa global con trayectoria en un archivo PNG.
        Si el archivo ya existe, añade un sufijo incremental automáticamente.
        """
        if self.map_data is None:
            return

        # --- generar nombre alternativo si ya existe ---
        base, ext = os.path.splitext(filename)
        final_name = filename
        counter = 1

        while os.path.exists(final_name):
            final_name = f"{base}_{counter}{ext}"
            counter += 1

        # --- generar figura ---
        fig, ax = plt.subplots(figsize=(6, 6))
        cmap, norm = self._get_global_cmap()
        ax.imshow(self.map_data, cmap=cmap, norm=norm, origin='lower')

        # trayectoria
        if len(self.robot_path) > 1:
            rows = [p[0] for p in self.robot_path]
            cols = [p[1] for p in self.robot_path]
            ax.plot(cols, rows, color='red', linewidth=2)

        # inicio
        if self.start_cell is not None:
            ax.scatter(self.start_cell[1], self.start_cell[0], c='blue', s=60)

        # final (solo si tú lo marcas)
        if self.final_cell is not None:
            ax.scatter(self.final_cell[1], self.final_cell[0], c='yellow', s=60)

        ax.axis('off')
        fig.savefig(final_name, dpi=200, bbox_inches='tight')
        plt.close(fig)

        print(f"Mapa guardado en: {final_name}")


    # ------------------------------------------------------------------
    #   UTILIDAD: CUATERNION → YAW
    # ------------------------------------------------------------------
    def _quat_to_yaw(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return float(np.arctan2(siny, cosy))
