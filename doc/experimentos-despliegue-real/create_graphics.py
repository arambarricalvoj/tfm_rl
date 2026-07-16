import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ==================================================
# CONFIGURACIÓN
# ==================================================
BASE_DIR = (
    "/home/javierac/Documents/ucm/TFM-PFM/"
    "experimentos-14-07-26-20260715T152136Z-1-001/"
    "experimentos-14-07-26"
)

OUTPUT_DIR = os.path.join(
    BASE_DIR,
    "graficas"
)

EXPERIMENTS = [
    f"exp-{i:02d}" for i in range(1, 11)
]

# Columna temporal del CSV
TIME_COLUMN = "Time_Elapsed_s"

# Tiempo máximo considerado
MAX_TIME = 40.01

# Número de puntos para interpolar antes de hacer la media
N_POINTS_MEAN = 500

# Variables a representar

VARIABLES = {

    "Explored_Percent": {
        "ylabel": "Porcentaje de exploración (%)",
        "title": "Exploración",
        "color": "tab:blue"
    },

    "Distance_Traveled_m": {
        "ylabel": "Distancia recorrida (m)",
        "title": "Movimiento",
        "color": "tab:orange"
    },

    "Min_Obstacle_Dist_m": {
        "ylabel": "Distancia mínima a obstáculo (m)",
        "title": "Seguridad",
        "color": "tab:green"
    },

    "Collisions": {
        "ylabel": "Cantidad de colisiones",
        "title": "Colisiones",
        "color": "tab:red"
    }

}

# ==================================================
# FUNCIONES
# ==================================================
def find_csv(folder):
    """
    Busca el CSV dentro de la carpeta del experimento.
    """
    files = glob.glob(
        os.path.join(folder, "*.csv")
    )
    if len(files) == 0:
        raise FileNotFoundError(
            f"No se encontró CSV en {folder}"
        )
    if len(files) > 1:
        print(
            f"Advertencia: varios CSV en {folder}. "
            f"Usando {files[0]}"
        )
    return files[0]

def limit_time(df):
    """
    Elimina muestras posteriores a MAX_TIME.
    """
    if TIME_COLUMN not in df.columns:
        raise KeyError(
            f"No existe la columna {TIME_COLUMN}"
        )
    return df[
        df[TIME_COLUMN] <= MAX_TIME
    ]

def plot_individual(df, exp_name, variable):
    """
    Genera una gráfica individual.
    """
    config = VARIABLES[variable]
    plt.figure(
        figsize=(8,4)
    )
    plt.plot(
        df[TIME_COLUMN],
        df[variable],
        color=config["color"],
        linewidth=2
    )
    plt.xlabel(
        "Tiempo (s)"
    )
    plt.ylabel(
        config["ylabel"]
    )
    plt.title(
        f"{config['title']} - {exp_name}"
    )
    plt.grid(True)
    plt.tight_layout()

    save_path = os.path.join(
        OUTPUT_DIR,
        exp_name,
        f"{variable}.png"
    )

    plt.savefig(
        save_path,
        dpi=300,
        bbox_inches="tight"
    )

    plt.close()

def plot_summary(df, exp_name):
    """
    Genera la gráfica conjunta 2x2.
    """
    fig, axes = plt.subplots(
        2,
        2,
        figsize=(12,8)
    )
    plots = [

        ("Explored_Percent", axes[0,0]),

        ("Distance_Traveled_m", axes[1,0]),

        ("Min_Obstacle_Dist_m", axes[0,1]),

        ("Collisions", axes[1,1])

    ]

    for variable, ax in plots:
        config = VARIABLES[variable]
        ax.plot(
            df[TIME_COLUMN],
            df[variable],
            color=config["color"],
            linewidth=2
        )
        ax.set_title(
            config["title"]
        )
        ax.set_xlabel(
            "Tiempo (s)"
        )
        ax.set_ylabel(
            config["ylabel"]
        )
        ax.grid(True)

    fig.suptitle(
        f"Experiment {exp_name}",
        fontsize=16
    )

    plt.tight_layout()
    save_path = os.path.join(
        OUTPUT_DIR,
        exp_name,
        "summary.png"
    )
    plt.savefig(
        save_path,
        dpi=300,
        bbox_inches="tight"
    )
    plt.close()

def compute_mean_experiments(dataframes):
    """
    Calcula la media y desviación estándar temporal de los experimentos.
    Todos se interpolan sobre el mismo eje temporal.
    """
    time_common = np.linspace(0, MAX_TIME, N_POINTS_MEAN)

    mean_data = {TIME_COLUMN: time_common}
    std_data = {TIME_COLUMN: time_common}

    for variable in VARIABLES:
        interpolated = []

        for df in dataframes:
            values = np.interp(
                time_common,
                df[TIME_COLUMN],
                df[variable]
            )
            interpolated.append(values)

        interpolated = np.array(interpolated)

        mean_data[variable] = np.mean(
            interpolated,
            axis=0
        )

        std_data[variable] = np.std(
            interpolated,
            axis=0
        )

    return (
        pd.DataFrame(mean_data),
        pd.DataFrame(std_data)
    )



def plot_mean(df_mean, df_std):
    """
    Genera gráficas individuales de la media con desviación estándar.
    """
    for variable in VARIABLES:

        config = VARIABLES[variable]

        plt.figure(figsize=(8,4))

        plt.plot(
            df_mean[TIME_COLUMN],
            df_mean[variable],
            color=config["color"],
            linewidth=3,
            label="Media"
        )

        plt.fill_between(
            df_mean[TIME_COLUMN],
            df_mean[variable] - df_std[variable],
            df_mean[variable] + df_std[variable],
            color=config["color"],
            alpha=0.2,
            label="Desviación estándar"
        )

        plt.xlabel(
            "Tiempo (s)"
        )

        plt.ylabel(
            config["ylabel"]
        )

        plt.title(
            f"{config['title']} - Media de 10 experimentos"
        )

        plt.legend()
        plt.grid(True)
        plt.tight_layout()

        save_path = os.path.join(
            OUTPUT_DIR,
            f"mean_{variable}.png"
        )

        plt.savefig(
            save_path,
            dpi=300,
            bbox_inches="tight"
        )

        plt.close()


def plot_mean_summary(df_mean, df_std):
    """
    Genera resumen 2x2 de las medias con desviación estándar.
    """

    fig, axes = plt.subplots(
        2,
        2,
        figsize=(12,8)
    )

    plots = [
        ("Explored_Percent", axes[0,0]),
        ("Distance_Traveled_m", axes[1,0]),
        ("Min_Obstacle_Dist_m", axes[0,1]),
        ("Collisions", axes[1,1])
    ]

    for variable, ax in plots:

        config = VARIABLES[variable]

        ax.plot(
            df_mean[TIME_COLUMN],
            df_mean[variable],
            color=config["color"],
            linewidth=3
        )

        ax.fill_between(
            df_mean[TIME_COLUMN],
            df_mean[variable] - df_std[variable],
            df_mean[variable] + df_std[variable],
            color=config["color"],
            alpha=0.2
        )

        ax.set_title(
            config["title"]
        )

        ax.set_xlabel(
            "Tiempo (s)"
        )

        ax.set_ylabel(
            config["ylabel"]
        )

        ax.grid(True)

    fig.suptitle(
        "Rendimiento medio en 10 experimentos",
        fontsize=16
    )

    plt.tight_layout()

    save_path = os.path.join(
        OUTPUT_DIR,
        "mean_summary_std.png"
    )

    plt.savefig(
        save_path,
        dpi=300,
        bbox_inches="tight"
    )

    plt.close()



# ==================================================
# MAIN
# ==================================================


def main():


    os.makedirs(
        OUTPUT_DIR,
        exist_ok=True
    )


    all_dataframes = []



    for exp in EXPERIMENTS:


        print(
            f"Procesando {exp}..."
        )


        exp_path = os.path.join(
            BASE_DIR,
            exp
        )


        csv_file = find_csv(
            exp_path
        )


        df = pd.read_csv(
            csv_file
        )


        df = limit_time(
            df
        )


        all_dataframes.append(
            df
        )



        output_exp = os.path.join(
            OUTPUT_DIR,
            exp
        )


        os.makedirs(
            output_exp,
            exist_ok=True
        )



        for variable in VARIABLES:


            if variable not in df.columns:

                print(
                    f"{variable} no encontrada en {csv_file}"
                )

                continue



            plot_individual(
                df,
                exp,
                variable
            )



        plot_summary(
            df,
            exp
        )



    print(
        "Calculando medias..."
    )


    df_mean, df_std = compute_mean_experiments(
        all_dataframes
    )

    plot_mean(
        df_mean,
        df_std
    )

    plot_mean_summary(
        df_mean,
        df_std
    )


    print(
        "Proceso terminado correctamente."
    )



if __name__ == "__main__":

    main()