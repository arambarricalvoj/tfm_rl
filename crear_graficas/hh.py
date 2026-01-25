"""import pickle

file = "ddpg_0_stage_9/stage9_episode1000.pkl"   # cámbialo si quieres
with open(file, "rb") as f:
    data = pickle.load(f)

print("Tipo de data:", type(data))
print("Longitud:", len(data))

print("\nTipos de los elementos:")
for i, elem in enumerate(data):
    print(f"  [{i}] -> {type(elem)}")
    if i >= 5:
        break

print("\nContenido del primer elemento:")
print(data[0])
"""

import pickle

file = "ddpg_0_stage_9/stage9_episode1000.pkl"
with open(file, "rb") as f:
    data = pickle.load(f)

print("Elemento 0 (total steps):", data[0])

for i in range(1, 5):
    print(f"\nElemento {i}: tipo={type(data[i])}, longitud={len(data[i])}")
    print("  primeros 10 valores:", data[i][:10])
