"""Prepara los datos de las figuras de la presentación a partir de los CSV del barrido.

Uso (desde la raíz del repo):
    python docs/presentacion/preparar_datos.py

Lee las curvas exportadas por la página /curvas y escribe en docs/presentacion/datos/:
  aire.dat  rpm ibus rpmstd duty            (tara: vaso sin agua)
  agua.dat  rpm ibus rpmstd duty dI dT acoplado
dI = (I_bus - I_bus,tara(n)) / d  [mA]: corriente de carga contra la tara a igual velocidad.
dT = K_t * dI                     [mN m]
La tara se interpola linealmente y no se extrapola. Se descarta el primer escalón de
cada barrido porque todavía es el transitorio del arranque.
"""
import csv
import math
from pathlib import Path

RAIZ = Path(__file__).resolve().parents[2]
CURVAS = RAIZ / 'simulacion_agitador' / 'curvas_experimentales'
AIRE = CURVAS / 'curva-escalera-20261126-131213.csv'
AGUA = CURVAS / 'curva-escalera-20261126-132235.csv'
SALIDA = Path(__file__).resolve().parent / 'datos'


def leer(path):
    with open(path, encoding='utf-8') as f:
        filas = [linea for linea in f if not linea.startswith('#')]
    return [{k: float(v) for k, v in fila.items()} for fila in csv.DictReader(filas)][1:]


def interpolar(x, xs, ys):
    for i in range(len(xs) - 1):
        if xs[i] <= x <= xs[i + 1]:
            t = (x - xs[i]) / (xs[i + 1] - xs[i])
            return ys[i] + t * (ys[i + 1] - ys[i])
    return None  # fuera del rango de la tara: no se extrapola


def recta(xs, ys):
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
    return my - b * mx, b


aire = leer(AIRE)
agua = leer(AGUA)

# K = K_e = K_t sale de la pendiente de d*V_bus contra la velocidad en la tara.
_, pendiente = recta([r['rpm_mean'] for r in aire], [r['duty_pct'] / 100 * r['bus_v'] for r in aire])
kt = pendiente * 60 / (2 * math.pi)  # V/rpm -> V s/rad = N m/A

SALIDA.mkdir(exist_ok=True)
with open(SALIDA / 'aire.dat', 'w', encoding='utf-8') as f:
    f.write('rpm ibus rpmstd duty\n')
    for r in aire:
        f.write(f"{r['rpm_mean']:.0f} {r['bus_ma']:.1f} {r['rpm_std']:.0f} {r['duty_pct']:.2f}\n")

xs = [r['rpm_mean'] for r in aire]
ys = [r['bus_ma'] for r in aire]
with open(SALIDA / 'agua.dat', 'w', encoding='utf-8') as f:
    f.write('rpm ibus rpmstd duty dI dT acoplado\n')
    for r in agua:
        tara = interpolar(r['rpm_mean'], xs, ys)
        if tara is None:
            di = dt = 'nan'
        else:
            valor = (r['bus_ma'] - tara) / (r['duty_pct'] / 100)
            di, dt = f'{valor:.1f}', f'{kt * valor:.3f}'
        acoplado = 1 if r['rpm_std'] < 15 else 0
        f.write(f"{r['rpm_mean']:.0f} {r['bus_ma']:.1f} {r['rpm_std']:.0f} {r['duty_pct']:.2f} {di} {dt} {acoplado}\n")

print(f'K_t = {kt:.4f} N m/A; datos escritos en {SALIDA}')
