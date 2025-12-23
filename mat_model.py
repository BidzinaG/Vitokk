import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import math
import os

# ЧИТАЕМ ЛОГ ИЗ KSP
BASE_DIR = os.path.dirname(os.path.abspath("sample.ipynb"))
CSV_FILE = os.path.join(BASE_DIR, "flight_log.csv")

df = pd.read_csv(CSV_FILE).sort_values("time").reset_index(drop=True)

t_ksp = df["time"].to_numpy()        # время из лога
h_ksp = df["altitude"].to_numpy()    # высота из лога
v_ksp = df["speed"].to_numpy()       # скорость из лога
m_ksp = df["mass"].to_numpy()        # масса

# 2) КОНСТАНТЫ ДЛЯ KERBIN
mu = 3.5316e12     # гравитационный параметр Kerbin (м^3/с^2)
R  = 600_000.0     # радиус Kerbin (м)
g0 = 9.8       # стандартная g (для Isp) (м/с^2)

# атмосфера
rho0 = 1.225       # плотность у земли
H = 5600.0         # масштаб высоты

# аэродинамика
C_d = 0.25
S = math.pi * 4.0  # площадь


# ОПИСАНИЕ СТУПЕНЕЙ

# (name, m_start, m_end, thrust, Isp, burn_time)
# m_start/m_end в кг, thrust в Н, Isp в сек, burn_time в сек
stages = [
    ("Stage1", 86.3e3, 44.3e3, 1.5818e6, 284.0, 284.0),
    ("Stage2", 32.0e3, 12.0e3, 3.626e5,  283.0, 283.0),
]

# ПРОГРАММА ТАНГАЖА

def pitch_func(h):
    # h — высота над уровнем моря
    # тут делаем простой разворот: 90° (вертикально) -> 0° (в горизонт) к 45 км
    if h <= 250.0:
        return math.radians(90.0)
    elif h <= 45000.0:
        k = (h - 250.0) / (45000.0 - 250.0)
        pitch_deg = 90.0 - 90.0 * k
        return math.radians(pitch_deg)
    else:
        return math.radians(0.0)

# ФУНКЦИИ ДЛЯ АТМОСФЕРЫ И ГРАВИТАЦИИ
def rho_atm(h):
    # плотность воздуха по экспоненте
    if h < 0:
        h = 0.0
    return rho0 * math.exp(-h / H)

def gravity_accel(x, y):
    # гравитация как центральное поле: a = -mu * r_vec / r^3
    r = math.sqrt(x*x + y*y)
    g = mu / (r*r)
    return (-g * x / r, -g * y / r)

# ОДУ ДЛЯ ОДНОЙ СТУПЕНИ
def make_ode(thrust, isp):
    # делаем функцию правых частей для solve_ivp
    def ode(t, state):
        # state = [x, y, vx, vy, m]
        x, y, vx, vy, m = state

        # радиус от центра планеты и высота
        r = math.sqrt(x*x + y*y) + 1e-9
        h = r - R

        # скорость по модулю
        v = math.sqrt(vx*vx + vy*vy) + 1e-9

        # угол тангажа по программе
        pitch = pitch_func(h)

        # ТЯГА: раскладываем по осям (x — горизонт, y — вверх)
        Tx = thrust * math.cos(pitch)
        Ty = thrust * math.sin(pitch)

        # СОПРОТИВЛЕНИЕ: D = 0.5 * Cd * rho * S * v^2, направление против скорости
        rh = rho_atm(h) if h <= 70_000 else 0.0  # выше 70 км воздух считаем нулем
        D = 0.5 * C_d * rh * S * v*v
        Dx = -D * (vx / v)
        Dy = -D * (vy / v)

        # ГРАВИТАЦИЯ (ускорение)
        ax_g, ay_g = gravity_accel(x, y)

        # ИТОГО УСКОРЕНИЯ: (T + D)/m + g
        ax = (Tx + Dx) / m + ax_g
        ay = (Ty + Dy) / m + ay_g

        # РАСХОД МАССЫ: dm/dt = -T / (Isp * g0)
        dm = -thrust / (isp * g0)

        # возвращаем производные: [dx/dt, dy/dt, dvx/dt, dvy/dt, dm/dt]
        return [vx, vy, ax, ay, dm]

    return ode

# событие: остановка когда масса дошла до m_end
def make_event_mass_end(m_end):
    def ev(t, state):
        return state[4] - m_end
    ev.terminal = True
    ev.direction = -1
    return ev

# событие: остановка по времени
def make_event_time_end(t_end):
    def ev(t, state):
        return t_end - t
    ev.terminal = True
    ev.direction = -1
    return ev

# ПОСЛЕДОВАТЕЛЬНО ИНТЕГРИРУЕМ СТУПЕНИ
# старт: на поверхности (x=0, y=R), скорость 0
x0, y0 = 0.0, R
vx0, vy0 = 0.0, 0.0

t0 = 0.0

# моделируем до конца лога или до конца всех ступеней
tf = min(float(t_ksp[-1]), sum(s[5] for s in stages))

# сюда будем собирать общий результат
T_all, X_all, Y_all, VX_all, VY_all, M_all = [], [], [], [], [], []

# текущее состояние для старта первой ступени
x, y, vx, vy = x0, y0, vx0, vy0

for idx, (name, m_start, m_end, thrust, isp, burn_time) in enumerate(stages):
    if t0 >= tf:
        break

    # масса на старте этой ступени
    m = m_start

    ode = make_ode(thrust=thrust, isp=isp)

    # конец времени для этого сегмента
    t_end = min(t0 + burn_time, tf)

    # интегрируем ОДУ на участке (t0 -> t_end)
    sol = solve_ivp(
        ode,
        (t0, t_end),
        [x, y, vx, vy, m],
        method="RK45",
        max_step=0.1,
        events=[make_event_mass_end(m_end), make_event_time_end(t_end)],
        rtol=1e-7,
        atol=1e-9
    )

    # добавляем точки в общий массив (чтобы не дублировать первую точку сегмента)
    sl = slice(None) if len(T_all) == 0 else slice(1, None)
    T_all.extend(sol.t[sl])
    X_all.extend(sol.y[0][sl])
    Y_all.extend(sol.y[1][sl])
    VX_all.extend(sol.y[2][sl])
    VY_all.extend(sol.y[3][sl])
    M_all.extend(sol.y[4][sl])

    # обновляем стартовые значения для следующей ступени
    x, y, vx, vy, m = sol.y[0][-1], sol.y[1][-1], sol.y[2][-1], sol.y[3][-1], sol.y[4][-1]
    t0 = float(sol.t[-1])


# переводим в numpy массивы
t_mod = np.array(T_all)
x_mod = np.array(X_all)
y_mod = np.array(Y_all)
vx_mod = np.array(VX_all)
vy_mod = np.array(VY_all)
m_mod = np.array(M_all)

# высота и скорость модели (для сравнения с KSP)
h_mod = np.sqrt(x_mod**2 + y_mod**2) - R
v_mod = np.sqrt(vx_mod**2 + vy_mod**2)

# СРАВНЕНИЕ ГРАФИКОВ (KSP vs МОДЕЛЬ)
mask_ksp = t_ksp <= t_mod[-1] if len(t_mod) else (t_ksp <= tf)

plt.figure(figsize=(15, 4))

# высота
plt.subplot(1, 3, 1)
plt.plot(t_ksp[mask_ksp], h_ksp[mask_ksp], label="KSP", linewidth=2)
plt.plot(t_mod, h_mod, label="Модель", linestyle="--", linewidth=2)
plt.xlabel("Время, с")
plt.ylabel("Высота, м")
plt.grid(True, alpha=0.3)
plt.legend()

# скорость
plt.subplot(1, 3, 2)
plt.plot(t_ksp[mask_ksp], v_ksp[mask_ksp], label="KSP", linewidth=2)
plt.plot(t_mod, v_mod, label="Модель", linestyle="--", linewidth=2)
plt.xlabel("Время, с")
plt.ylabel("Скорость, м/с")
plt.grid(True, alpha=0.3)
plt.legend()

# масса
plt.subplot(1, 3, 3)
if m_ksp is not None:
    plt.plot(t_ksp[mask_ksp], m_ksp[mask_ksp], label="KSP", linewidth=2)
plt.plot(t_mod, m_mod, label="Модель", linestyle="--", linewidth=2)
plt.xlabel("Время, с")
plt.ylabel("Масса, кг")
plt.grid(True, alpha=0.3)
plt.legend()

plt.tight_layout()
plt.show()
