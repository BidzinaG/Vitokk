import krpc
import time
import csv
import matplotlib.pyplot as plt
import os
import pandas as pd

# Соединение
conn = krpc.connect(name='Mission')
vessel = conn.space_center.active_vessel
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(BASE_DIR, "flight_log.csv")

# Потоки телеметрии
flight = vessel.flight()
orbit = vessel.orbit
ut = conn.add_stream(getattr, conn.space_center, 'ut')
latitude = conn.add_stream(getattr, flight, 'latitude')
longitude = conn.add_stream(getattr, flight, 'longitude')
alt = conn.add_stream(getattr, flight, 'mean_altitude')
apo = conn.add_stream(getattr, orbit, 'apoapsis_altitude')
peri = conn.add_stream(getattr, orbit, 'periapsis_altitude')
speed = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'speed')
mass = conn.add_stream(getattr, vessel, 'mass')
dyn_press = conn.add_stream(getattr, flight, 'dynamic_pressure')
surf_flight = vessel.flight(vessel.surface_reference_frame)
v_h = conn.add_stream(getattr, surf_flight, 'horizontal_speed')
I_ud = conn.add_stream(getattr, vessel, 'specific_impulse')
F_thrust = conn.add_stream(getattr, vessel, 'thrust')

# переменные для лога
m_start = None
m_final_value = None
_prev_t = None
_prev_vh = None

# отсчет
for i in range(3, 0, -1):
    print(f'{i}...')
    time.sleep(1)
print('ЗАПУСК!')

# Создаем CSV и функцию для сохранения логов
with open(CSV_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "time", "altitude", "apoapsis", "periapsis", "speed", "pitch", "throttle", "mass", "dynamic_pressure",
        "I_ud", "F_thrust", "v_h", "a_h", "m_start", "m_final", "is_MVD"
    ])

def log_data(is_mvd=False):
    global _prev_t, _prev_vh, m_start, m_final_value

    now = time.time() - start_time

    vh_now = v_h()
    if _prev_t is None:
        a_h = 0.0
    else:
        dt = now - _prev_t
        a_h = (vh_now - _prev_vh) / dt if dt > 1e-6 else 0.0

    _prev_t, _prev_vh = now, vh_now

    # если это строка МВД — фиксируем m_final один раз
    if is_mvd:
        m_final_value = mass()

    with open(CSV_FILE, "a", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            round(now, 2),
            round(alt(), 2),
            round(apo(), 2),
            round(peri(), 2),
            round(speed(), 2),
            vessel.flight().pitch,
            vessel.control.throttle,
            round(mass(), 2),
            round(dyn_press(), 2),

            round(I_ud(), 2),
            round(F_thrust(), 2),
            round(vh_now, 2),
            round(a_h, 4),

            round(m_start, 2) if m_start is not None else "",
            round(m_final_value, 2) if m_final_value is not None else "",
            1 if is_mvd else 0
        ])

# Подготовка к запуску
vessel.auto_pilot.target_pitch_and_heading(90, 90)
vessel.auto_pilot.engage()

# Запуск двигателей
vessel.control.throttle = 1.0
vessel.control.activate_next_stage()
start_time = time.time()

# Стартовая масса
m_start = mass()

# Вертикальный подъем до 1 км
while alt() < 1000:
    log_data()
    time.sleep(1)

# Гравитационный разворот
turn_start_altitude = 1000
turn_end_altitude = 45000

# Ждем нужного апогея
while True:
    log_data()
    current_alt = alt()

    # Плавный разворот от 90° до 0°
    if current_alt > turn_start_altitude and current_alt < turn_end_altitude:
        frac = ((current_alt - turn_start_altitude) /
                (turn_end_altitude - turn_start_altitude))
        turn_angle = 90 * (1 - frac)
        vessel.auto_pilot.target_pitch = turn_angle

    # Целевой апогей
    if apo() > 70000:
        vessel.control.rcs = True
        vessel.auto_pilot.target_pitch = 0
        vessel.auto_pilot.target_heading = 90
        print(f"Апогей достигнут: {apo():.0f} м")
        vessel.control.throttle = 0.0



        time.sleep(1)
        vessel.control.activate_next_stage()
        vessel.control.throttle = 0.1
        break

    time.sleep(0.1)

# ОЖИДАЕМ РОВНО 65 СЕКУНД ДО АПОЦЕНТРА
print("Ожидание времени для маневра...")

while vessel.orbit.time_to_apoapsis > 65:
    log_data()
    time_to_apo = vessel.orbit.time_to_apoapsis
    print(f"Время до апоцентра: {time_to_apo:.1f} с")
    time.sleep(2)

print("ПОДГОТОВКА К МАНЕВРУ - 65 СЕКУНД ДО АПОЦЕНТРА!")
vessel.auto_pilot.target_pitch = 0
vessel.auto_pilot.target_heading = 90
time.sleep(3)

print("ЗАПУСК ДВИГАТЕЛЯ ЗА 65 СЕКУНД ДО АПОЦЕНТРА!")
vessel.control.throttle = 1.0

# Следим за орбитой во время маневра
while peri() < 75000:
    log_data()
    time_to_apo = vessel.orbit.time_to_apoapsis
    current_apo = apo()
    current_peri = peri()
    maneuver_time = time.time() - start_time

    print(
        f"Маневр: {maneuver_time:.1f} с | До апоцентра: {time_to_apo:.1f} с | Апо: {current_apo:.0f} м | Пери: {current_peri:.0f} м"
    )

    if current_apo - current_peri <= 30000 and current_peri > 70000:
        vessel.control.throttle = 0.0
        print("Достигнут целевой апоцентр")
        # МВД: записываем строку с m_final и is_MVD=1
        log_data(is_mvd=True)
        break

    time.sleep(0.5)

# Читаем CSV ПОСЛЕ логирования, чтобы графики были полные
df = pd.read_csv(CSV_FILE)

def save_plot(x, y, filename, xlabel, ylabel, title):
    plt.figure()
    plt.plot(df[x], df[y])
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    plt.savefig(os.path.join(BASE_DIR, filename))
    plt.close()

save_plot("time", "altitude", "altitude.png", "Время (c)", "Высота (м)", "Высота vs Время")
save_plot("time", "speed", "speed.png", "Время (c)", "Скорость (м/c)", "Скорость vs Время")
save_plot("time", "pitch", "pitch.png", "Время (c)", "Угол (град)", "Угол vs Время")
save_plot("time", "apoapsis", "apoapsis.png", "Время (c)", "Апоцентр (м)", "Апоцентр vs Время")
save_plot("time", "periapsis", "periapsis.png", "Время (c)", "Перицентр (м)", "Перицентр vs Время")
save_plot("time", "mass", "mass.png", "Время (c)", "Масса (кг)", "Масса ракеты vs Время")
save_plot("time", "dynamic_pressure", "dynamic_pressure.png", "Время (c)", "Давление (Па)", "Давление vs Время")
save_plot("time", "I_ud", "isp.png", "Время (c)", "Isp (с)", "Isp vs Время")
save_plot("time", "F_thrust", "thrust.png", "Время (c)", "Тяга (Н)", "Тяга vs Время")
save_plot("time", "v_h", "v_horizontal.png", "Время (c)", "Гориз. скорость (м/с)", "Гориз. скорость vs Время")
save_plot("time", "a_h", "a_horizontal.png", "Время (c)", "Гориз. ускорение (м/с^2)", "Гориз. ускорение vs Время")

print("Графики сохранены в:", BASE_DIR)

vessel.control.throttle = 0.0
print("Орбитальный маневр завершен")
time.sleep(5)
vessel.control.activate_next_stage()

final_apo = apo()
final_peri = peri()

vessel.auto_pilot.disengage()

print(f"\n=== ОРБИТА ДОСТИГНУТА! ===")
print(f"Апоцентр: {final_apo:.0f} м")
print(f"Перицентр: {final_peri:.0f} м")


"""Выполнение одного витка по орбите"""
print("\n=== НАЧАЛО ОРБИТАЛЬНОГО ВИТКА ===")

start_ut = ut()
orbit_period = vessel.orbit.period

print(f"Период орбиты: {orbit_period:.0f} секунд")
print("Начинаем отсчет витка...")

vessel.control.sas = True
time.sleep(1)

# Прогресс витка
while True:
    current_ut = ut()
    time_elapsed = current_ut - start_ut
    orbit_progress = (time_elapsed / orbit_period) * 100

    if orbit_progress >= 90:
        print("Приближается время схода с орбиты")
        break

    print(f"Прогресс витка: {orbit_progress:.1f}% | Высота: {alt():.0f} м")
    time.sleep(5)

print("\n=== ВИТОК ЗАВЕРШЕН! ===")


"""Сход с орбиты и возвращение на Землю"""
print("\n=== НАЧАЛО СХОДА С ОРБИТЫ ===")

print("Ориентация для тормозного импульса...")
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch = -90
vessel.auto_pilot.target_heading = 270
time.sleep(5)

print("Выполнение тормозного импульса...")
vessel.control.throttle = 1.0
time.sleep(40)

while peri() > 30000:
    print(f"Текущий перицентр: {peri():.0f} м")
    time.sleep(1)

vessel.control.throttle = 0.0
time.sleep(3)
print("Тормозной импульс завершен")
print(f"Перицентр снижен до: {peri():.0f} м")

vessel.auto_pilot.disengage()
vessel.control.activate_next_stage()

print("=== ВХОД В АТМОСФЕРУ ===")

vessel.control.sas = True
time.sleep(1)

vessel.control.sas_mode = conn.space_center.SASMode.retrograde
time.sleep(2)

vessel.control.throttle = 1.0
print("Ориентация: Retrograde (против движения), двигатель включен")

vessel.auto_pilot.target_pitch = -90
vessel.control.throttle = 1.0

# Спуск
while alt() > 2500:
    current_alt = alt()
    speed_now = vessel.flight(vessel.orbit.body.reference_frame).speed

    if current_alt > 50000:
        print(f"Высота: {current_alt:.0f} м | Скорость: {speed_now:.0f} м/с")
        time.sleep(5)
    elif current_alt > 11000:
        print(f"Высота: {current_alt:.0f} м | Скорость: {speed_now:.0f} м/с")
        time.sleep(2)
    elif current_alt > 10000:
        vessel.control.throttle = 0.0
        print(f"Высота: {current_alt:.0f} м | Скорость: {speed_now:.0f} м/с")
        time.sleep(2)
    elif current_alt > 3000:
        print(f"Высота: {current_alt:.0f} м | Скорость: {speed_now:.0f} м/с")
        time.sleep(2)
    else:
        print(f"Высота: {current_alt:.0f} м | Скорость: {speed_now:.0f} м/с")
        vessel.control.activate_next_stage()
        time.sleep(1)

print("\n=== ФИНАЛЬНОЕ СНИЖЕНИЕ ===")

if alt() < 1000:
    vessel.control.activate_next_stage()
    print("Парашюты раскрыты!")

while alt() > 50:
    current_alt = alt()
    vvs = vessel.flight(vessel.orbit.body.reference_frame).vertical_speed
    print(f"Высота: {current_alt:.0f} м | Вертикальная скорость: {vvs:.0f} м/с")
    time.sleep(0.5)

print("\n*** ПОСАДКА! ***")

