import krpc, time, math, matplotlib.pyplot as plt

# === Connection ===
conn   = krpc.connect(name='Fixed Launch')
vessel = conn.space_center.active_vessel
flight = vessel.flight(vessel.orbit.body.reference_frame)
orbit  = vessel.orbit

# === Parameters ===
TURN_START_ALT        = 1000     # m
TURN_END_ALT          = 45000    # m
MAX_DYNAMIC_PRESSURE  = 20000    # Pa
SAFE_DYNAMIC_PRESSURE = 10000    # Pa
TARGET_APOAPSIS       = 75000    # m

# === Telemetry Data Containers ===
# Ensure 'data' dict is defined before use
data = {'alt': [], 'spd': [], 'q': [], 'twr': []}

# === Telemetry Dashboard Setup ===
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))

l1, = ax1.plot([], [], label='Speed vs Altitude')
ax1.set_xlabel('Altitude (m)')
ax1.set_ylabel('Speed (m/s)')
ax1.grid(True)

l2, = ax2.plot([], [], label='Q vs TWR')
ax2.set_xlabel('Dynamic Pressure (Pa)')
ax2.set_ylabel('TWR')
ax2.grid(True)

# === Helper Functions ===
def get_twr():
    return vessel.available_thrust / (vessel.mass * vessel.orbit.body.surface_gravity)

def limit_throttle(q):
    # Full throttle until safe dynamic pressure threshold
    if q > MAX_DYNAMIC_PRESSURE:
        return 0.3
    elif q > SAFE_DYNAMIC_PRESSURE:
        return 0.6
    return 1.0

def sigmoid_turn(alt, twr):
    # Start & end of gravity turn
    if alt < TURN_START_ALT:
        return 0
    if alt > TURN_END_ALT:
        return 90
    frac = (alt - TURN_START_ALT) / (TURN_END_ALT - TURN_START_ALT)
    s = 1 / (1 + math.exp(-12 * (frac - 0.5)))
    return min(s * 90 * twr, 90)

def update_dashboard(alt, spd, q, twr):
    # Append data and update plots
    data['alt'].append(alt)
    data['spd'].append(spd)
    data['q'].append(q)
    data['twr'].append(twr)
    
    l1.set_xdata(data['alt'])
    l1.set_ydata(data['spd'])
    ax1.relim()
    ax1.autoscale_view()

    l2.set_xdata(data['q'])
    l2.set_ydata(data['twr'])
    ax2.relim()
    ax2.autoscale_view()

    plt.pause(0.001)

# === Launch Sequence ===
print("Liftoff!")
vessel.control.throttle = 1.0
vessel.control.sas = False
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)
vessel.control.activate_next_stage()
time.sleep(1)

# === Ascent Loop ===
while True:
    alt = flight.mean_altitude
    spd = flight.speed
    q   = flight.dynamic_pressure
    twr = get_twr()
    apo = orbit.apoapsis_altitude

    # Gravity turn
    angle = sigmoid_turn(alt, twr)
    vessel.auto_pilot.target_pitch_and_heading(90 - angle, 90)

    # Throttle control
    thr = limit_throttle(q)
    # Cap TWR if too high
    if twr > 2.0:
        thr = min(thr, 0.8)
    vessel.control.throttle = thr

    if q > SAFE_DYNAMIC_PRESSURE:
        print(f"⚠️ Q={q:.0f} Pa, throttle={thr*100:.0f}%")

    # Update telemetry
    update_dashboard(alt, spd, q, twr)

    # Auto-stage
    if vessel.control.current_stage > 0 and vessel.resources.amount('LiquidFuel') < 1:
        vessel.control.activate_next_stage()

    if apo >= TARGET_APOAPSIS:
        vessel.control.throttle = 0.0
        print(f"Apoapsis reached: {apo:.0f} m")
        break

    time.sleep(0.1)

plt.ioff()
plt.show()