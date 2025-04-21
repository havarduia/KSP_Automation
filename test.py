import krpc
import time
import math
import matplotlib.pyplot as plt

# === Connection ===
print("Connecting to kRPC...")
try:
    conn = krpc.connect(name='Improved Gravity Turn Final Staging Fixed') # Updated name
    vessel = conn.space_center.active_vessel
    print(f"Connected to vessel: {vessel.name}")
except krpc.error.NetworkError as e:
    print(f"Connection Error: {e}")
    print("Ensure KSP is running and the kRPC server is started.")
    exit()
except Exception as e:
    print(f"An unexpected error occurred during connection: {e}")
    exit()

# === Reference Frames ===
body_ref = vessel.orbit.body.reference_frame
orbit_ref = vessel.orbit.body.orbital_reference_frame # Non-rotating frame for orbital speed/vectors
surface_ref = vessel.surface_reference_frame # *** USE THIS for ascent pitch/heading ***

# === Parameters ===
# Ascent
TURN_START_ALTITUDE   = 1000     # m - Altitude to start pitchover
TURN_END_ALTITUDE     = 45000    # m - Altitude where pitch is mostly horizontal (tune based on TWR)
TARGET_APOAPSIS       = 80000    # m - Target apoapsis altitude during ascent
MAX_DYNAMIC_PRESSURE  = 22000    # Pa - Throttle down HARD if exceeding this
TARGET_DYNAMIC_PRESSURE = 18000  # Pa - Aim to stay below this by throttling
MIN_THROTTLE          = 0.1      # Minimum throttle to prevent flameout/instability
GRAVITY_TURN_EXPONENT = 0.7      # Controls turn curve shape (1=linear, <1=faster initial turn, >1=slower initial turn)

# Circularization
TARGET_PERIAPSIS      = 80000    # m - Target periapsis altitude for final orbit
CIRC_BURN_LEAD_TIME   = 8        # Seconds before apoapsis to START THE BURN CALCULATION
CIRC_THROTTLE         = 0.6      # Throttle for circularization burn (0.2-1.0)

# === Telemetry Data Containers ===
data = {'alt': [], 'spd': [], 'q': [], 'twr': [], 'pitch': [], 'apo': [], 'peri': []}

# === Telemetry Dashboard Setup ===
plt.ion()
fig, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=False)
ax1, ax2, ax3 = axes.flatten()

# Plot 1: Altitude vs Speed & Pitch
l_spd, = ax1.plot([], [], label='Orbital Speed (m/s)', color='blue') # Plotting Orbital Speed
ax1.set_ylabel('Orbital Speed (m/s)', color='blue')
ax1.tick_params(axis='y', labelcolor='blue')
ax1.grid(True)
ax1b = ax1.twinx()
l_pitch, = ax1b.plot([], [], label='Pitch (deg)', color='red', linestyle='--')
ax1b.set_ylabel('Pitch (deg)', color='red')
ax1b.tick_params(axis='y', labelcolor='red')
ax1b.set_ylim(0, 95)
ax1.set_xlabel('Altitude (km)')
lines1, labels1 = ax1.get_legend_handles_labels()
lines1b, labels1b = ax1b.get_legend_handles_labels()
ax1.legend(lines1 + lines1b, labels1 + labels1b, loc='center left')
ax1.set_title('Ascent Profile')

# Plot 2: Altitude vs Dynamic Pressure & TWR
l_q, = ax2.plot([], [], label='Dyn. Pressure (kPa)', color='green')
ax2.set_ylabel('Dyn. Pressure (kPa)', color='green')
ax2.tick_params(axis='y', labelcolor='green')
ax2.grid(True)
ax2b = ax2.twinx()
l_twr, = ax2b.plot([], [], label='TWR', color='purple', linestyle=':')
ax2b.set_ylabel('TWR', color='purple')
ax2b.tick_params(axis='y', labelcolor='purple')
ax2.set_xlabel('Altitude (km)')
lines2, labels2 = ax2.get_legend_handles_labels()
lines2b, labels2b = ax2b.get_legend_handles_labels()
ax2.legend(lines2 + lines2b, labels2 + labels2b, loc='center left')

# Plot 3: Altitude vs Apoapsis/Periapsis
l_apo, = ax3.plot([], [], label='Apoapsis (km)', color='orange')
l_peri, = ax3.plot([], [], label='Periapsis (km)', color='cyan')
ax3.set_xlabel('Altitude (km)')
ax3.set_ylabel('Orbit Altitudes (km)')
ax3.grid(True)
ax3.axhline(TARGET_APOAPSIS / 1000, color='red', linestyle='--', linewidth=0.8, label=f'Target Orbit ({TARGET_APOAPSIS/1000:.0f}km)')
ax3.legend(loc='center left')

fig.tight_layout()
plt.show(block=False)

# === kRPC Streams ===
print("Setting up kRPC streams...")
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
orbital_speed = conn.add_stream(getattr, vessel.flight(orbit_ref), 'speed') # For plot
surface_speed = conn.add_stream(getattr, vessel.flight(surface_ref), 'speed') # For status print
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
periapsis = conn.add_stream(getattr, vessel.orbit, 'periapsis_altitude')
dyn_pressure = conn.add_stream(getattr, vessel.flight(body_ref), 'dynamic_pressure')
thrust = conn.add_stream(getattr, vessel, 'available_thrust') # Use available_thrust for TWR helper
mass = conn.add_stream(getattr, vessel, 'mass')
current_stage = conn.add_stream(getattr, vessel.control, 'current_stage')
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
pitch = conn.add_stream(getattr, vessel.flight(surface_ref), 'pitch') # Use surface_ref
surface_gravity = conn.add_stream(getattr, vessel.orbit.body, 'surface_gravity')
ut = conn.add_stream(getattr, conn.space_center, 'ut')

# === Helper Functions ===
def get_current_twr():
    """Calculates Thrust-to-Weight Ratio using stream data."""
    m = mass()
    g = surface_gravity()
    # Use available_thrust from stream for TWR calculation
    # as vessel.thrust might be zero momentarily during staging
    t = thrust()
    if m <= 0 or g <= 0: return 0
    # Consider available thrust - if > 0, TWR is potentially > 0
    # If actual thrust is needed, read vessel.thrust directly
    return t / (m * g) if t > 0 else 0


def calculate_target_pitch(alt):
    """Calculates the target pitch angle based on altitude for gravity turn."""
    if alt < TURN_START_ALTITUDE:
        return 90.0
    elif alt > TURN_END_ALTITUDE:
        return 0.0
    else:
        turn_frac = (alt - TURN_START_ALTITUDE) / (TURN_END_ALTITUDE - TURN_START_ALTITUDE)
        turn_frac = max(0.0, min(1.0, turn_frac))
        pitch_angle = 90.0 * (1.0 - turn_frac ** GRAVITY_TURN_EXPONENT)
        return max(0.0, pitch_angle)

def manage_throttle(q):
    """Adjusts throttle to manage dynamic pressure."""
    current_throttle = vessel.control.throttle
    target_throttle = 1.0 # Default to full throttle

    if q > MAX_DYNAMIC_PRESSURE:
        # Print only once when crossing threshold
        if current_throttle > MIN_THROTTLE + 0.01:
             print(f"\nWARNING: q exceeded MAX ({q/1000:.1f} kPa > {MAX_DYNAMIC_PRESSURE/1000:.1f} kPa)! Forcing lower throttle.")
        over_limit_factor = (q - MAX_DYNAMIC_PRESSURE) / (MAX_DYNAMIC_PRESSURE - TARGET_DYNAMIC_PRESSURE)
        target_throttle = 1.0 - (0.4 + 0.6 * min(1.0, over_limit_factor))
    elif q > TARGET_DYNAMIC_PRESSURE:
        throttle_reduction_factor = (q - TARGET_DYNAMIC_PRESSURE) / (MAX_DYNAMIC_PRESSURE - TARGET_DYNAMIC_PRESSURE)
        target_throttle = 1.0 - throttle_reduction_factor * 0.8
    else:
         target_throttle = 1.0 # Below target, allow full throttle

    return max(MIN_THROTTLE, target_throttle)


def check_staging():
    """Checks if the current stage needs to be activated based on fuel."""
    stage = current_stage()
    if stage <= 0: return False
    try:
        resources = vessel.resources_in_decouple_stage(stage - 1, cumulative=False)
    except krpc.error.RPCError:
         return False # Ignore errors that might happen during staging

    liquid_fuel = resources.amount('LiquidFuel')
    solid_fuel = resources.amount('SolidFuel')
    oxidizer = resources.amount('Oxidizer')
    stage_if_lf_empty = resources.max('LiquidFuel') > 0 and liquid_fuel < 0.1 and oxidizer < 0.1
    stage_if_srb_empty = resources.max('SolidFuel') > 0 and solid_fuel < 0.1

    if stage_if_lf_empty or stage_if_srb_empty:
        print(f"\nStaging: Stage {stage - 1} fuel depleted (LF: {liquid_fuel:.1f}, Ox: {oxidizer:.1f}, SF: {solid_fuel:.1f}). Activating stage {stage - 1}.")
        return True
    return False

last_plot_update_ut = 0
def update_dashboard(force_update=False):
    """Appends current data to lists and updates plots, throttled by UT."""
    global last_plot_update_ut
    current_ut = ut()
    if not force_update and (current_ut - last_plot_update_ut < 0.2):
        return
    last_plot_update_ut = current_ut

    try:
        alt_val = altitude() / 1000 # km
        spd_val = orbital_speed() # Plotting orbital speed
        q_val = dyn_pressure() / 1000 # kPa
        twr_val = get_current_twr() # Uses available_thrust stream
        pitch_val = pitch() # Read from stream (using surface_ref)
        apo_val = apoapsis() / 1000 # km
        peri_val = periapsis() / 1000 # km

        data['alt'].append(alt_val)
        data['spd'].append(spd_val)
        data['q'].append(q_val)
        data['twr'].append(twr_val)
        data['pitch'].append(pitch_val)
        data['apo'].append(apo_val)
        data['peri'].append(peri_val)

        # Update plots... (code is unchanged here)
        l_spd.set_data(data['alt'], data['spd'])
        l_pitch.set_data(data['alt'], data['pitch'])
        ax1.relim()
        ax1.autoscale_view(tight=False)
        ax1b.relim()
        ax1b.autoscale_view(tight=False)
        ax1b.set_ylim(0, 95)

        l_q.set_data(data['alt'], data['q'])
        l_twr.set_data(data['alt'], data['twr'])
        ax2.relim()
        ax2.autoscale_view(tight=False)
        ax2b.relim()
        ax2b.autoscale_view(tight=False)
        ax2b.set_ylim(bottom=0)

        l_apo.set_data(data['alt'], data['apo'])
        l_peri.set_data(data['alt'], data['peri'])
        ax3.relim()
        ax3.autoscale_view(tight=False)
        min_peri_plot = min(data['peri']) if data['peri'] else -vessel.orbit.body.equatorial_radius / 2000
        max_apo_plot = max(max(data['apo']) if data['apo'] else TARGET_APOAPSIS/1000, TARGET_APOAPSIS/1000 * 1.1)
        ax3.set_ylim(min_peri_plot - 10, max_apo_plot + 10)

        max_alt_plot = max(data['alt']) if data['alt'] else 1
        for ax in axes:
            ax.set_xlim(0, max_alt_plot * 1.05)

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    except Exception as e:
        if 'Stream error' not in str(e): # Avoid spamming stream errors
             print(f"\nError updating dashboard: {e}")
        pass

# === Launch Sequence ===
print("Configuring launch...")
vessel.control.throttle = 0.0
vessel.control.sas = False
vessel.control.rcs = False
vessel.auto_pilot.engage()
vessel.auto_pilot.reference_frame = surface_ref # Use surface_ref for initial alignment
vessel.auto_pilot.target_pitch_and_heading(90, 90)
print("Initial target set (skipping indefinite wait).")
# vessel.auto_pilot.wait() # NO WAIT ON PAD

vessel.control.throttle = 1.0 # Set throttle before countdown

print("Launch Countdown:")
for i in range(3, 0, -1):
    print(f"{i}...")
    time.sleep(1)
print("Liftoff!")
liftoff_time = time.time() # Record liftoff time

# *** SIMPLIFIED AND CORRECTED STAGING: Unconditionally activate the first stage ***
print("Activating next stage (Stage 0 engines / launch clamps / decouplers)...")
try:
    vessel.control.activate_next_stage()
    print("activate_next_stage() command sent.")
    # Add a very brief pause to allow KSP physics/engines to react
    time.sleep(0.1)
    # Optional immediate check after staging command
    current_thrust_after_stage = vessel.thrust
    if current_thrust_after_stage > 1:
         print(f"Confirmed Thrust Detected: {current_thrust_after_stage:.1f} N")
    else:
         print("WARN: Thrust still not detected immediately after staging command.")

except Exception as e:
    print(f"Error during activate_next_stage(): {e}")
    # Consider adding logic here to stop the script if staging fails critically


# === Ascent Loop ===
print("Starting ascent guidance...")
last_print_time = time.time()
while True: # Main loop condition
    try:
        # Read streams needed for logic first
        alt = altitude()
        ap = apoapsis()

        # --- Check loop exit condition ---
        if ap >= TARGET_APOAPSIS:
            print(f"\nTarget apoapsis ({TARGET_APOAPSIS/1000:.1f}km) reached ({ap/1000:,.1f}km). Proceeding to coast phase.")
            vessel.control.throttle = 0.0
            break # Exit ascent loop
        if ap < -1000 : # Check for escape trajectory
             print("\nWARNING: Apoapsis negative - likely escape trajectory! Cutting throttle.")
             vessel.control.throttle = 0.0
             break # Exit ascent loop

        # --- Staging ---
        if check_staging():
            vessel.control.activate_next_stage()
            time.sleep(0.5) # Give time for stage separation/engine activation

        # --- Throttle Control ---
        q = dyn_pressure() # Read dynamic pressure stream
        if alt > 1000: # Only manage throttle after initial liftoff phase
          target_throttle = manage_throttle(q)
          vessel.control.throttle = target_throttle
        elif vessel.thrust > 0: # If engines are running but below 1km
            vessel.control.throttle = 1.0 # Ensure full throttle
        # If alt < 1000 and thrust is 0, don't change throttle (might still be igniting)

        # --- Guidance ---
        target_pitch = calculate_target_pitch(alt)
        # *** Ensure autopilot uses SURFACE reference frame consistently ***
        if vessel.auto_pilot.reference_frame != surface_ref:
             # print("\nINFO: Setting autopilot reference frame to surface_ref") # Optional: Log if it changes
             vessel.auto_pilot.reference_frame = surface_ref
        vessel.auto_pilot.target_pitch_and_heading(target_pitch, 90)

        # --- DEBUG ENGINE/FUEL BLOCK (Optional - Can be removed if things work) ---
        current_time_in_flight = time.time() - liftoff_time
        if current_time_in_flight < 10.0 and current_time_in_flight > 0.5:
             try:
                 # Only print this debug info periodically to reduce spam
                 if time.time() - last_print_time > 0.8: # Match status print frequency
                     print(f"\n--- ENGINE DEBUG (T+{current_time_in_flight:.1f}s) ---")
                     direct_thrust = vessel.thrust
                     direct_avail_thrust = vessel.available_thrust
                     direct_dyn_press = vessel.flight(body_ref).dynamic_pressure
                     print(f"Direct Thrust: {direct_thrust:.1f} N")
                     print(f"Direct Avail Thrust: {direct_avail_thrust:.1f} N")
                     print(f"Direct Dyn Pressure: {direct_dyn_press/1000:.2f} kPa")
                     lf = vessel.resources.amount('LiquidFuel')
                     ox = vessel.resources.amount('Oxidizer')
                     sf = vessel.resources.amount('SolidFuel')
                     print(f"Vessel Fuel - LF: {lf:.1f}, Ox: {ox:.1f}, SF: {sf:.1f}")
                     active_engine_info = []
                     for engine in vessel.parts.engines:
                         if engine.active:
                              active_engine_info.append(f"  - {engine.part.name} (Active: {engine.active}, Thrust: {engine.thrust:.1f}N, HasFuel: {engine.has_fuel})")
                     if active_engine_info: print("Active Engines:"); [print(info) for info in active_engine_info]
                     else: print("No Active Engines Found!")
                     print(f"--------------------------")

             except Exception as debug_e:
                 print(f"\nError during engine debug: {debug_e}")

        # --- Telemetry & Status Update ---
        update_dashboard()
        current_time = time.time()
        if current_time - last_print_time > 1.0: # Print status once per second
            # Calculate TWR using DIRECT actual thrust for status line accuracy
            m = mass()
            g = surface_gravity()
            direct_t = vessel.thrust # Use direct read here for status
            twr = (direct_t / (m * g)) if m > 0 and g > 0 and direct_t > 0 else 0

            current_pitch_val = pitch()
            # Read q directly for status line as stream might lag
            q_direct_status = vessel.flight(body_ref).dynamic_pressure

            # Use surface_speed() for status print
            print(f"Alt: {alt/1000:,.1f}km | Spd(Srf): {surface_speed():,.0f}m/s | Pitch: {current_pitch_val:.1f}°(Tgt:{target_pitch:.1f}°) | Apo: {ap/1000:,.1f}km | Pe: {periapsis()/1000:,.1f}km | q: {q_direct_status/1000:.1f}kPa | Thr: {vessel.control.throttle*100:.0f}% | TWR: {twr:.2f}   ", end='\r')
            last_print_time = current_time

        time.sleep(0.02) # Main loop sleep time

    except krpc.error.RPCError as e:
        if "connection closed" in str(e).lower() or "vessel destroyed" in str(e).lower():
            print(f"\nRPC Error indicating connection/vessel lost: {e}. Stopping script.")
            break
        else:
            print(f"\nRPC Error during ascent loop: {e}. Trying to continue...")
            time.sleep(1)
    except Exception as e:
        print(f"\nUnexpected Error during ascent loop: {e}. Stopping ascent.")
        try: vessel.control.throttle = 0.0
        except Exception: pass # Ignore errors stopping throttle if connection is lost
        break


# === Coast Phase ===
try:
    print(f"\nAscent loop exited. Final Apoapsis: {apoapsis()/1000:,.1f}km. Cutting throttle.")
    vessel.control.throttle = 0.0
    initial_coast_apoapsis = apoapsis()
    print(f"Initial coasting Apoapsis: {initial_coast_apoapsis / 1000:,.1f} km")
    update_dashboard(force_update=True)
except Exception as e:
    print(f"Error during post-ascent setup: {e}")
    initial_coast_apoapsis = -1 # Indicate error


# === Circularization Burn ===
if initial_coast_apoapsis > 0:
    print("Coasting to apoapsis for circularization burn...")
    try:
        # Autopilot setup for prograde burn
        vessel.auto_pilot.disengage()
        time.sleep(0.1)
        vessel.auto_pilot.engage()
        vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame # Use non-rotating frame
        vessel.auto_pilot.target_direction = (0, 1, 0) # Point Prograde

        # Wait until close to Apoapsis to calculate burn
        while time_to_apoapsis() > CIRC_BURN_LEAD_TIME:
            update_dashboard()
            print(f"Coasting... Time to Calc: {max(0, time_to_apoapsis() - CIRC_BURN_LEAD_TIME):.1f}s | Apo: {apoapsis()/1000:.1f}km | Pe: {periapsis()/1000:.1f}km | TTA: {time_to_apoapsis():.1f}s       ", end='\r')
            time.sleep(0.2)

        print("\nCalculating circularization burn...")
        time.sleep(0.1) # Pause for streams

        # Burn Calculation Logic (remains the same)
        mu = vessel.orbit.body.gravitational_parameter
        delta_v = 0; full_throttle_burn_time = 0; actual_burn_time_estimate = 0
        calculation_successful = False
        try:
            current_apo_radius = max(1.0, vessel.orbit.apoapsis)
            current_speed_near_apo = max(0.0, vessel.orbit.speed)
            if current_apo_radius <= 0: raise ValueError("Apoapsis radius zero/negative.")
            v_circ_apo = math.sqrt(mu / current_apo_radius)
            delta_v = v_circ_apo - current_speed_near_apo
            print(f"--- Burn Calculation Inputs ---"); print(f"TTA: {time_to_apoapsis():.2f}s, ApRad: {current_apo_radius:,.0f}m, Speed: {current_speed_near_apo:.1f}m/s, Vcirc: {v_circ_apo:.1f}m/s, dV: {delta_v:.1f}m/s"); print(f"-----------------------------")
            if delta_v < -5.0: delta_v = 0; print("WARN: dV negative.")
            elif delta_v < 0: delta_v = 0; print("INFO: dV slightly negative.")

            current_thrust = vessel.available_thrust # Use available for calculation planning
            current_isp = vessel.specific_impulse
            current_m = mass()
            g0 = 9.80665
            if current_thrust > 0 and current_isp > 0 and current_m > 0 and delta_v > 0:
                m_final = current_m * math.exp(-delta_v / (current_isp * g0)); mass_flow_rate = current_thrust / (current_isp * g0)
                if mass_flow_rate > 0 : full_throttle_burn_time = (current_m - m_final) / mass_flow_rate; calculation_successful = True
                else: print("WARN: MFR zero.")
            elif delta_v > 0: print("WARN: Cannot calc Tsiolkovsky time.")
            if not calculation_successful and delta_v > 0:
                if current_m > 0 and current_thrust > 0: avg_acceleration = current_thrust / current_m; full_throttle_burn_time = delta_v / avg_acceleration; print(f"WARN: Using F=ma estimate: {full_throttle_burn_time:.2f}s"); calculation_successful = True
                else: print("ERROR: Cannot estimate burn time.");
            if calculation_successful and CIRC_THROTTLE > 0.01: actual_burn_time_estimate = full_throttle_burn_time / CIRC_THROTTLE
            elif calculation_successful: print("WARN: Low throttle."); actual_burn_time_estimate = float('inf')
            print(f"Est. FT burn: {full_throttle_burn_time:.2f}s"); print(f"Using {CIRC_THROTTLE*100:.0f}% throttle."); print(f"Adj. est. burn: {actual_burn_time_estimate:.2f}s")
        except ValueError as e: print(f"ERROR calc burn: {e}."); delta_v = 0
        except Exception as e: print(f"UNEXPECTED ERROR calc burn: {e}."); delta_v = 0

        # --- Burn Execution ---
        if delta_v > 0 and calculation_successful and actual_burn_time_estimate >= 0:
            time_to_apo = time_to_apoapsis()
            burn_start_offset = actual_burn_time_estimate / 2.0
            wait_time = max(0, time_to_apo - burn_start_offset)
            print(f"TTA: {time_to_apo:.1f}s. Waiting {wait_time:.1f}s (starts {burn_start_offset:.1f}s before predicted Apo)...")
            time.sleep(wait_time)

            print("Executing circularization burn...")
            vessel.auto_pilot.target_direction = (0, 1, 0) # Ensure prograde target

            # *** REPLACE wait() with a fixed sleep ***
            # vessel.auto_pilot.wait() # REMOVED - Likely cause of hang
            print("Pausing briefly for alignment...")
            time.sleep(1.5) # Give autopilot 1.5 seconds to point prograde

            # Add a check *after* the sleep to see where we are pointing
            try:
                flight_info = vessel.flight(vessel.orbital_reference_frame)
                prograde_direction = flight_info.prograde # Ideal direction
                current_direction = flight_info.direction # Actual direction
                # Calculate angle between vectors (dot product)
                dot_product = sum(x*y for x, y in zip(prograde_direction, current_direction))
                dot_product = max(-1.0, min(1.0, dot_product)) # Clamp to avoid math errors
                angle_error = math.degrees(math.acos(dot_product))
                print(f"Alignment Error after pause: {angle_error:.2f} degrees")
            except Exception as align_e:
                print(f"Could not check alignment: {align_e}")


            print("Setting throttle and starting burn loop...")
            vessel.control.throttle = CIRC_THROTTLE
            burn_start_time = time.time()

            # *** Add print BEFORE the loop ***
            print(f"Checking burn condition: periapsis() = {periapsis()/1000:.1f}km < TARGET_PERIAPSIS = {TARGET_PERIAPSIS/1000:.1f}km")

            while periapsis() < TARGET_PERIAPSIS:
                # *** Add print INSIDE the loop (first thing) ***
                # print(f"Entered burn loop. Pe = {periapsis()/1000:.1f}km") # Optional debug, can be noisy

                update_dashboard()
                current_burn_time = time.time() - burn_start_time
                # The rest of the loop (status print, safety checks, sleep) remains the same
                print(f"Burning... Target Pe: {TARGET_PERIAPSIS/1000:.1f}km | Curr Pe: {periapsis()/1000:.1f}km | Apo: {apoapsis()/1000:.1f}km | Burn Time: {current_burn_time:.1f}s / {actual_burn_time_estimate:.1f}s    ", end='\r')

                # Safety checks...
                if actual_burn_time_estimate > 0.1 and current_burn_time > actual_burn_time_estimate * 1.8: print("\nWARN: Burn time exceeded estimate."); break
                if apoapsis() > max(TARGET_APOAPSIS * 1.25, initial_coast_apoapsis * 1.1): print(f"\nWARN: Apoapsis high."); break
                try:
                    if vessel.available_thrust < 1 and vessel.max_thrust > 0: print("\nWARN: Lost thrust."); break
                except krpc.error.RPCError: print("\nWARN: RPC Error checking thrust."); break
                if periapsis() > TARGET_PERIAPSIS * 1.03: print(f"\nINFO: Periapsis overshoot."); break
                time.sleep(0.02)

            vessel.control.throttle = 0.0
            print("\nCircularization burn complete.")
        else:
             # Print reason for skipping
             if not calculation_successful: print("Skipping circ burn (Calc Error).")
             elif delta_v <= 0: print("Skipping circ burn (dV <= 0).")
             elif actual_burn_time_estimate < 0: print("Skipping circ burn (Neg time).")

    except Exception as e:
        print(f"\nError during circularization phase: {e}")
        try: vessel.control.throttle = 0.0
        except Exception: pass
else:
     print("\nSkipping circularization phase due to issue during ascent.")


# === Post-Burn ===
try:
    vessel.auto_pilot.disengage()
    vessel.control.sas = True
    print("SAS Enabled.")
    time.sleep(0.1)
    if conn.space_center.SASMode.prograde in vessel.control.available_sas_modes:
        vessel.control.sas_mode = conn.space_center.SASMode.prograde; print("SAS Mode: Prograde.")
    elif conn.space_center.SASMode.stability_assist in vessel.control.available_sas_modes:
         vessel.control.sas_mode = conn.space_center.SASMode.stability_assist; print("SAS Mode: Stability Assist.")
except Exception as e:
     print(f"Could not set SAS mode: {e}")

# Final orbit printout
try:
    final_apo = apoapsis(); final_peri = periapsis()
    print("\n--- FINAL ORBIT ---"); print(f"Apoapsis: {final_apo/1000:,.1f} km"); print(f"Periapsis: {final_peri/1000:,.1f} km")
    print(f"(Target: {TARGET_APOAPSIS/1000:,.1f} km x {TARGET_PERIAPSIS/1000:,.1f} km)"); print("-------------------")
except Exception as e: print(f"\nError getting final orbit details: {e}")


# Keep plot open until manually closed
print("Script finished. Close the plot window to exit.")
plt.ioff()
update_dashboard(force_update=True) # Ensure final state is plotted
plt.show(block=True) # Block execution until plot window is closed

print("Closing connection.")
conn.close()