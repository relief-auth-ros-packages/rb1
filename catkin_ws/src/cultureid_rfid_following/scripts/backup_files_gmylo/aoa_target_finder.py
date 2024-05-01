import time

def found(input_path, EPC_str, past_estimate, N_start, reusability, sigma, den_thres):

    import math
    import matplotlib.pyplot as plt
    import numpy as np

    # #######################################################################################################
    # Settings
    # #######################################################################################################
    EPC = int(EPC_str, 16)
    file_to_be_cleared = True
    ant1_no_left = 2
    ant1_no_right = 3
    phase_DC = 0.0
    frequency = 867.5 * (10 ** 6)
    lamda = 299792458 / frequency
    L = 0.13
    past_estimation = float(past_estimate)
    spread = float(sigma)
    density_threshold = float(den_thres)

    # #######################################################################################################
    # Global Variables
    # #######################################################################################################
    left_phase_raw = []
    right_phase_raw = []
    delta_phase = []
    alt_delta_phase = []
    phase_time = []
    antenna = []
    data_density = []
    pi = math.pi
    N_particles = int(float(N_start))
    reuse_factor = float(reusability)
    N_offsprings = 2
    min_particles_to_be_reused = 20
    if past_estimation == 1000:
        no_dist_known = True
    else:
        no_dist_known = False
    particles = []
    lost_phase_time_threshold = 0.02
    avg_rssi = [0.0]
    density_counter = [0]

    # #######################################################################################################
    # get the sign of the point
    # #######################################################################################################
    def sgn(point):
        if point < 0:
            return -1
        else:
            return 1

    # #######################################################################################################
    # Create the particle list for the p_filtering process
    # If there is no prior information, the distribution is uniform
    # #######################################################################################################
    def create_particle_list():
        if no_dist_known:
            for idx in range(N_particles):
                new_particle = Particle(np.random.uniform(-pi/2, pi/2))
                particles.append(new_particle)
        else:
            for idx in range(N_particles):
                poss_theta = np.random.normal(past_estimation, spread)
                if abs(poss_theta) > pi/2:
                    poss_theta = sgn(poss_theta)*pi/2 - sgn(poss_theta)*(abs(poss_theta) - pi/2)
                new_particle = Particle(poss_theta)
                particles.append(new_particle)

    # #######################################################################################################
    # Evaluate the current possible particles based on the latest data entry and update their weights.
    # data_entry = [antenna_number, measured_phase, measured_rssi]
    # #######################################################################################################
    def evaluate_particles(theta_entry):
        for idx in range(particles.__len__()):
            calculate_weight(theta_entry, particles[idx])

    # #######################################################################################################
    # Sort the particles based on their weight and delete the worst estimations.
    # For the reuse_factor percentage of surviving particles generate offsprings
    # #######################################################################################################
    def update_particles():
        particles.sort(key=get_weight)
        density_counter.append(density_counter[-1] + 1)
        density_counter.pop(0)
        #print("Updated")
        N_particles_to_be_reused = int(reuse_factor * particles.__len__())
        if N_particles_to_be_reused < min_particles_to_be_reused:
            N_particles_to_be_reused = min_particles_to_be_reused
        if N_particles_to_be_reused > N_particles:
            N_particles_to_be_reused = N_particles
        for idx in range(particles.__len__()-N_particles_to_be_reused):
            particles.pop(0)
        for idx in range(particles.__len__()):
            for j in range(N_offsprings):
                poss_theta = particles[idx].theta + np.random.normal(0, spread)
                if abs(poss_theta) > pi/2:
                    poss_theta = sgn(poss_theta)*pi/2 - sgn(poss_theta)*(abs(poss_theta) - pi/2)
                new_particle = Particle(poss_theta)
                particles.append(new_particle)

    # #######################################################################################################
    # Calculate the weight of the specific particle for the current data entry.
    # data_entry = [antenna_number, measured_phase, measured_rssi]
    # #######################################################################################################
    def calculate_weight(theta_entry, curr_particle):
        theta_offset = min(abs(theta_entry[0]-curr_particle.theta), abs(theta_entry[1]-curr_particle.theta))
        curr_particle.set_weight((pi-theta_offset) / pi)

    # #######################################################################################################
    # Make estimation based on current particles.
    # #######################################################################################################
    def make_estimation():
        avg_theta = 0
        for idx in range(particles.__len__()):
            avg_theta = avg_theta + particles[idx].theta
        avg_theta = avg_theta / particles.__len__()
        return avg_theta

    # #######################################################################################################
    # particle class:
    # Main features: { x_from_robot, y_from_robot} in relation to the robot
    # Main features are not calculatable. They are generated from distributions.
    # Calculatable fratures: {phase_to_left_antenna, phase_to_right_antenna, theta_to_robot, rssi}
    # Calculatable features are extracted from the main features.
    # #######################################################################################################
    class Particle:
        def __init__(self, theta_to_normal):
            self.theta = theta_to_normal
            self.weight = 0

        def set_weight(self, new_weight):
            self.weight = new_weight

    # #######################################################################################################
    # function to be used as key for sorting the particles according to weight 0->1
    # #######################################################################################################
    def get_weight(curr_particle):
        return curr_particle.weight

    def show_particles(flag, N):
        if flag:
            for idx in range(N):
                plt.plot(math.sin(particles[-idx-1].theta), math.cos(particles[-idx-1].theta), "bo")
            plt.xlim(-1.1, 1.1)
            plt.ylim(0, 1.1)
            plt.grid(True)
            plt.show()
        else:
            for idx in range(particles.__len__()):
                print("Particle no: ", idx, " with theta and weight: ", particles[idx].theta, particles[idx].weight)

    # #######################################################################################################
    # change first data entries for the antenna that was not read first, to be equal to
    # first actual data and not 0
    # #######################################################################################################
    def fix_first_data_entries():
        fix_first_left = False
        fix_first_right = False
        first_left = 0
        first_right = 0
        i = 0
        while not fix_first_left and i < phase_time.__len__():
            if left_phase_raw[i] != 0:
                first_left = i
                fix_first_left = True
            else:
                i += 1
        i = 0
        while not fix_first_right and i < phase_time.__len__():
            if right_phase_raw[i] != 0:
                first_right = i
                fix_first_right = True
            else:
                i += 1
        if first_left > 0:
            for i in range(first_left):
                left_phase_raw[i] = left_phase_raw[first_left]
        if first_right > 0:
            for i in range(first_right):
                right_phase_raw[i] = right_phase_raw[first_right]

    ###################################################################################
    # Calculate possible positions to be added to the paths
    # if N == 1 only instants with unused data are considered
    ###################################################################################
    def process_phase_data():
        N = 1
        fix_first_data_entries()
        delta_phase.append(left_phase_raw[0] - right_phase_raw[0])
        alt_delta_phase.append(left_phase_raw[0] - right_phase_raw[0] - 2 * sgn(delta_phase[-1]) * pi)
        consecutive_antenna_entries_counter = 0
        for idx in range(phase_time.__len__()):
            if idx > 0:
                if antenna[idx] == antenna[idx - 1]:
                    consecutive_antenna_entries_counter += 1
                else:
                    consecutive_antenna_entries_counter = 0
                if consecutive_antenna_entries_counter < N and phase_time[idx] - phase_time[idx-1] < lost_phase_time_threshold:
                    delta_phase.append(left_phase_raw[idx] - right_phase_raw[idx])
                    alt_delta_phase.append(left_phase_raw[idx] - right_phase_raw[idx] - 2 * sgn(delta_phase[-1]) * pi)
                    if abs(delta_phase[-1] * lamda / (4 * pi * L)) < 1:
                        currA = math.asin(delta_phase[-1] * lamda / (4 * pi * L))
                    else:
                        currA = sgn(delta_phase[-1]) * pi / 2
                    if abs(alt_delta_phase[-1] * lamda / (4 * pi * L)) < 1:
                        currB = math.asin(alt_delta_phase[-1] * lamda / (4 * pi * L))
                    else:
                        currB = sgn(alt_delta_phase[-1]) * pi / 2
                    if phase_time[idx] - phase_time[idx-1] < lost_phase_time_threshold:
                        evaluate_particles([currA, currB])
                        update_particles()
                else:
                    delta_phase.append(delta_phase[-1])
                    alt_delta_phase.append(alt_delta_phase[-1])

    # #######################################################################################################
    # read the available data and create the vectors:
    # left_phase_raw_data: Shows how the phase read by the left antenna develops through the data entries
    # right_phase_raw_data: Shows how the phase read by the right antenna develops through the data entries
    # phase_time: Shows how the time of each data entry
    # antenna: Shows which antenna was read for each data entry
    # #######################################################################################################
    def read_data():
        input_file = open(input_path, "r")
        input_lines = input_file.readlines()
        input_file.close()
        rssi = 0
        rssi_count = 0
        if file_to_be_cleared:
            input_file = open(input_path, "w")
            input_file.close()
        if input_lines:
            for line in input_lines:
                results = line.strip().split(', ')
                results[0] = results[0].replace('\x00', '')
                if results.__len__() > 4:
                    rssi = rssi + float(results[4])
                    rssi_count += 1
                    ant1No = float(results[1])
                    # sort each information line to the appropriate antenna list
                    if ant1No == ant1_no_left and int(results[2], 16) == EPC:
                        antenna.append(ant1No)
                        left_phase_raw.append((float(results[3]) - phase_DC))
                        phase_time.append(float(results[0]))
                        if right_phase_raw.__len__() > 0:
                            right_phase_raw.append(right_phase_raw[-1])
                        else:
                            right_phase_raw.append(0)
                    elif ant1No == ant1_no_right and int(results[2], 16) == EPC:
                        antenna.append(ant1No)
                        right_phase_raw.append(float(results[3]))
                        phase_time.append(float(results[0]))
                        if left_phase_raw.__len__() > 0:
                            left_phase_raw.append(left_phase_raw[-1])
                        else:
                            left_phase_raw.append(0)
            avg_rssi.append(rssi / rssi_count)
            avg_rssi.pop(0)
            if left_phase_raw.__len__() > 0 and right_phase_raw.__len__() > 0:
                return True
            else:
                return False
        else:
            return False

    if read_data():
        if phase_time.__len__() < 2:
            return [0.0, 0.0]
        create_particle_list()
        process_phase_data()
        if (density_counter[-1] / (phase_time[-1]-phase_time[0])) > density_threshold:
            print(density_counter[-1] / (phase_time[-1]-phase_time[0]))
            theta_estimation = make_estimation()
            if abs(theta_estimation) < 10*pi/180:
                print("Target found at: ", theta_estimation * 180 / pi, " degrees")
                return True, theta_estimation
            else:
                print("Not close enough")
                return False, 0
        else:
            print("No Target yet")
            return False, 0
    else:
        print("No Target yet")
        return False, 0


def target_finder(input_path, EPC_str):
    target_in_sight = False
    theta = 0
    while not target_in_sight:
        target_in_sight, theta = found(input_path, EPC_str, 1000, 100, 0.1, 0.3, 10)
        time.sleep(0.5)
    input_file = open(input_path, "w")
    input_file.close()
    return theta
