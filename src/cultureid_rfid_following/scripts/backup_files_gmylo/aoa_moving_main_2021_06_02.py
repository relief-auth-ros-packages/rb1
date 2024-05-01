def track(input_path, bank_path, EPC_str, last_estimation, past_estimation, antenna_poses):

    import math
    import matplotlib.pyplot as plt

    ###################################################################################
    # Settings
    # EPC, antenna numbers and frequency should be compatible with input file.txt
    # Phase DC to to wiring to be determined experimentally
    # Distance between antennas (L) should change if antennas are moved
    ###################################################################################
    EPC = int(EPC_str, 16)
    ant1_no_left = 2
    ant1_no_right = 3
    phaseDC = 0.0
    frequency = 867.5 * (10 ** 6)
    lamda = 299792458 / frequency
    L = 0.13

    ###################################################################################
    # Global Variables
    #
    ###################################################################################
    initial_theta = past_estimation
    last_theta = last_estimation
    leftPhaseRaw = []
    rightPhaseRaw = []
    delta_phase = []
    alt_delta_phase = []
    phase_time = []
    antenna = []
    pathA = []
    pathB = []
    pathA_filtered = []
    pathB_filtered = []
    pi = math.pi
    lost_phase_time_threshold = 0.15
    avg_rssi = [0]
    data_window = 5000
    timestamp = [0]
    theta_start = [0.0]
    sg_11 = [-36, 9, 44, 69, 84, 89, 84, 69, 44, 9, -36]

    ###################################################################################
    # get the sign of the point
    ###################################################################################
    def sgn(point):
        if point < 0:
            return -1
        else:
            return 1

    ###################################################################################
    # find which combination of "path exchanges" is the most likely
    ###################################################################################
    def find_min_path(aa, ab, ba, bb):
        scenarios = [aa + bb, ab + ba]
        shortest = [scenarios[0], 0]
        for idx in range(scenarios.__len__()):
            if scenarios[idx] < shortest[0]:
                shortest.pop(0)
                shortest.pop(0)
                shortest.append(scenarios[idx])
                shortest.append(idx)
        return int(shortest[1])

    ###################################################################################
    # add points to paths
    ###################################################################################
    def add_paths(new_A, new_B):
        pathA.append(new_A)
        pathB.append(new_B)

    ###################################################################################
    # find which combination of "path exchanges" is the most likely
    ###################################################################################
    def flip_paths(scenario, idx):
        if scenario == 1:
            pA = pathA_filtered[idx]
            pathA_filtered[idx] = pathB_filtered[idx]
            pathB_filtered[idx] = pA

    ###################################################################################
    # short new points to the appropriate path
    # flag == 0 -> do not add points to path
    # flag == 1 -> add points to path
    ###################################################################################
    def short_points_to_path(flag, prev_point_A, prev_point_B, curr_point_A, curr_point_B):
        aa = abs(dist_to(prev_point_A, curr_point_A))
        ab = abs(dist_to(prev_point_A, curr_point_B))
        ba = abs(dist_to(prev_point_B, curr_point_A))
        bb = abs(dist_to(prev_point_B, curr_point_B))
        shortest = find_min_path(aa, ab, ba, bb)
        if flag == 1:
            if shortest == 0:
                add_paths(curr_point_A, curr_point_B)
            elif shortest == 1:
                add_paths(curr_point_B, curr_point_A)
        return shortest

    ###################################################################################
    # shortest distance to point
    ###################################################################################
    def dist_to(prev_point, curr_point):
        if abs(prev_point - curr_point) < abs(prev_point + pi - curr_point) and abs(prev_point - curr_point) < abs(
                prev_point - pi - curr_point):
            return -(prev_point - curr_point)
        elif abs(prev_point - curr_point) > abs(prev_point + pi - curr_point) and abs(
                prev_point + pi - curr_point) < abs(prev_point - pi - curr_point):
            return -(prev_point + pi - curr_point)
        else:
            return -(prev_point - pi - curr_point)

    ###################################################################################
    # If consecutive point in path are below threshold, it is likely correct
    ###################################################################################
    def path_looks_right(prev_point, curr_point):
        threshold = 0.5
        if abs(dist_to(prev_point, curr_point)) < threshold:
            return True
        else:
            return False

    ###################################################################################
    # read input.txt for any new lines & clear the file
    # create phase-time vectors & calculate mean RSSI
    ###################################################################################
    def read_phase():
        input_file = open(input_path, "r")
        input_lines = input_file.readlines()
        input_file.close()
        bank_file = open(bank_path, "r")
        bank_lines = bank_file.readlines()
        bank_file.close()
        rssi = 0
        rssi_count = 0
        rssi_N = 100
        timestamp_check = False
        input_file = open(input_path, "w")
        bank_file = open(bank_path, "w")
        idx = 0
        if bank_lines:
            print(bank_lines.__len__(), " lines in bank")
            for b_line in bank_lines:
                if (bank_lines.__len__() - idx) + input_lines.__len__() < data_window:
                    if not timestamp_check:
                        timestamp_check = True
                        timestamp.append(idx)
                        timestamp.pop(0)
                    bank_file.write(b_line)
                idx += 1
                results = b_line.strip().split(', ')
                if results.__len__() > 4:
                    results[0] = results[0].replace('\x00', '')
                    ant1No = float(results[1])
                    # sort each information line to the appropriate antenna list
                    if ant1No == ant1_no_left and int(results[2], 16) == EPC:
                        antenna.append(ant1No)
                        leftPhaseRaw.append((float(results[3]) - phaseDC))
                        phase_time.append(float(results[0]))
                        if rightPhaseRaw.__len__() > 0:
                            rightPhaseRaw.append(rightPhaseRaw[-1])
                        else:
                            rightPhaseRaw.append(0)
                    elif ant1No == ant1_no_right and int(results[2], 16) == EPC:
                        antenna.append(ant1No)
                        rightPhaseRaw.append(float(results[3]))
                        phase_time.append(float(results[0]))
                        if leftPhaseRaw.__len__() > 0:
                            leftPhaseRaw.append(leftPhaseRaw[-1])
                        else:
                            leftPhaseRaw.append(0)
        if input_lines:
            print("No of inputs: ", input_lines.__len__())
            for line in input_lines:
                bank_file.write(line)
                results = line.strip().split(', ')
                if results.__len__() > 4:
                    rssi = rssi + float(results[4])
                    rssi_count += 1
                    results[0] = results[0].replace('\x00', '')
                    ant1No = float(results[1])
                    # sort each information line to the appropriate antenna list
                    if ant1No == ant1_no_left and int(results[2], 16) == EPC:
                        antenna.append(ant1No)
                        leftPhaseRaw.append((float(results[3]) - phaseDC))
                        phase_time.append(float(results[0]))
                        if rightPhaseRaw.__len__() > 0:
                            rightPhaseRaw.append(rightPhaseRaw[-1])
                        else:
                            rightPhaseRaw.append(0)
                    elif ant1No == ant1_no_right and int(results[2], 16) == EPC:
                        antenna.append(ant1No)
                        rightPhaseRaw.append(float(results[3]))
                        phase_time.append(float(results[0]))
                        if leftPhaseRaw.__len__() > 0:
                            leftPhaseRaw.append(leftPhaseRaw[-1])
                        else:
                            leftPhaseRaw.append(0)
            input_file.close()
            bank_file.close()
            avg_rssi.append(rssi / rssi_count)
            avg_rssi.pop(0)
            if leftPhaseRaw.__len__() > 0 and rightPhaseRaw.__len__() > 0:
                print("Input read")
                return True
            else:
                bank_file = open(bank_path, "w")
                bank_file.close()
                print("Not enough input")
                return False
        else:
            print("####################################################################################\nTag Lost\n####################################################################################\n")
            # print("Tag Lost")
            input_file.close()
            bank_file.close()
            bank_file = open(bank_path, "w")
            bank_file.close()
            print("No input")
            return False

    ###################################################################################
    # change first data entries for the antenna that was not read first, to be equal to
    # first actual data and not 0
    ###################################################################################
    def fix_first_data_entries():
        fix_first_left = False
        fix_first_right = False
        first_left = 0
        first_right = 0
        i = 0
        while not fix_first_left and i < phase_time.__len__():
            if leftPhaseRaw[i] != 0:
                first_left = i
                fix_first_left = True
            else:
                i += 1
        i = 0
        while not fix_first_right and i < phase_time.__len__():
            if rightPhaseRaw[i] != 0:
                first_right = i
                fix_first_right = True
            else:
                i += 1
        if first_left > 0:
            for i in range(first_left):
                leftPhaseRaw[i] = leftPhaseRaw[first_left]
        if first_right > 0:
            for i in range(first_right):
                rightPhaseRaw[i] = rightPhaseRaw[first_right]

    ###################################################################################
    # short the first entries to the appropriate path
    ###################################################################################
    def find_first_path_entries():
        delta_phase.append(leftPhaseRaw[0] - rightPhaseRaw[0])
        alt_delta_phase.append(leftPhaseRaw[0] - rightPhaseRaw[0] - 2 * sgn(delta_phase[-1]) * pi)
        if abs(delta_phase[-1] * lamda / (4 * pi * L)) < 1:
            currA = math.asin(delta_phase[-1] * lamda / (4 * pi * L))
        else:
            currA = sgn(delta_phase[-1]) * pi / 2
        if abs(alt_delta_phase[-1] * lamda / (4 * pi * L)) < 1:
            currB = math.asin(alt_delta_phase[-1] * lamda / (4 * pi * L))
        else:
            currB = sgn(alt_delta_phase[-1]) * pi / 2
        pastA = initial_theta
        if abs((((math.sin(initial_theta) * 4 * pi * L) - 2 * sgn(initial_theta) * pi) / (4 * pi * L))) < 1:
            pastB = math.asin((((math.sin(initial_theta) * 4 * pi * L) - 2 * pi) / (4 * pi * L)))
        else:
            pastB = sgn(
                (((math.sin(initial_theta) * 4 * pi * L) - 2 * sgn(initial_theta) * pi) / (4 * pi * L))) * pi / 2
        short_points_to_path(1, pastA, pastB, currA, currB)

    ###################################################################################
    # Calculate possible positions to be added to the paths
    # if N == 1 only instants with unused data are considered
    ###################################################################################
    def process_phase_data():
        N = 1
        fix_first_data_entries()
        find_first_path_entries()
        consecutive_antenna_entries_counter = 0
        for idx in range(phase_time.__len__()):
            if idx > 0:
                pathA_filtered.append(pathA[-1])
                pathB_filtered.append(pathB[-1])
                if antenna[idx] == antenna[idx - 1]:
                    consecutive_antenna_entries_counter += 1
                else:
                    consecutive_antenna_entries_counter = 0
                if consecutive_antenna_entries_counter < N and phase_time[idx] - phase_time[idx-1] < lost_phase_time_threshold:
                    delta_phase.append(leftPhaseRaw[idx] - rightPhaseRaw[idx])
                    alt_delta_phase.append(leftPhaseRaw[idx] - rightPhaseRaw[idx] - 2 * sgn(delta_phase[-1]) * pi)
                    if abs(delta_phase[-1] * lamda / (4 * pi * L)) < 1:
                        currA = math.asin(delta_phase[-1] * lamda / (4 * pi * L))
                    else:
                        currA = sgn(delta_phase[-1]) * pi / 2
                    if abs(alt_delta_phase[-1] * lamda / (4 * pi * L)) < 1:
                        currB = math.asin(alt_delta_phase[-1] * lamda / (4 * pi * L))
                    else:
                        currB = sgn(alt_delta_phase[-1]) * pi / 2
                    short_points_to_path(1, pathA[-1], pathB[-1], currA, currB)
                else:
                    delta_phase.append(delta_phase[-1])
                    alt_delta_phase.append(alt_delta_phase[-1])
                    pathA.append(pathA[-1])
                    pathB.append(pathB[-1])
        pathA_filtered.append(pathA[-1])
        pathB_filtered.append(pathB[-1])

    ###################################################################################
    #
    #
    ###################################################################################
    def process_paths():
        for idx in range(pathA.__len__()):
            if pi/2 - abs(pathA_filtered[idx]) < 0.05:
                flip_paths(1, idx)
        for idx in range(1, pathA.__len__()):
            if abs(dist_to(pathA_filtered[idx-1], pathA_filtered[idx])) > abs(dist_to(pathB_filtered[idx-1], pathA_filtered[idx])):
                flip_paths(1, idx)
        for idx in range(pathA.__len__()):
            if pi/2 - abs(pathA_filtered[idx]) < 0.05:
                flip_paths(1, idx)

    ###################################################################################
    #
    #
    ###################################################################################
    def filter_paths():
        N = 11
        path_A_marker = 0
        path_B_marker = 0
        for idx in range(1, pathA.__len__()):
            if not path_looks_right(pathA_filtered[idx-1], pathA_filtered[idx]):
                path_A_marker = idx
            if not path_looks_right(pathB_filtered[idx-1], pathB_filtered[idx]):
                path_B_marker = idx
            if path_A_marker + N < idx:
                local_A = []
                for j in range(N):
                    local_A.append(pathA_filtered[idx] + dist_to(pathA_filtered[idx], pathA_filtered[idx - j]))
                flt_A = 0
                for j in range(N):
                    flt_A = flt_A + local_A[j]
                flt_A = flt_A / N
                if abs(flt_A) > pi/2:
                    flt_A = sgn(flt_A)*pi/2
                pathA_filtered[idx] = flt_A
            else:
                local_A = []
                n = idx + 1 - path_A_marker
                for j in range(n):
                    local_A.append(pathA_filtered[idx] + dist_to(pathA_filtered[idx], pathA_filtered[idx - j]))
                flt_A = 0
                for j in range(n):
                    flt_A = flt_A + local_A[j]
                flt_A = flt_A / n
                if abs(flt_A) > pi/2:
                    flt_A = sgn(flt_A)*pi/2
                pathA_filtered[idx] = flt_A
            if path_B_marker + N < idx:
                local_B = []
                for j in range(N):
                    local_B.append(pathB_filtered[idx] + dist_to(pathB_filtered[idx], pathB_filtered[idx - j]))
                flt_B = 0
                for j in range(N):
                    flt_B = flt_B + local_B[j]
                flt_B = flt_B / N
                if abs(flt_B) > pi/2:
                    flt_B = sgn(flt_B)*pi/2
                pathB_filtered[idx] = flt_B
            else:
                local_B = []
                n = idx + 1 - path_B_marker
                for j in range(n):
                    local_B.append(pathB_filtered[idx] + dist_to(pathB_filtered[idx], pathB_filtered[idx - j]))
                flt_B = 0
                for j in range(n):
                    flt_B = flt_B + local_B[j]
                flt_B = flt_B / n
                if abs(flt_B) > pi/2:
                    flt_B = sgn(flt_B)*pi/2
                pathB_filtered[idx] = flt_B

    ###################################################################################
    #
    #
    ###################################################################################
    if read_phase():
        # print(phase_time.__len__())
        process_phase_data()
        filter_paths()
        process_paths()
        # plt.plot(phase_time, pathA_processed, 'k')
        # plt.plot(estimations, 'k')
        # plt.plot(altEstimations, 'ro')
        # plt.plot(phase_time, pathA, 'o')
        # plt.plot(phase_time, pathB, 'o')
        # plt.plot(phase_time, pathA_processed, 'o')
        # plt.plot(phase_time, pathB_processed, 'o')
        # plt.plot(phase_time, pathC_processed, 'o')
        # plt.plot(phase_time, pathA, 'ro')
        # plt.plot(phase_time, pathB, 'bo')
        # plt.plot(phase_time, pathA_filtered, 'o')
        # plt.plot(phase_time, pathB_filtered, 'o')
        # plt.grid(True)
        # plt.ylim(-1.6, 1.6)
        # plt.show()
        if pathA_filtered.__len__() > 0 and pathB_filtered.__len__() > 0:
            print("theta is: ", pathA_filtered[-1]*180/pi, " degrees")
            if pathA_filtered.__len__() > timestamp[-1]:
                if abs(last_theta - pathA_filtered[-1]) < abs(last_theta - pathB_filtered[-1]):
                    return [pathA_filtered[-1], pathB_filtered[-1], avg_rssi[-1], pathA_filtered[timestamp[-1]]]
                else:
                    return [pathB_filtered[-1], pathA_filtered[-1], avg_rssi[-1], pathA_filtered[timestamp[-1]]]
            else:
                return [pathA_filtered[-1], pathB_filtered[-1], avg_rssi[-1], 0]
    else:
        return [0, 0, 0, 0]
