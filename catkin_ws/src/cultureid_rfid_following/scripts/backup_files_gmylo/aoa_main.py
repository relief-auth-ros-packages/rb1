def track(input_path, EPC_str, past_estimation):
    import time
    import math
    import matplotlib.pyplot as plt

    #################################################################
    # Settings
    EPC = int(EPC_str, 16)
    ant1_no_left = 2
    ant1_no_right = 3
    lost_tag_time_threshold = 5
    lost_phase_time_threshold = 0.025
    phaseDC = -1.125
    frequency = 867.5 * (10 ** 6)
    lamda = 299792458 / frequency
    L = 0.13

    #################################################################
    # Global Variables
    initial_theta = past_estimation
    unprocessedEstimations = []
    unprocessedEstimations_time = []
    estimations = []
    estimations_time = []
    altEstimations = []
    leftPhaseRaw = []
    rightPhaseRaw = []
    leftPhaseProcessed = []
    rightPhaseProcessed = []
    delta_phase = []
    alt_delta_phase = []
    phase_time = []
    antenna = []
    pathA = []
    pathB = []
    pathA_processed = []
    pathB_processed = []
    pathA_filtered = []
    pathB_filtered = []
    path_time = []
    path_insertion = []
    pi = math.pi
    rssi = [0]
    N = 5
    N_flip = 5
    sg_11 = [-36, 9, 44, 69, 84, 89, 84, 69, 44, 9, -36]
    #################################################################
    def make_projection(path, path_t, curr_time):
        if path.__len__() > N and path_t.__len__() > N:
            dth = 0
            for i in range(N - 1):
                dth = (path[-i] - path[-i - 1])
            dth = dth / (N - 1)
            projection = path[-1] + dth
        else:
            projection = path[-1]
        return projection

    #################################################################
    # phase lost
    def lost_phase(curr_time):
        if estimations.__len__() > N:
            estimations.append(make_projection(estimations, estimations_time, curr_time))
            estimations_time.append(curr_time)
            plt.plot(estimations, 'o-')
            plt.plot(estimations.__len__() - 1, estimations[-1], 'o')
            plt.show()

    ##################################################################
    # read input.txt for any new lines
    def read_phase():
        left_read = False
        right_read = False
        input_file = open(input_path, "r")
        input_lines = input_file.readlines()
        input_file.close()
        input_file = open(input_path, "w")
        if input_lines:
            idx = 0
            print("No of inputs: ", input_lines.__len__())
            for line in input_lines:
                idx += 1
                if idx-2 > input_lines.__len__():
                    input_file.write(line)
                results = line.strip().split(', ')
                if results.__len__() > 4:
                    results[0] = results[0].replace('\x00', '')
                    ant1No = float(results[1])
                    antenna.append(ant1No)
                    rssi.append(rssi[-1] + float(results[4]))
                    rssi.pop(0)
                    # sort each information line to the appropriate antenna list
                    if ant1No == ant1_no_left and int(results[2], 16) == EPC:
                        leftPhaseRaw.append((float(results[3]) - phaseDC))
                        phase_time.append(float(results[0]))
                        if rightPhaseRaw.__len__() > 0:
                            rightPhaseRaw.append(rightPhaseRaw[-1])
                        else:
                            rightPhaseRaw.append(0)
                    elif ant1No == ant1_no_right and int(results[2], 16) == EPC:
                        rightPhaseRaw.append(float(results[3]))
                        phase_time.append(float(results[0]))
                        if leftPhaseRaw.__len__() > 0:
                            leftPhaseRaw.append(leftPhaseRaw[-1])
                        else:
                            leftPhaseRaw.append(0)

            rssi.append(rssi[-1] / len(phase_time))
            rssi.pop(0)
            input_file.close()
            return True
        else:
            print("No imput")
            input_file.close()
            return False

    ##################################################################################################
    def process_phase_data():
        fix_first_left = False
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
        fix_first_right = False
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
        antenna_counter = 0
        delta_phase.append(leftPhaseRaw[0] - rightPhaseRaw[0])
        if delta_phase[0] > 0:
            lock = -1
        else:
            lock = 1
        alt_delta_phase.append(leftPhaseRaw[0] - rightPhaseRaw[0] + 2 * lock * pi)
        if abs(delta_phase[-1] * lamda / (4 * pi * L)) < 1 and abs(alt_delta_phase[-1] * lamda / (4 * pi * L)) < 1:
            pathA.append(delta_phase[-1] * lamda / (4 * pi * L))
            pathB.append(alt_delta_phase[-1] * lamda / (4 * pi * L))
        elif abs(delta_phase[-1] * lamda / (4 * pi * L)) < 1:
            pathA.append(delta_phase[-1] * lamda / (4 * pi * L))
            pathB.append(lock*pi/2)
        elif abs(alt_delta_phase[-1] * lamda / (4 * pi * L)) < 1:
            pathA.append(alt_delta_phase[-1] * lamda / (4 * pi * L))
            pathB.append(-lock * pi / 2)
        for idx in range(antenna.__len__()):
            if idx > 0:
                if antenna[idx] == antenna[idx-1]:
                    antenna_counter += 1
                else:
                    antenna_counter = 0
                if antenna_counter < N_flip:
                    delta_phase.append(leftPhaseRaw[idx] - rightPhaseRaw[idx])
                    if delta_phase[idx] > 0:
                        lock = -1
                    else:
                        lock = 1
                    alt_delta_phase.append(leftPhaseRaw[idx] - rightPhaseRaw[idx] + 2 * lock * pi)
                    if abs(delta_phase[-1] * lamda / (4 * pi * L)) < 1:
                        pathA.append(math.asin(delta_phase[-1] * lamda / (4 * pi * L)))
                    elif abs((delta_phase[-1] - 4*lock*pi) * lamda / (4 * pi * L)) < 1:
                        pathA.append(math.asin((delta_phase[-1] - 4*lock*pi) * lamda / (4 * pi * L)))
                    else:
                        pathA.append(-lock * pi / 2)
                    if abs(alt_delta_phase[-1] * lamda / (4 * pi * L)) < 1:
                        pathB.append(math.asin(alt_delta_phase[-1] * lamda / (4 * pi * L)))
                    else:
                        pathB.append(lock * pi / 2)
                else:
                    delta_phase.append(delta_phase[-1])
                    alt_delta_phase.append(alt_delta_phase[-1])
                    pathA.append(pathA[-1])
                    pathB.append(pathB[-1])


    ##################################################################################################
    def process_paths():
        pathA_processed.append(pathA[0])
        pathB_processed.append(pathB[0])
        pathA_filtered.append(pathA[0])
        pathB_filtered.append(pathB[0])
        markerA = 0
        markerB = 0
        for idx in range(pathA.__len__()):
            if abs(pathA[idx]) == pi/2:
                th = pathA[idx]
                pathA[idx] = pathB[idx]
                pathB[idx] = th
        for idx in range(pathA.__len__()):
            if 0 < idx < N+1:
                if abs(pathA[idx] - pathA[idx - 1]) > abs(pathB[idx] - pathA[idx - 1]):
                    th = pathA[idx]
                    pathA[idx] = pathB[idx]
                    pathB[idx] = th
            if idx > N:
                if abs(pathA[idx] - pathA[idx - 1]) > abs(pathB[idx] - pathA[idx - 1]) \
                        and abs(pathB[idx] - pathA[idx - 1]) < 0.2:
                    th = pathA[idx]
                    pathA[idx] = pathB[idx]
                    pathB[idx] = th
                else:
                    pA = 0.2*(pathA[idx - 1]+pathA[idx - 2]+pathA[idx - 3]+pathA[idx - 4]+pathA[idx - 5])
                    if abs(pathA[idx] - pA) > abs(pathB[idx] - pA):
                        th = pathA[idx]
                        pathA[idx] = pathB[idx]
                        pathB[idx] = th
        pathA_jump_marker = 0
        pathB_jump_marker = 0
        for idx in range(pathA.__len__()):
            if 0 < idx < pathA.__len__()-3:
                if abs(pathA[idx] - pathA[idx - 1]) > pi / 4:
                    pathA_filtered.append(pathA[idx])
                    pathA_jump_marker = idx
                elif idx - pathA_jump_marker < ((N + 1) / 2):
                    pathA_filtered.append(pathA[idx])
                elif antenna.__len__() - idx < ((N + 1) / 2):
                    pathA_filtered.append(pathA[idx])
                elif abs(pathA[idx + 1] - pathA[idx]) > pi / 4 or abs(pathA[idx + 2] - pathA[idx + 1]) > pi / 4:
                    pathA_filtered.append(pathA[idx])
                else:
                    flt = 0
                    for j in range(N):
                        flt = flt + pathA[idx + j - int((N - 1) / 2)]
                    flt = flt / N
                    pathA_filtered.append(flt)
                if abs(pathB[idx] - pathB[idx - 1]) > pi / 4:
                    pathB_filtered.append(pathB[idx])
                    pathB_jump_marker = idx
                elif idx - pathB_jump_marker < (N / 2):
                    pathB_filtered.append(pathB[idx])
                elif antenna.__len__() - idx < ((N + 1) / 2):
                    pathB_filtered.append(pathB[idx])
                elif abs(pathB[idx + 1] - pathB[idx]) > pi / 4 or abs(pathB[idx + 2] - pathB[idx + 1]) > pi / 4:
                    pathB_filtered.append(pathB[idx])
                else:
                    flt = 0
                    for j in range(N):
                        flt = flt + pathB[idx + j - int((N - 1) / 2)]
                    flt = flt / N
                    pathB_filtered.append(flt)
    ##################################################################################################
    def filter_paths():
        N = 11
        pathA_jump = False
        pathB_jump = False
        for idx in range(pathA_filtered.__len__()):
            pathA_jump = False
            pathB_jump = False
            if (idx + N) < pathA_filtered.__len__():
                for j in range(N):
                    if abs(pathA_filtered[idx] - pathA_filtered[idx+1]) > pi/2:
                        pathA_jump = True
                if not pathA_jump:
                    flt = 0
                    for j in range(N):
                        flt = flt + sg_11[j] * pathA_filtered[idx + j]
                    flt = flt / 429
                    pathA_filtered[idx] = flt
                else:
                    pathA_jump = False
                    for j in range(5):
                        if abs(pathA_filtered[idx] - pathA_filtered[idx + 1]) > pi / 2:
                            pathA_jump = True
                    if not pathA_jump:
                        flt = 0
                        for j in range(5):
                            flt = flt + pathA_filtered[idx + j]
                        flt = flt / 5
                        pathA_filtered[idx] = flt
            if (idx + N) < pathB_filtered.__len__():
                for j in range(N):
                    if abs(pathB_filtered[idx] - pathB_filtered[idx+1]) > pi/2:
                        pathB_jump = True
                if not pathB_jump:
                    flt = 0
                    for j in range(N):
                        flt = flt + sg_11[j] * pathA_filtered[idx + j]
                    flt = flt / 429
                    pathA_filtered[idx] = flt
                else:
                    pathB_jump = False
                    for j in range(5):
                        if abs(pathB_filtered[idx] - pathB_filtered[idx + 1]) > pi / 2:
                            pathB_jump = True
                    if not pathB_jump:
                        flt = 0
                        for j in range(5):
                            flt = flt + pathB_filtered[idx + j]
                        flt = flt / 5
                        pathB_filtered[idx] = flt

    ##################################################################################################
    def make_estimations():
        if abs(initial_theta - pathA_filtered[0]) < abs(initial_theta - pathB_filtered[0]):
            unprocessedEstimations.append(pathA_filtered[0])
            altEstimations.append(pathB_filtered[0])
        else:
            unprocessedEstimations.append(pathB_filtered[0])
            altEstimations.append(pathA_filtered[0])
        for idx in range(pathA_filtered.__len__()):
            if idx > 0:
                if abs(unprocessedEstimations[idx - 1] - pathA_filtered[idx]) < abs(
                        unprocessedEstimations[idx - 1] - pathB_filtered[idx]):
                    unprocessedEstimations.append(pathA_filtered[idx])
                    altEstimations.append(pathB_filtered[idx])
                else:
                    unprocessedEstimations.append(pathB_filtered[idx])
                    altEstimations.append(pathA_filtered[idx])

    ##################################################################################################
    def process_estimations():
        N = 21
        for idx in range(unprocessedEstimations.__len__()):
            if ((N - 2) / 2) < idx < (unprocessedEstimations.__len__() - (N / 2)):
                flt = 0
                for j in range(N):
                    flt = flt + unprocessedEstimations[idx + j - int((N - 1) / 2)]
                flt = flt / N
                estimations.append(flt)
            elif idx < ((N - 2) / 2):
                flt = 0
                for j in range(idx+1):
                    flt = flt + unprocessedEstimations[j]
                flt = flt / (idx+1)
                estimations.append(flt)
            else:
                flt = 0
                for j in range(unprocessedEstimations.__len__()-idx):
                    flt = flt + unprocessedEstimations[-j-1]
                flt = flt / (unprocessedEstimations.__len__()-idx)
                estimations.append(flt)

    ##################################################################################################
    def fix_estimations():
        fixed_start = False
        marker = 0
        while not fixed_start and (marker+1) < altEstimations.__len__():
            if abs(altEstimations[marker]) < 0.45*pi:
                fixed_start = True
            else:
                marker += 1
        if fixed_start:
            for idx in range(marker):
                altEstimations[idx] = altEstimations[marker]

    ################################################################################################
    if read_phase():
        print(phase_time.__len__())
        process_phase_data()
        process_paths()
        filter_paths()
        make_estimations()
        process_estimations()
    print(rssi[-1])
    if len(estimations) > 0 and len(altEstimations) == 0:
        return [estimations[-1], 0, rssi[-1]]
    if len(estimations) == 0 and len(altEstimations) > 0:
        return [0, altEstimations[-1], rssi[-1]]
    if len(estimations) > 0 and len(altEstimations) > 0:
        return [estimations[-1], altEstimations[-1], rssi[-1]]
    return [0,0, rssi[-1]]
