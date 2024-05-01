

import itertools
import numpy as np
from scipy.optimize import least_squares
import math
from math import pi
from scipy.stats.distributions import t




# %%
def estimate_location(tag):

    #print(tag.EPC)

    if tag.to_locate==1: #measurements>kati


        time, x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices = tag.get_all_unwrapped_measurements()

        #print(antenna_indices)

        #print("-------3D--------")
        #results_tag_best_combination, results_tag_all_combinations = localization_3D(x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices )
        results_tag_best_combination = []


        flag_good_estimation = evaluate_estimation(results_tag_best_combination,30)

        if flag_good_estimation==0:

            #print("-------2D--------")
            results_tag_best_combination, results_tag_all_combinations = localization_2D(x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices )

            flag_good_estimation = evaluate_estimation(results_tag_best_combination,30)


        if flag_good_estimation==0:

            #print("-------rough--------")

            time, x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices = tag.get_all_wrapped_measurements()

            results_tag_best_combination = localization_approximate(x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices)

            #print("best")
            #print(results_tag_best_combination)



    else:

            #print("-------rough--------")

            time, x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices = tag.get_all_wrapped_measurements()

            results_tag_best_combination = localization_approximate(x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices)



    tag=save_tag_location(tag,results_tag_best_combination)
    #tag.print_location()

    return tag




# %%
def localization_approximate(x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices):

    z_tag=find_z_tag_for_2D_localization(z_antenna,rssi,antenna_indices)
    x_tag, y_tag = approximate_tag_point(x_antenna,y_antenna,th_antenna,rssi,antenna_indices)

    results_tag_best_combination=[1,x_tag,y_tag,z_tag,float("nan"),float("nan"),float("nan")]

    return results_tag_best_combination


# %%
def save_tag_location(tag,results_tag_best_combination):

    tag.x=results_tag_best_combination[1]
    tag.y=results_tag_best_combination[2]
    tag.z=results_tag_best_combination[3]

    if tag.z < 0.0:
      tag.z = 30
    elif tag.z > 180:
      tag.z = 170

    tag.CI=results_tag_best_combination[5]+results_tag_best_combination[6]
    tag.reset_antennas_to_locate()


    return tag


# %%
def localization_3D(x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices):

    results_tag_best_combination, results_tag_all_combinations = estimate_for_all_combinations_3D(x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices)

    #results_tag_best_combination = choose_combination(results_tag_all_combinations)

    return results_tag_best_combination, results_tag_all_combinations




# %%
def localization_2D(x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices):


    z_tag=find_z_tag_for_2D_localization(z_antenna,rssi,antenna_indices)

    results_tag_best_combination, results_tag_all_combinations = estimate_for_all_combinations_2D(x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices,z_tag)

    #results_tag_best_combination = choose_combination(results_tag_all_combinations)

    return results_tag_best_combination, results_tag_all_combinations


# %%
def choose_combination(results_tag_all_combinations):

    results_tag_best_combination=np.empty((0,7))
    if len(results_tag_all_combinations):

        CI_x=results_tag_all_combinations[:,5]
        CI_y=results_tag_all_combinations[:,6]

        CI_sum=CI_x+CI_y
        #print(CI_sum)
        if not(np.isnan(CI_sum).all()):
            best_index=np.nanargmin(CI_sum)
            results_tag_best_combination = results_tag_all_combinations[best_index,:]


    return results_tag_best_combination


# %%
def approximate_tag_point(x_antenna,y_antenna,th_antenna,rssi,antenna_indices):

    tag_point=[]

    x=np.empty((0))
    y=np.empty((0))
    th=np.empty((0))
    r=np.empty((0))
    for comb in antenna_indices:
        x=np.hstack((x,x_antenna[comb]))
        y=np.hstack((y,y_antenna[comb]))
        th=np.hstack((th,th_antenna[comb]))
        r=np.hstack((r,abs(rssi[comb])))


    if len(r):
        min_index=np.argmin(r)

        direction_vector=np.matrix([[math.cos(th[min_index])],[math.sin(th[min_index])]])
        center_point=np.matrix([[x[min_index]],[y[min_index]]])
        tag_point=center_point+70*direction_vector

        return tag_point[0,0], tag_point[1,0]

    return [],[]


# %%
def evaluate_estimation(results_tag_best_combination,threshold):

    flag_good_estimation=1

    if len(results_tag_best_combination):

        CI_x=results_tag_best_combination[5]
        CI_y=results_tag_best_combination[6]
        rsquare=results_tag_best_combination[4]
        sign = results_tag_best_combination[0]

        if (np.isnan(rsquare)) or (np.isnan(CI_x)) or (np.isnan(CI_y)):
            flag_good_estimation=0
            return flag_good_estimation

        if (CI_x+CI_y>=threshold) or (CI_x>=3*threshold/4) or (CI_y>=3*threshold/4) or (rsquare<=0.75) or (sign==-1):
            flag_good_estimation=0
            return flag_good_estimation

    else:
        flag_good_estimation=0

    return flag_good_estimation


# %%
def estimate_for_all_combinations_3D(x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices):

    results_tag_all_combinations=np.empty((0,7))
    results_tag_best_combination=np.empty((0,7))

    #k=[all antennas, 2]
    #for k in range(len(antenna_indices),len(antenna_indices)-1,-1):
    for k in range(len(antenna_indices),1,-1):
        all_combos = np.array(list(itertools.combinations(antenna_indices, k)))

        results_tag_all_combinations=np.empty((0,7))

        #i=all possible combination of the specific number of antennas
        for i in range(0,len(all_combos)):

            start = find_start_point_3D(x_antenna,y_antenna,z_antenna,th_antenna,all_combos[i])

            param, rsquare, CI = nonlinear_fit_3D(x_antenna,y_antenna,z_antenna,phase,lamda,start,all_combos[i])

            if len(param):
                value, param[0], param[1] = evaluate_side(x_antenna,y_antenna,th_antenna,param[0],param[1],all_combos[i])

                results_tag = [value,param[0],param[1],param[2],rsquare,CI[0],CI[1]]
                results_tag_all_combinations=np.vstack((results_tag_all_combinations,results_tag))

        results_tag_best_combination = choose_combination(results_tag_all_combinations)
        flag_good_estimation = evaluate_estimation(results_tag_best_combination,20)

        #print(str(len(all_combos[i]))+" antennas")

        if flag_good_estimation:
            return results_tag_best_combination, results_tag_all_combinations

    return results_tag_best_combination, results_tag_all_combinations


# %%
def estimate_for_all_combinations_2D(x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices,z_tag):


    results_tag_all_combinations=np.empty((0,7))
    results_tag_best_combination=np.empty((0,7))
    #k=[all antennas, 1]
    #for k in range(len(antenna_indices),len(antenna_indices)-1,-1):
    for k in range(len(antenna_indices),0,-1):
        all_combos = np.array(list(itertools.combinations(antenna_indices, k)))

        results_tag_all_combinations=np.empty((0,7))

        #i=all possible combination of the specific number of antennas
        for i in range(0,len(all_combos)):

            start = find_start_point_2D(x_antenna,y_antenna,th_antenna,all_combos[i])

            param, rsquare, CI = nonlinear_fit_2D(x_antenna,y_antenna,z_antenna,phase,lamda,start,all_combos[i],z_tag)

            if len(param):
                value, param[0], param[1] = evaluate_side(x_antenna,y_antenna,th_antenna,param[0],param[1],all_combos[i])

                results_tag = [value,param[0],param[1],z_tag,rsquare,CI[0],CI[1]]
                results_tag_all_combinations=np.vstack((results_tag_all_combinations,results_tag))


        results_tag_best_combination = choose_combination(results_tag_all_combinations)
        flag_good_estimation = evaluate_estimation(results_tag_best_combination,15)

        #print(str(len(all_combos[i]))+" antennas")

        if flag_good_estimation:
            return results_tag_best_combination, results_tag_all_combinations

    return results_tag_best_combination, results_tag_all_combinations


# %%
def theoretical_3D(x,x_antenna,y_antenna,z_antenna,phase,lamda,combination):


    F=[]

    for i in range(0,len(combination)):


        fun= np.sqrt( (x[0]-x_antenna[combination[i]])**2 + (x[1]-y_antenna[combination[i]])**2 +(x[2]-z_antenna[combination[i]])**2)/lamda[combination[i]]*4*pi+ x[i+3] - phase[combination[i]]
        F=np.hstack((F,fun))

    return F


# %%
def theoretical_2D(x,x_antenna,y_antenna,z_antenna,phase,lamda,z_tag,combination):


    F=[]

    for i in range(0,len(combination)):

        fun= np.sqrt( (x[0]-x_antenna[combination[i]])**2 + (x[1]-y_antenna[combination[i]])**2 +(z_tag-z_antenna[combination[i]])**2)/lamda[combination[i]]*4*pi+ x[i+2] - phase[combination[i]]
        F=np.hstack((F,fun))

    return F


# %%
def find_start_point_3D(x_antenna,y_antenna,z_antenna,th_antenna,combination):

    trajectory_length=[]
    z=np.empty((0))
    for comb in combination:
        trajectory_length.append(len(x_antenna[comb]))
        z=np.hstack((z,z_antenna[comb]))

    max_index=trajectory_length.index(max(trajectory_length))

    theta=th_antenna[combination[max_index]]
    x=x_antenna[combination[max_index]]
    y=y_antenna[combination[max_index]]

    center_index=int(len(theta)/2)

    direction_vector=np.matrix([[math.cos(theta[center_index])],[math.sin(theta[center_index])]])
    center_point=np.matrix([[x[center_index]],[y[center_index]]])

    start_point=center_point+50*direction_vector

    start=[start_point[0,0],start_point[1,0],np.mean(z)]

    for comb in combination:
        start.append(-30)


    return start


# %%
def find_start_point_2D(x_antenna,y_antenna,th_antenna,combination):

    trajectory_length=[]
    for comb in combination:
        trajectory_length.append(len(x_antenna[comb]))


    max_index=trajectory_length.index(max(trajectory_length))

    theta=th_antenna[combination[max_index]]
    x=x_antenna[combination[max_index]]
    y=y_antenna[combination[max_index]]

    center_index=int(len(theta)/2)

    direction_vector=np.matrix([[math.cos(theta[center_index])],[math.sin(theta[center_index])]])
    center_point=np.matrix([[x[center_index]],[y[center_index]]])

    start_point=center_point+50*direction_vector

    start=[start_point[0,0],start_point[1,0]]

    for comb in combination:
        start.append(-30)

    return start


# %%
def evaluate_side(x_antenna,y_antenna,th_antenna,x_est,y_est,combination):

    #trajectory_length=[]
    for comb in combination:
        #trajectory_length.append(len(x_antenna[comb]))


    #max_index=trajectory_length.index(max(trajectory_length))

        theta=th_antenna[comb]
        x=x_antenna[comb]
        y=y_antenna[comb]

        center_index=int(len(theta)/2)

        direction_vector=np.matrix([[math.cos(theta[center_index])],[math.sin(theta[center_index])]])
        center_point=np.matrix([[x[center_index]],[y[center_index]]])
        est_point=np.matrix([[x_est],[y_est]])

        vector=est_point-center_point
        theta_est=np.arctan2(vector[1],vector[0])


        if abs(theta_est-theta[center_index])<=pi/2:
            value=1
        else:
            value=-1

            theta_est=theta_est+pi
            theta_est=theta[center_index]-(theta_est-theta[center_index])

            new_vector=np.matrix([[math.cos(theta_est)],[math.sin(theta_est)]])
            est_point=center_point+np.linalg.norm(vector)*new_vector

            x_est=est_point[0,0]
            y_est=est_point[1,0]

            return value, x_est, y_est

    return value, x_est, y_est


# %%
def nonlinear_fit_3D(x_antenna,y_antenna,z_antenna,phase,lamda,start,combination):


    param=least_squares(theoretical_3D,start,args=(x_antenna,y_antenna,z_antenna,phase,lamda,combination),method='trf')

    residuals=param.fun
    jac=param.jac
    #residuals=Ph_data-theoretical(XY_data, *popt)

    # calculate rsquare
    ss_res=np.sum(residuals**2)
    ss_tot=0
    for comb in combination:
        ss_tot=ss_tot+np.sum((phase[comb]-np.mean(phase[comb]))**2)

    rsquare=1-ss_res/ss_tot
#    print(rsquare)

    #Hessian matrix
    Hessian=np.dot(jac.T,jac)

    if (np.linalg.det(Hessian)==0):
        #print('det=0')
        return [],float("nan"),np.array([float("nan"),float("nan")])

    #degrees of freedom
    dof = len(residuals)-len(param)

    #variance of residuals
    sigma_resi = np.var(residuals)

    #vairance of coefficients
    sigma_cov=sigma_resi*np.linalg.inv(Hessian)

    #t-value
    alpha=0.05
    tval=t.ppf(1.0-alpha/2,dof)

    #confidence interval
    CI=2*tval*np.sqrt(np.diag(sigma_cov))

    return param.x, rsquare, CI[0:2]


# %%
def nonlinear_fit_2D(x_antenna,y_antenna,z_antenna,phase,lamda,start,combination,z_tag):


    param=least_squares(theoretical_2D,start,args=(x_antenna,y_antenna,z_antenna,phase,lamda,z_tag,combination),method='trf')

    residuals=param.fun
    jac=param.jac
    #residuals=Ph_data-theoretical(XY_data, *popt)

    # calculate rsquare
    ss_res=np.sum(residuals**2)
    ss_tot=0
    for comb in combination:
        ss_tot=ss_tot+np.sum((phase[comb]-np.mean(phase[comb]))**2)

    rsquare=1-ss_res/ss_tot
#    print(rsquare)

    #Hessian matrix
    Hessian=np.dot(jac.T,jac)

    if (np.linalg.det(Hessian)==0):
        #print('det=0')
        return [],float("nan"),np.array([float("nan"),float("nan")])

    #degrees of freedom
    dof = len(residuals)-len(param)

    #variance of residuals
    sigma_resi = np.var(residuals)

    #vairance of coefficients
    sigma_cov=sigma_resi*np.linalg.inv(Hessian)

    #t-value
    alpha=0.05
    tval=t.ppf(1.0-alpha/2,dof)

    #confidence interval
    CI=2*tval*np.sqrt(np.diag(sigma_cov))

    return param.x, rsquare, CI[0:2]


# %%
def find_z_tag_for_2D_localization(z_antenna,rssi,antenna_indices):

    z_tag = []

    z=np.empty((0))
    r=np.empty((0))
    for comb in antenna_indices:
        z=np.hstack((z,z_antenna[comb]))
        r_min=min(abs(rssi[comb]))
        r=np.hstack((r,r_min*np.ones((z_antenna[comb].shape))))

    if len(z):
        z_tag=np.average(z, weights = (1/r)**4)

    return z_tag


# %%


