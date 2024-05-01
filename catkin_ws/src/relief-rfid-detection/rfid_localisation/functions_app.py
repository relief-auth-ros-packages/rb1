import numpy as np
import os 
import pandas
import math
import itertools
from scipy.optimize import least_squares
import math
from math import pi
from scipy.stats.distributions import t

def dot(v,w):
    x,y = v
    X,Y = w
    return x*X + y*Y 

def length(v):
    x,y = v
    return math.sqrt(x*x + y*y)

def vector(b,e):
    x,y = b
    X,Y = e
    return (X-x, Y-y)

def unit(v):
    x,y = v
    mag = length(v)
    return (x/mag, y/mag)

def distance(p0,p1):
    return length(vector(p0,p1))

def scale(v,sc):
    x,y = v
    return (x * sc, y * sc)

def add(v,w):
    x,y = v
    X,Y = w
    return (x+X, y+Y)

def pnt2line(pnt, start, end):
    line_vec = vector(start, end)
    pnt_vec = vector(start, pnt)
    line_len = length(line_vec)
    line_unitvec = unit(line_vec)
    pnt_vec_scaled = scale(pnt_vec, 1.0/line_len)
    t = dot(line_unitvec, pnt_vec_scaled)    
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    nearest = scale(line_vec, t)
    dist = distance(nearest, pnt_vec)
    nearest = add(nearest, start)
    return dist

def min_point(a,b,c,x0,y0):

    #b=1
    #a=-slope
    #c=-intercept
    x = (b*(b*x0-a*y0)-a*c)/(a**2+b**2)
    y = (a*(-b*x0+a*y0)-b*c)/(a**2+b**2)

    return x,y

def estimate_location_for_app(tag,storage_coords):
    
    
    if len(storage_coords):
        
        if tag.x:
            time, x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices = tag.get_all_wrapped_measurements()

            storage = choose_storage(tag, storage_coords)

            z_tag = find_z_tag_for_app(z_antenna,rssi,antenna_indices)
            #print("z before: "+ str(z_tag))
            z_tag = choose_fixed_height(z_tag,storage)
            tag.z=z_tag

            #print("z after: " +str(z_tag))

            #---------------1st version-----------------

            #time, x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices = tag.get_all_unwrapped_measurements()
            #results_tag_best_combination, results_tag_all_combinations = localization_for_app(x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices,z_tag,storage)
            #tag = save_tag_location_for_app(tag,results_tag_best_combination)

            #--------------2nd version---------------
            tag = localization_for_app_projection(storage,tag)



    return tag



def localization_for_app_projection(storage,tag):    
    tag.x, tag.y = min_point(-storage[4],1,-storage[5],tag.x,tag.y)
    #tag = save_tag_location_for_app(x_tag,y_tag,z_tag)
    return tag



def choose_storage(tag,storage_coords):
    
    dist=np.empty([len(storage_coords),1])
    
    tag_coords = np.array([tag.x,tag.y])

    for i in range(0,len(storage_coords)):
        dist[i] = pnt2line(tag_coords,storage_coords[i,0:2],storage_coords[i][2:4])
        #print(dist[i])

    shortest_index = np.argmin(dist)
    #print(shortest_index)

    storage = storage_coords[shortest_index]

    return storage



def choose_fixed_height(z_tag,storage):

    diff_z = abs(z_tag-np.array([x for x in storage[6:] if x>=0]))
    #print(diff_z)
    min_index = np.nanargmin(diff_z)
    #print(min_index)

    return storage[6+min_index]




def get_storage_coordinates(storage_spaces_file):

    filename=storage_spaces_file
    if (os.path.exists(filename)) and ( not (os.stat(filename).st_size == 0)):
            
        storage = pandas.read_csv(filename, sep=",", header=None)
        storage=storage.to_numpy()

        if storage[0,0]==1:
            slopes = (storage[1:,3]-storage[1:,1])/(storage[1:,2]-storage[1:,0])
            intercepts = -slopes*storage[1:,0] + storage[1:,1]
            heights = storage[1:,4:]


            storage_coords = np.vstack((storage[1:,0:4].T,slopes,intercepts,heights.T)).T

        else:

            storage_coords=[]

        return storage_coords
    
    else:
        print('storage-coordinates file is empty or does not exist')
        return []


def find_z_tag_for_app(z_antenna,rssi,antenna_indices):

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



def localization_for_app(x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices,z_tag,storage):
    

    #print("-------app--------")

    
    #for k in range(len(antenna_indices),len(antenna_indices)-1,-1):
    for k in range(len(antenna_indices),0,-1):
        all_combos = np.array(list(itertools.combinations(antenna_indices, k)))
        
        results_tag_all_combinations=np.empty((0,5))

        #i=all possible combination of the specific number of antennas
        for i in range(0,len(all_combos)):

            start = find_start_point_for_app(storage,all_combos[i])
            #print("start " +str(start))

            param, rsquare, CI = nonlinear_fit_for_app(x_antenna,y_antenna,z_antenna,phase,lamda,start,all_combos[i],z_tag,storage)

            if len(param):
               
                #param[0] = evaluate_storage_limits(storage,param[0])
                results_tag = [param[0],storage[4]*param[0]+storage[5],z_tag,rsquare,CI]
                results_tag_all_combinations=np.vstack((results_tag_all_combinations,results_tag))


        results_tag_best_combination = choose_combination_for_app(results_tag_all_combinations)
        flag_good_estimation = evaluate_estimation_for_app(results_tag_best_combination,15)

        #print(str(len(all_combos[i]))+" antennas")

        if flag_good_estimation: 
            return results_tag_best_combination, results_tag_all_combinations

    return results_tag_best_combination, results_tag_all_combinations



def find_start_point_for_app(storage,combination):
    
    start = [ storage[0]]

    for comb in combination:
        start.append(-30)

    return start


def evaluate_storage_limits(storage,x_tag):

    if x_tag<min(np.array([storage[0],storage[2]])):
        x_tag = min(np.array([storage[0],storage[2]]))
    elif x_tag>max(np.array([storage[0],storage[2]])):
        x_tag = max(np.array([storage[0],storage[2]]))

    return x_tag

def theoretical_for_app(x,x_antenna,y_antenna,z_antenna,phase,lamda,z_tag,storage,combination):
    
    slope = storage[4]
    intercept = storage[5]

    F=[]

    for i in range(0,len(combination)):

        fun= np.sqrt( (x[0]-x_antenna[combination[i]])**2 + (slope*x[0]+intercept-y_antenna[combination[i]])**2 +(z_tag-z_antenna[combination[i]])**2)/lamda[combination[i]]*4*pi+ x[i+1] - phase[combination[i]]
        F=np.hstack((F,fun))
        
    return F


def nonlinear_fit_for_app(x_antenna,y_antenna,z_antenna,phase,lamda,start,combination,z_tag,storage):
    
    
    param=least_squares(theoretical_for_app,start,args=(x_antenna,y_antenna,z_antenna,phase,lamda,z_tag,storage,combination),method='trf')

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
        return [],float("nan"), float("nan")
    
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
    
    return param.x, rsquare, CI[0]


def choose_combination_for_app(results_tag_all_combinations):

    results_tag_best_combination=np.empty((0,5))
    if len(results_tag_all_combinations):

        CI=results_tag_all_combinations[:,4]

        #print(CI_sum)
        if not(np.isnan(CI).all()):
            best_index=np.nanargmin(CI)
            results_tag_best_combination = results_tag_all_combinations[best_index,:]


    return results_tag_best_combination


def evaluate_estimation_for_app(results_tag_best_combination,threshold):

    flag_good_estimation=1

    if len(results_tag_best_combination):

        CI=results_tag_best_combination[4]  
        rsquare=results_tag_best_combination[4]  
        sign = results_tag_best_combination[0]  

        if (np.isnan(rsquare)) or (np.isnan(CI)):
            flag_good_estimation=0
            return flag_good_estimation

        if (CI>=threshold) or (rsquare<=0.75) or (sign==-1):
            flag_good_estimation=0
            return flag_good_estimation

    else:
        flag_good_estimation=0

    return flag_good_estimation


def save_tag_location_for_app(tag,results_tag_best_combination):

    tag.x=results_tag_best_combination[0]
    tag.y=results_tag_best_combination[1]
    tag.z=results_tag_best_combination[2]
    tag.CI=results_tag_best_combination[4]

    tag.reset_antennas_to_locate()


    return tag
