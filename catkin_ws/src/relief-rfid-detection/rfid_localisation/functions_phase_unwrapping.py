

import math
import numpy as np
import warnings
warnings.filterwarnings("ignore")
from scipy.optimize import least_squares
import copy
from math import pi
from scipy.stats.distributions import t



from classes import *




# %%
def get_unwrapped_measurements(antenna_measurements):

    #print('Unwrapping....')
    global unwrap_var
    global polynomial
    global number_of_considered_data
    global Dx_max
    global flag_target_segments
    
    #kane unwrap mono an exeis polles metriseis
    if len(antenna_measurements)>0:

        flag_target_segments=0

        R=-9999999999

        #2pi poly 1 
        unwrap_var=2
        Dx_max=10  #in cm
        polynomial=1
        number_of_considered_data=5

        tag_unwrap_measurements, param, rsquare, CI = phase_unwrapping(antenna_measurements)
        if rsquare>R:
            R=rsquare
            antenna_unwrapped_measurements=tag_unwrap_measurements


        
        #pi without poly
        unwrap_var=1
        Dx_max=10
        
        tag_unwrap_measurements, param, rsquare, CI = phase_unwrapping(antenna_measurements)
        if rsquare>R:
            R=rsquare
            antenna_unwrapped_measurements=tag_unwrap_measurements


        if R==-9999999999:
            return []


        return antenna_unwrapped_measurements     

    else:

        return []




# %%
def phase_unwrapping(antenna_measurements):

    #print(len(antenna_measurements))
    global flag_target_segments


    windows=create_segments(antenna_measurements)
    if len(windows)==0:
        #print('No windows created')
        return [], [], float("nan"), np.array([float("nan"),float("nan"),float("nan")])

    windows = create_sequences(windows)
    windows=delete_empty_windows(windows)

    if len(windows)==0:
        #print('No windows created')
        return [], [], float("nan"), np.array([float("nan"),float("nan"),float("nan")])

    windows=unwrap_windows(windows)
    windows = delete_empty_unwrapped_windows(windows)

    if len(windows)==0:
        #print('No windows created')
        return [], [], float("nan"), np.array([float("nan"),float("nan"),float("nan")])


    windows=remove_static_data(windows)
    windows=delete_empty_unwrapped_windows(windows)
    


    if len(windows)==0:
        #print('No windows created')
        return [], [], float("nan"), np.array([float("nan"),float("nan"),float("nan")])

    for w in windows:
        w=add_pi_single_window(w)


    initial_window = find_initial_segment(windows)
    #windows[initial_window].plot()

    if flag_target_segments:
        target_segments = find_target_segments(initial_window, windows)
    else:
        target_segments=[]

    unwrapped_measurements, param, rsquare, CI=unwrap_segments(initial_window,target_segments,windows)
    
    return unwrapped_measurements, param, rsquare, CI


# %%
def polyfit(x,y,degree):
    
    
    coeffs=np.polyfit(x,y,degree)
    p=np.poly1d(coeffs) 
               
   
    residuals=y-p(x)
    ss_res=np.sum(residuals**2)
    ss_tot=np.sum((y-np.mean(y))**2)
    rsquare=1-ss_res/ss_tot
    
    return coeffs, p, rsquare, ss_res


# %%
def create_segments(measurements):
    global Dx_max
    
    speed = measurements[:len(measurements)-1,9]
    time = measurements[:,0]
    Dt=np.diff(time)
    n_measurements=len(measurements)
   
    index=np.where(speed*Dt>Dx_max)   
    idx=index[0]
    idx=np.insert(idx,0,-1)
    idx=np.append(idx,n_measurements-1)
    
    windows=[]
    
    for j in range(0,len(idx)-1):
        
        window=Window(measurements[idx[j]+1:idx[j+1]+1,:])
        if len(window.wrapped_measurements):
            windows.append(window)

    return windows


# %%
def delete_empty_windows(windows):
    temp_windows=[w for w in windows if len(w.sequences)]
    windows=temp_windows

    return windows


# %%
def create_sequences_single_window(window):
    
    global Dx_max
    global unwrap_var

    measurements_temp=window.wrapped_measurements

    speed = measurements_temp[:len(measurements_temp)-1,9]
    time=measurements_temp[:,0]
    measurements_temp[:,2] = np.mod(measurements_temp[:,2],unwrap_var*pi) 
    phase=measurements_temp[:,2]
    
   
    n_measurements_window = len(measurements_temp)

    D_Phase=np.diff(phase)
    D_Phase_max=1
    
    D_Time=np.diff(time)
    D_X_max=Dx_max/2
    
    index=np.where((abs(D_Phase)>=D_Phase_max) | (abs(speed*D_Time)>=D_X_max) )   
    idx=index[0]
    idx=np.insert(idx,0,-1)
    idx=np.append(idx,n_measurements_window-1)
  
    window.sequences=[]
    
    for i in range(0,len(idx)-1):
        
        sequence=Sequence(measurements_temp[idx[i]+1:idx[i+1]+1,:])
        if len(sequence.wrapped_measurements)>=3:
            window.sequences.append(sequence)

    
    return window


# %%
def create_sequences(windows):
       
    for single_window in windows:
        
        single_window = create_sequences_single_window(single_window)

       
    return windows


# %%
def delete_empty_unwrapped_windows(windows):
    temp_windows=[w for w in windows if len(w.unwrapped_measurements)>5]
    windows=temp_windows
    return windows


# %%

def remove_static_data(windows):
    
    for w in windows:
        speed=w.unwrapped_measurements[:,10]
        w.unwrapped_measurements=w.unwrapped_measurements[speed==1]

    return windows


# %%
def unwrap_windows(windows):

    for w in windows:

        w = unwrap_single_window(w)
    
    return windows


# %%
def unwrap_single_window(window):
    
    global unwrap_var   
    
        
    k=0   
    
    for sequence in window.sequences:
        
        #n_measurements_sequence=len(sequence.wrapped_measurements)
        
        #if n_measurements_sequence>=3:
            
        time_sequence=sequence.wrapped_measurements[:,0]
        phase_sequence=sequence.wrapped_measurements[:,2]   
            
            
        if len(window.k)>=1:
            
            phase_window=window.unwrapped_measurements[:,2]
            time_window=window.unwrapped_measurements[:,0]   
            
            #[-2,2] gia pi [-1,1] gia 2pi
            step=list(range(unwrap_var-3,4-unwrap_var))
       
            if unwrap_var==1:
                k = find_best_k_for_unwrap_without_poly(step,window.k,phase_window,phase_sequence)
            else:                      
                k = find_best_k_for_unwrap(step,window.k,phase_window,phase_sequence,time_window,time_sequence)

        window.k.append(k)
        
        sequence.unwrapped_measurements = sequence.wrapped_measurements
        sequence.unwrapped_measurements[:,2] = phase_sequence+unwrap_var*pi*window.k[-1]

        window.unwrapped_measurements=np.vstack((window.unwrapped_measurements,sequence.unwrapped_measurements))

      
    return window


# %%
def find_best_k_for_unwrap(step,k_final,phase_window,phase_sequence,time_window,time_sequence):
   
   global polynomial
   
   rsquare=[None]*len(step)
   k_test=[None]*len(step)
   ss_res=[None]*len(step)
   
   for j in range(0,len(step)): 
                       
       k_test[j]=k_final[-1]+step[j]
               
       Phase_for_fit, Time_for_fit = create_test_data_for_fit(phase_window,phase_sequence,time_window,time_sequence,k_test[j])    
       
       z, f, rsquare[j], ss_res[j] = polyfit(Time_for_fit,Phase_for_fit,polynomial)


   index_best=rsquare.index(max(rsquare))
   if max(rsquare)<0.9:
       index_best=ss_res.index(min(ss_res))
   k=k_test[index_best]

   return k



# %%
def find_best_k_for_unwrap_without_poly(step,k_final,phase_window,phase_sequence):

   global unwrap_var

   difference=[None]*len(step)
   k_test=[None]*len(step)
  
   for j in range(0,len(step)): 
                       
       k_test[j]=k_final[-1]+step[j]
               
       ph_test=phase_sequence+unwrap_var*pi*k_test[j] 
       difference[j]=abs(phase_window[-1]-ph_test[0])

   index_best=difference.index(min(difference))

   k=k_test[index_best]

   return k


# %%
def create_test_data_for_fit(phase_window,phase_sequence,time_window,time_sequence,k_test):

    global number_of_considered_data
    global unwrap_var
    
    if len(phase_sequence)<=number_of_considered_data:
        
        Ph_test=phase_sequence+unwrap_var*pi*k_test                     
        T_test=time_sequence
    
    else:
        
        Ph_test=phase_sequence[0:number_of_considered_data-1]+unwrap_var*pi*k_test                      
        T_test=time_sequence[0:number_of_considered_data-1]   
    
    

    if len(phase_window)<=number_of_considered_data:
        Phase_for_fit=np.hstack((phase_window, Ph_test))
        Time_for_fit=np.hstack((time_window, T_test))
    
    else:
        Phase_for_fit=np.hstack((phase_window[-number_of_considered_data:], Ph_test))
        Time_for_fit=np.hstack((time_window[-number_of_considered_data:], T_test))  


    return Phase_for_fit, Time_for_fit


# %%
def find_initial_segment(windows):
        
        if windows:
#        RSSI_mean=[]
            num_of_meas=[]
            
            for w in windows: 
                #RSSI=w.unwrapped_measurements[:,3]           
    #            RSSI_mean.append(np.mean(RSSI))
                num_of_meas.append(len(w.unwrapped_measurements))
    
    #        initial_idx=RSSI_mean.index(max(RSSI_mean))
    
            initial_idx=num_of_meas.index(max(num_of_meas))
            
        else:
            initial_idx=-1
        
        return initial_idx


# %%
def nonlinear_fit(x_antenna,y_antenna,phase,lamda,weight,start):
    
    
    param =least_squares(theoretical,start,args=(x_antenna,y_antenna,phase,lamda,weight),method='trf')
    
    
    residuals=param.fun
    jac=param.jac
    #print(residuals)    
    # calculate rsquare
    ss_res=np.sum(residuals**2)
    ss_tot=np.sum(np.multiply(weight,(phase-np.average(phase,weights=weight))**2))
    rsquare=1-ss_res/ss_tot
    #print(rsquare)
  
    #Hessian matrix
    Hessian=np.dot(jac.T,jac)
    #print(Hessian)
    if (np.linalg.det(Hessian)==0):
        #print('det=0')
        return [],float("nan"),np.array([float("nan"),float("nan"),float("nan")])

    #degrees of freedom
    dof = len(residuals)-len(param.x)
    
    #variance of residuals
    sigma_resi = np.var(residuals)
    #print(sigma_resi)
    
    #vairance of coefficients
    sigma_cov=sigma_resi*np.linalg.inv(Hessian)
    
    #t-value 
    alpha=0.05
    tval=t.ppf(1.0-alpha/2,dof)
    
    #confidence interval
    CI=2*tval*np.sqrt(np.diag(sigma_cov))
    
    return param.x, rsquare, CI




def theoretical(x,x_antenna,y_antenna,phase,lamda,weight):


    return (np.sqrt( (x[0]-x_antenna)**2 + (x[1]-y_antenna)**2)/lamda*4*pi+ x[2] -phase)*weight


# %%
def find_start_point(x_antenna,y_antenna,theta_antenna):
    

    center_index=int(len(theta_antenna)/2)
    

    direction_vector=np.matrix([[math.cos(theta_antenna[center_index])],[math.sin(theta_antenna[center_index])]])
    center_point=np.matrix([[x_antenna[center_index]],[y_antenna[center_index]]])
    
    start_point=center_point+50*direction_vector
    

    start=[start_point[0,0],start_point[1,0],-30]
    
    return start


# %%
def add_pi_single_window(window):

    time=window.wrapped_measurements[:,0]
    phase=window.wrapped_measurements[:,2]
    time_unwrapped=window.unwrapped_measurements[:,0]
    phase_unwrapped=window.unwrapped_measurements[:,2]

    phase=phase[np.nonzero(np.in1d(time,time_unwrapped))[0]]
    
    phase_plus=np.mod(phase_unwrapped+pi,2*pi)
    phase_noplus=np.mod(phase_unwrapped,2*pi)
    
    if np.sum(np.in1d(phase_plus,phase))>np.sum(np.in1d(phase_noplus,phase)):
        window.unwrapped_measurements[:,2]=window.unwrapped_measurements[:,2]+pi
        
    
        
    window.k[0]=0
    window.k[-1]=math.floor(window.unwrapped_measurements[-1,2]/(2*pi))
    
    
    return window


# %%
def unwrap_segments(initial_idx,target_segments,windows):  
   
          
    if initial_idx==-1:
        return [], [], float("nan"), np.array([float("nan"),float("nan"),float("nan")])


    T_Unwrapped, X_Unwrapped, Y_Unwrapped, Z_Unwrapped, Th_Unwrapped, Ph_Unwrapped, L_Unwrapped, R_Unwrapped, k_unwrapped_first, k_unwrapped_last = take_unwrapped_measurements_from_specific_window(windows[initial_idx])
               
  
    for j in target_segments:
        j=int(j)      

        
        T_target, X_target, Y_target ,Z_target , Th_target, Ph_target, L_target, R_target, k_target_first, k_target_last = take_unwrapped_measurements_from_specific_window(windows[j])

        step, kmax = compute_step(T_Unwrapped,X_Unwrapped,Y_Unwrapped,T_target,X_target,Y_target,L_Unwrapped[0],k_unwrapped_first,k_unwrapped_last,k_target_first,k_target_last)
       

        if (kmax>=4 or kmax<0):
            continue

        if len(step)>5:
            step=step[int(len(step)/2)-2:int(len(step)/2)+3]
            
        
        k, best_param, best_CI, best_rsquare = find_best_k_for_unwrap_from_nonlinear_fit(step,T_Unwrapped,X_Unwrapped,Y_Unwrapped,Th_Unwrapped,Ph_Unwrapped,L_Unwrapped,T_target,X_target,Y_target,Th_target,Ph_target,L_target)

 
        if not(math.isnan(best_rsquare)):

            Ph_target=Ph_target+2*pi*k
            k_target_first=k_target_first+k
            k_target_last=k_target_last+k          
        
            T_Unwrapped, X_Unwrapped, Y_Unwrapped, Z_Unwrapped, Th_Unwrapped, Ph_Unwrapped, L_Unwrapped, R_Unwrapped, k_unwrapped_first, k_unwrapped_last = merge_target_unwrapped_measurements(T_Unwrapped, X_Unwrapped, Y_Unwrapped, Z_Unwrapped, Th_Unwrapped, Ph_Unwrapped, L_Unwrapped, R_Unwrapped,T_target, X_target, Y_target ,Z_target , Th_target, Ph_target, L_target, R_target, k_target_first, k_target_last,k_unwrapped_first, k_unwrapped_last)


    tag_unwrapped_measurements = create_tag_measurements_after_unwrapping(T_Unwrapped, X_Unwrapped, Y_Unwrapped, Z_Unwrapped, Th_Unwrapped, Ph_Unwrapped, L_Unwrapped, R_Unwrapped) 


    W_Unwrapped=np.ones(len(T_Unwrapped))
    start=find_start_point(X_Unwrapped,Y_Unwrapped,Th_Unwrapped)
    param, rsquare, CI = nonlinear_fit(X_Unwrapped,Y_Unwrapped,Ph_Unwrapped,L_Unwrapped,W_Unwrapped,start)              
            

    return tag_unwrapped_measurements, param, rsquare, CI


# %%
def create_tag_measurements_after_unwrapping(Time,X_Antenna,Y_Antenna,Z_Antenna,Theta_Antenna,Phase,Lamda,RSSI):   

    tag_measurements=np.empty(shape=(len(Time),8))
    
    tag_measurements[:,0]=Time
    tag_measurements[:,1]=X_Antenna
    tag_measurements[:,2]=Y_Antenna
    tag_measurements[:,3]=Z_Antenna
    tag_measurements[:,6]=RSSI
    tag_measurements[:,5]=Phase
    tag_measurements[:,7]=Lamda
    tag_measurements[:,4]=Theta_Antenna

   
    return tag_measurements


# %%
def merge_target_unwrapped_measurements(T_Unwrapped, X_Unwrapped, Y_Unwrapped, Z_Unwrapped, Th_Unwrapped, Ph_Unwrapped, L_Unwrapped, R_Unwrapped,T_target, X_target, Y_target ,Z_target , Th_target, Ph_target, L_target, R_target, k_target_first, k_target_last,k_unwrapped_first, k_unwrapped_last):
        
    if T_target[0]<T_Unwrapped[0]:
        
        k_unwrapped_first=k_target_first
        
        Ph_Unwrapped=np.hstack((Ph_target,Ph_Unwrapped))
        X_Unwrapped=np.hstack((X_target,X_Unwrapped))
        Y_Unwrapped=np.hstack((Y_target,Y_Unwrapped))
        Z_Unwrapped=np.hstack((Z_target,Z_Unwrapped))
        R_Unwrapped=np.hstack((R_target,R_Unwrapped))
        T_Unwrapped=np.hstack((T_target,T_Unwrapped))
        Th_Unwrapped=np.hstack((Th_target,Th_Unwrapped))
        L_Unwrapped=np.hstack((L_target,L_Unwrapped))
        
    else:
        
        k_unwrapped_last=k_target_last
                        
        Ph_Unwrapped=np.hstack((Ph_Unwrapped,Ph_target))
        X_Unwrapped=np.hstack((X_Unwrapped,X_target))
        Y_Unwrapped=np.hstack((Y_Unwrapped,Y_target))
        Z_Unwrapped=np.hstack((Z_Unwrapped,Z_target))
        R_Unwrapped=np.hstack((R_Unwrapped,R_target))
        T_Unwrapped=np.hstack((T_Unwrapped,T_target))
        Th_Unwrapped=np.hstack((Th_Unwrapped,Th_target))
        L_Unwrapped=np.hstack((L_Unwrapped,L_target))
    
    return  T_Unwrapped, X_Unwrapped, Y_Unwrapped, Z_Unwrapped, Th_Unwrapped, Ph_Unwrapped, L_Unwrapped, R_Unwrapped, k_unwrapped_first, k_unwrapped_last


# %%
def create_test_data_for_nonlinear_fit(T_Unwrapped, X_Unwrapped,Y_Unwrapped,Th_Unwrapped,L_Unwrapped,T_target, X_target,Y_target,Th_target,L_target):
        
    power=0
    
    if T_target[0]<T_Unwrapped[0]:
            
        testX=np.hstack((X_target,X_Unwrapped))
        testY=np.hstack((Y_target,Y_Unwrapped))   
        testTh=np.hstack((Th_target,Th_Unwrapped))  
        testT=np.hstack((T_target,T_Unwrapped))        
        
        distance_target = np.sqrt( (X_Unwrapped[0]-X_target)**2 + (Y_Unwrapped[0]-Y_target)**2 )
        distance_unwrapped = np.sqrt( (X_Unwrapped-X_target[-1])**2 + (Y_Unwrapped-Y_target[-1])**2 ) 
        
        testW_target=1/distance_target
        testW_Unwrapped=1/distance_unwrapped
        
        testW=np.hstack((testW_target,testW_Unwrapped)) 
                    
    else:
            
        testX=np.hstack((X_Unwrapped,X_target))
        testY=np.hstack((Y_Unwrapped,Y_target))
        testTh=np.hstack((Th_Unwrapped,Th_target))
        testT=np.hstack((T_Unwrapped,T_target))
        
        distance_target = np.sqrt( (X_Unwrapped[-1]-X_target)**2 + (Y_Unwrapped[-1]-Y_target)**2 )
        distance_unwrapped = np.sqrt( (X_Unwrapped-X_target[0])**2 + (Y_Unwrapped-Y_target[0])**2 )
        
        testW_target=1/distance_target
        testW_Unwrapped=1/distance_unwrapped
        
        testW=np.hstack((testW_Unwrapped,testW_target))

    testL=np.hstack((L_target,L_Unwrapped))  

    testW=testW**power
    testW=testW/max(testW)

    return testT, testX, testY, testTh, testW, testL


# %%
def find_best_k_for_unwrap_from_nonlinear_fit(step,T_Unwrapped,X_Unwrapped,Y_Unwrapped,Th_Unwrapped,Ph_Unwrapped,L_Unwrapped,T_target,X_target,Y_target,Th_target,Ph_target,L_target):  
        
    T_for_fit, X_for_fit,Y_for_fit, Th_for_fit, W_for_fit, L_for_fit = create_test_data_for_nonlinear_fit(T_Unwrapped,X_Unwrapped,Y_Unwrapped,Th_Unwrapped,L_Unwrapped,T_target,X_target,Y_target,Th_target,L_target)


    rsquare=[None]*len(step)
    param=[None]*len(step)
    CI=[None]*len(step)         
    
    for i in range(0,len(step)):
        
        if T_target[0]<T_Unwrapped[0]:
            
            Ph_for_fit=np.hstack((Ph_target+step[i]*2*pi,Ph_Unwrapped))              

        else:
            
            Ph_for_fit=np.hstack((Ph_Unwrapped,Ph_target+step[i]*2*pi))

        
        #plt.scatter(T_for_fit,Ph_for_fit)
        #plt.show


        start=find_start_point(X_for_fit,Y_for_fit,Th_for_fit) 
        
        param[i], rsquare[i], CI[i]=nonlinear_fit(X_for_fit,Y_for_fit,Ph_for_fit,L_for_fit,W_for_fit,start)

    #print(rsquare)
    #print(CI)
       
    index_best=rsquare.index(max(rsquare))
    
    k=step[index_best]
    
    best_param=param[index_best]   
    best_CI=CI[index_best]
    best_rsquare=rsquare[index_best]  

    return k, best_param, best_CI, best_rsquare


# %%
def compute_step(T_Unwrapped,X_Unwrapped,Y_Unwrapped,T_target,X_target,Y_target,lamda,k_unwrapped_first,k_unwrapped_last,k_target_first,k_target_last):

    if T_target[0]<T_Unwrapped[0]:  #to data einai aristera kai to unwrapped deksia

        distance= np.sqrt( (X_Unwrapped[0]-X_target[-1])**2 + (Y_Unwrapped[0]-Y_target[-1])**2 )
        k = math.ceil(distance/(lamda/2))+1  
                    
        step=list(range(int(k_unwrapped_first-k_target_last-k),int(k_unwrapped_first-k_target_last+k+1)))

    else:
        
        distance= np.sqrt( (X_Unwrapped[-1]-X_target[0])**2 + (Y_Unwrapped[-1]-Y_target[0])**2 )
        k = math.ceil(distance/(lamda/2)) +1  
                                
        step=list(range(int(k_unwrapped_last-k_target_first-k),int(k_unwrapped_last-k_target_first+k+1)))


    return step, k 


# %%
def take_unwrapped_measurements_from_specific_window(w):

    data = w.unwrapped_measurements

    T=data[:,0] 
    X=data[:,5]
    Y=data[:,6]
    Z=data[:,7]
    Ph=data[:,2]
    R=data[:,3]
    L=data[:,4]
    Th=data[:,8]
    
    k_first=w.k[0]
    k_last=w.k[-1]

    
    return T, X, Y ,Z , Th, Ph, L, R, k_first, k_last


# %%
def find_target_segments(initial_idx,windows):           
        
    target_segments=[]

    n_windows=len(windows)

    size_window=[]
    for w in windows:
        size_window.append(len(w.unwrapped_measurements))
    #print(size_window)

    if initial_idx!=-1:
        
        right_segments=list(range(initial_idx+1,n_windows))
        left_segments=list(range(initial_idx-1,-1,-1))

  

        #for i in range(len(right_segments),max(len(right_segments),len(left_segments))):
            #right_segments.append([])
        
       # for i in range(len(left_segments),max(len(right_segments),len(left_segments))):
           # left_segments.append([])
        
        #print(right_segments)
        #print(left_segments)
        while left_segments or right_segments:
            if left_segments:
                size_left_segment=size_window[int(left_segments[0])]
            else:
                size_left_segment=0

            if right_segments:
                size_right_segment=size_window[int(right_segments[0])]
            else:
                size_right_segment=0  

            #print(size_left_segment)
            #print(size_right_segment)
            if size_left_segment>size_right_segment:
                target_segments = np.hstack((target_segments,left_segments[0]))
                del left_segments[0]
            else:
                target_segments = np.hstack((target_segments,right_segments[0]))
                del right_segments[0]


            #target_segments=np.hstack((target_segments,right_segments[i],left_segments[i]))
            #target_segments=np.hstack((target_segments,left_segments[i],right_segments[i]))

        
    return target_segments 