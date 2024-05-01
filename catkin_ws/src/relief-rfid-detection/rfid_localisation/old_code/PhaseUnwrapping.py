#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 19 02:28:10 2020

@author: tasos
"""

from scipy.optimize import least_squares
from scipy.stats.distributions import t
import os
import pandas as pandas
import numpy as np
import math
from math import pi
#import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore")








def unwrapSegments_2(initial_idx,target_segments,T_window_Unwrapped, X_window_Unwrapped, Y_window_Unwrapped, Z_window_Unwrapped, R_window_Unwrapped, Ph_window_Unwrapped,k_final,lamda,polarity):

    global flag_k
    global power

    if initial_idx==-1:
        return [], [],[],[],[],[]


    X_Unwrapped=X_window_Unwrapped[initial_idx]
    Y_Unwrapped=Y_window_Unwrapped[initial_idx]
    Z_Unwrapped=Z_window_Unwrapped[initial_idx]
    Ph_Unwrapped=Ph_window_Unwrapped[initial_idx]
    R_Unwrapped=R_window_Unwrapped[initial_idx]
    T_Unwrapped=T_window_Unwrapped[initial_idx]

    k_Unwrapped_first=k_final[initial_idx][0]
    k_Unwrapped_last=k_final[initial_idx][-1]



#    plt.figure()
#    plt.scatter(T_Unwrapped, Ph_Unwrapped)
#    plt.show()


    for j in target_segments:
        j=int(j)

        Ph_data=Ph_window_Unwrapped[j]
        X_data=X_window_Unwrapped[j]
        Y_data=Y_window_Unwrapped[j]
        Z_data=Z_window_Unwrapped[j]
        R_data=R_window_Unwrapped[j]
        T_data=T_window_Unwrapped[j]

        k_data_first=k_final[j][0]
        k_data_last=k_final[j][-1]


        if T_data[0]<T_Unwrapped[0]:  #to data einai aristera kai to unwrapped deksia

            distance= np.sqrt( (X_Unwrapped[0]-X_data[-1])**2 + (Y_Unwrapped[0]-Y_data[-1])**2 )
            k = math.ceil(distance/(lamda/2))+1

            step=list(range(int(k_Unwrapped_first-k_data_last-k),int(k_Unwrapped_first-k_data_last+k+1)))

        else:

            distance= np.sqrt( (X_Unwrapped[-1]-X_data[0])**2 + (Y_Unwrapped[-1]-Y_data[0])**2 )
            k = math.ceil(distance/(lamda/2)) +1


            step=list(range(int(k_Unwrapped_last-k_data_first-k),int(k_Unwrapped_last-k_data_first+k+1)))


        if (k>=2 or k<0) and flag_k==1:
            continue


        rsquare=[None]*len(step)
        param=[None]*len(step)
        CI=[None]*len(step)
        CI_c=[None]*len(step)


        if T_data[0]<T_Unwrapped[0]:

                testX=np.hstack((X_data,X_Unwrapped))
                testY=np.hstack((Y_data,Y_Unwrapped))
                testZ=np.hstack((Z_data,Z_Unwrapped))


                distance_data = np.sqrt( (X_Unwrapped[0]-X_data)**2 + (Y_Unwrapped[0]-Y_data)**2 )
                distance_unwrapped = np.sqrt( (X_Unwrapped-X_data[-1])**2 + (Y_Unwrapped-Y_data[-1])**2 )

#                W_data=np.linspace(1/len(Ph_data),1,len(Ph_data))
#                W_Unwrapped= np.linspace(1,1/len(Ph_Unwrapped),len(Ph_Unwrapped))

                testW_data=1/distance_data
                testW_Unwrapped=1/distance_unwrapped

                testW=np.hstack((testW_data,testW_Unwrapped))



        else:

                testX=np.hstack((X_Unwrapped,X_data))
                testY=np.hstack((Y_Unwrapped,Y_data))
                testZ=np.hstack((Z_Unwrapped,Z_data))

#                W_Unwrapped=np.linspace(1/len(Ph_Unwrapped),1,len(Ph_Unwrapped))
#                W_data= np.linspace(1,1/len(Ph_data),len(Ph_data))

                distance_data = np.sqrt( (X_Unwrapped[-1]-X_data)**2 + (Y_Unwrapped[-1]-Y_data)**2 )
                distance_unwrapped = np.sqrt( (X_Unwrapped-X_data[0])**2 + (Y_Unwrapped-Y_data[0])**2 )

                testW_data=1/distance_data
                testW_Unwrapped=1/distance_unwrapped

                testW=np.hstack((testW_Unwrapped,testW_data))

        testW=testW**power
        testW=testW/max(testW)


        for i in range(0,len(step)):

            if T_data[0]<T_Unwrapped[0]:

                testPh=np.hstack((Ph_data+step[i]*2*pi,Ph_Unwrapped))


            else:

                testPh=np.hstack((Ph_Unwrapped,Ph_data+step[i]*2*pi))


            start=findStartPoint(testX,testY,polarity)
            param[i], rsquare[i], CI[i]=nonlinear_fit(testX,testY,testZ,testPh,testW,start,lamda)
            CI_c[i]=CI[i][2]
#            print(rsquare[i])

        index_best=rsquare.index(max(rsquare))
#            index_best=CI_c.index(min(CI_c))

        Ph_data=Ph_data+2*pi*step[index_best]
        k_data_first=k_data_first+step[index_best]
        k_data_last=k_data_last+step[index_best]

        best_param=param[index_best]
        best_CI=CI[index_best]
        best_rsquare=rsquare[index_best]


        if T_data[0]<T_Unwrapped[0]:

            k_Unwrapped_first=k_data_first

            Ph_Unwrapped=np.hstack((Ph_data,Ph_Unwrapped))
            X_Unwrapped=np.hstack((X_data,X_Unwrapped))
            Y_Unwrapped=np.hstack((Y_data,Y_Unwrapped))
            Z_Unwrapped=np.hstack((Z_data,Z_Unwrapped))
            R_Unwrapped=np.hstack((R_data,R_Unwrapped))
            T_Unwrapped=np.hstack((T_data,T_Unwrapped))

        else:

            k_Unwrapped_last=k_data_last

            Ph_Unwrapped=np.hstack((Ph_Unwrapped,Ph_data))
            X_Unwrapped=np.hstack((X_Unwrapped,X_data))
            Y_Unwrapped=np.hstack((Y_Unwrapped,Y_data))
            Z_Unwrapped=np.hstack((Z_Unwrapped,Z_data))
            R_Unwrapped=np.hstack((R_Unwrapped,R_data))
            T_Unwrapped=np.hstack((T_Unwrapped,T_data))




#            plt.figure()
#            plt.scatter(T_Unwrapped, Ph_Unwrapped)
#            plt.show()

    return T_Unwrapped, X_Unwrapped, Y_Unwrapped, Z_Unwrapped, R_Unwrapped, Ph_Unwrapped














def unwrapSegments_1(initial_idx,target_segments,T_window_Unwrapped, X_window_Unwrapped, Y_window_Unwrapped, Z_window_Unwrapped, R_window_Unwrapped, Ph_window_Unwrapped,lamda,polarity):
    global flag_k

    if initial_idx==-1:
        return [], [],[],[],[],[]


    X_Unwrapped=X_window_Unwrapped[initial_idx]
    Y_Unwrapped=Y_window_Unwrapped[initial_idx]
    Z_Unwrapped=Z_window_Unwrapped[initial_idx]
    Ph_Unwrapped=Ph_window_Unwrapped[initial_idx]
    R_Unwrapped=R_window_Unwrapped[initial_idx]
    T_Unwrapped=T_window_Unwrapped[initial_idx]



    for j in target_segments:
        j=int(j)

        if len(Ph_window_Unwrapped[j])>=5:



            Ph_data=Ph_window_Unwrapped[j]
            X_data=X_window_Unwrapped[j]
            Y_data=Y_window_Unwrapped[j]
            Z_data=Z_window_Unwrapped[j]
            R_data=R_window_Unwrapped[j]
            T_data=T_window_Unwrapped[j]


            if T_data[0]<T_Unwrapped[0]:  #to data einai aristera kai to unwrapped deksia

                distance= np.sqrt( (X_Unwrapped[0]-X_data[-1])**2 + (Y_Unwrapped[0]-Y_data[-1])**2 )
                k = math.ceil(distance/(lamda/2))

            else:

                distance= np.sqrt( (X_Unwrapped[-1]-X_data[0])**2 + (Y_Unwrapped[-1]-Y_data[0])**2 )
                k = math.ceil(distance/(lamda/2))


            if (k>=2 or k<0) and flag_k==1:
                continue



            if T_data[0]<T_Unwrapped[0]:


                distance_data = np.sqrt( (X_Unwrapped[0]-X_data)**2 + (Y_Unwrapped[0]-Y_data)**2 )
                distance_unwrapped = np.sqrt( (X_Unwrapped-X_data[-1])**2 + (Y_Unwrapped-Y_data[-1])**2 )

                W_data=1/distance_data
                W_Unwrapped=1/distance_unwrapped

            else:

                distance_data = np.sqrt( (X_Unwrapped[-1]-X_data)**2 + (Y_Unwrapped[-1]-Y_data)**2 )
                distance_unwrapped = np.sqrt( (X_Unwrapped-X_data[0])**2 + (Y_Unwrapped-Y_data[0])**2 )

                W_data=1/distance_data
                W_Unwrapped=1/distance_unwrapped





#            if T_data[0]<T_Unwrapped[0]:
#                W_data=np.linspace(1/len(Ph_data),1,len(Ph_data))
#                W_Unwrapped= np.linspace(1,1/len(Ph_Unwrapped),len(Ph_Unwrapped))
#            else:
#                W_Unwrapped=np.linspace(1/len(Ph_Unwrapped),1,len(Ph_Unwrapped))
#                W_data= np.linspace(1,1/len(Ph_data),len(Ph_data))

            W=np.ones(len(X_Unwrapped))
            start=findStartPoint(X_Unwrapped,Y_Unwrapped,polarity)
            param,rsquare,CI=nonlinear_fit(X_Unwrapped,Y_Unwrapped,Z_Unwrapped,Ph_Unwrapped,W,start,lamda)

            best_param=param
            best_CI=CI
            best_rsquare=rsquare

            W=np.ones(len(X_data))
            ph_dif=theoretical(best_param.x,X_data,Y_data,Z_data,Ph_data,W,lamda)
            num=np.mean((ph_dif)/(2*pi))
            k_init=round(num)


            if T_data[0]<T_Unwrapped[0]:

                testX=np.hstack((X_data,X_Unwrapped))
                testY=np.hstack((Y_data,Y_Unwrapped))
                testZ=np.hstack((Z_data,Z_Unwrapped))

                testW=np.hstack((W_data,W_Unwrapped))


            else:

                testX=np.hstack((X_Unwrapped,X_data))
                testY=np.hstack((Y_Unwrapped,Y_data))
                testZ=np.hstack((Z_Unwrapped,Z_data))

                testW=np.hstack((W_Unwrapped,W_data))

            step=list(range(-2,3))

            k_test=[None]*len(step)
            rsquare=[None]*len(step)
            param=[None]*len(step)
            CI=[None]*len(step)
            CI_c=[None]*len(step)

            for i in range(0,len(step)):


                k_test[i]=k_init+step[i]

                if T_data[0]<T_Unwrapped[0]:

                    testPh=np.hstack((Ph_data+k_test[i]*2*pi,Ph_Unwrapped))

                else:

                    testPh=np.hstack((Ph_Unwrapped,Ph_data+k_test[i]*2*pi))


                param[i], rsquare[i], CI[i]=nonlinear_fit(testX,testY,testZ,testPh,testW,start,lamda)
                CI_c[i]=CI[i][2]


            index_best=rsquare.index(max(rsquare))
#            index_best=CI_c.index(min(CI_c))

            k_final=k_test[index_best]
            best_param=param[index_best]
            best_CI=CI[index_best]
            best_rsquare=rsquare[index_best]

            Ph_data=Ph_data+2*pi*k_final

            if T_data[0]<T_Unwrapped[0]:


                Ph_Unwrapped=np.hstack((Ph_data,Ph_Unwrapped))
                X_Unwrapped=np.hstack((X_data,X_Unwrapped))
                Y_Unwrapped=np.hstack((Y_data,Y_Unwrapped))
                Z_Unwrapped=np.hstack((Z_data,Z_Unwrapped))
                R_Unwrapped=np.hstack((R_data,R_Unwrapped))
                T_Unwrapped=np.hstack((T_data,T_Unwrapped))

            else:

                Ph_Unwrapped=np.hstack((Ph_Unwrapped,Ph_data))
                X_Unwrapped=np.hstack((X_Unwrapped,X_data))
                Y_Unwrapped=np.hstack((Y_Unwrapped,Y_data))
                Z_Unwrapped=np.hstack((Z_Unwrapped,Z_data))
                R_Unwrapped=np.hstack((R_Unwrapped,R_data))
                T_Unwrapped=np.hstack((T_Unwrapped,T_data))


#    W_Unwrapped=np.ones(len(Ph_Unwrapped))
#
#    start=findStartPoint(X_Unwrapped,Y_Unwrapped)
#    param,rsquare,CI=nonlinear_fit(X_Unwrapped,Y_Unwrapped,Z_Unwrapped,Ph_Unwrapped,W_Unwrapped,start)
#



    return T_Unwrapped, X_Unwrapped, Y_Unwrapped, Z_Unwrapped, R_Unwrapped, Ph_Unwrapped











def findTargetSegments_Center(initial_idx,n_windows):


    target_segments=[]

    if initial_idx!=-1:

        right_segments=list(range(initial_idx+1,n_windows))
        left_segments=list(range(initial_idx-1,-1,-1))

        for i in range(len(right_segments),max(len(right_segments),len(left_segments))):
            right_segments.append([])

        for i in range(len(left_segments),max(len(right_segments),len(left_segments))):
            left_segments.append([])

        for i in range(0,len(left_segments)):
            target_segments=np.hstack((target_segments,right_segments[i],left_segments[i]))


    return target_segments











def findInitialSegment(R_window_Unwrapped,n_windows):

        if n_windows:
#        RSSI_mean=[]
            NumOfMeas=[]

            for i in range(0,n_windows):
    #            RSSI_mean.append(np.mean(R_window_Unwrapped[i]))
                NumOfMeas.append(len(R_window_Unwrapped[i]))

    #        initial_idx=RSSI_mean.index(max(RSSI_mean))

            initial_idx=NumOfMeas.index(max(NumOfMeas))

        else:
            initial_idx=-1

        return initial_idx











def unwrap(T_Seq, X_Seq, Y_Seq, Z_Seq, R_Seq, Ph_Seq, S_Seq,n_windows,n_sequences):


    X_window_Unwrapped=[None]*n_windows
    Y_window_Unwrapped=[None]*n_windows
    Ph_window_Unwrapped=[None]*n_windows
    R_window_Unwrapped=[None]*n_windows
    T_window_Unwrapped=[None]*n_windows
    Z_window_Unwrapped=[None]*n_windows
    S_window_Unwrapped=[None]*n_windows

    k_final=[None]*n_windows

    for j in range(0,n_windows):

        if flag_pi==0:

            T_window_Unwrapped[j], X_window_Unwrapped[j], Y_window_Unwrapped[j], Z_window_Unwrapped[j], R_window_Unwrapped[j], Ph_window_Unwrapped[j],S_window_Unwrapped[j], k_final[j] = unwrap_SingleWindow_2pi(T_Seq[j], X_Seq[j],Y_Seq[j],Z_Seq[j],R_Seq[j],Ph_Seq[j],S_Seq[j])

        else:

            T_Seq[j], X_Seq[j],Y_Seq[j],Z_Seq[j],R_Seq[j],Ph_Seq[j],S_Seq[j],n_sequences[j]  = Sequencespi(T_Seq[j], X_Seq[j],Y_Seq[j],Z_Seq[j],R_Seq[j],Ph_Seq[j],S_Seq[j],n_sequences[j])
#            print(n_sequences[j])
#            for i in range(0,n_sequences[j]):
#                plt.figure
#                plt.scatter(T_Seq[j][i],Ph_Seq[j][i])
#                plt.show()

            T_window_Unwrapped[j], X_window_Unwrapped[j], Y_window_Unwrapped[j], Z_window_Unwrapped[j], R_window_Unwrapped[j], Ph_window_Unwrapped[j],S_window_Unwrapped[j], k_final[j] = unwrap_SingleWindow_pi(T_Seq[j], X_Seq[j],Y_Seq[j],Z_Seq[j],R_Seq[j],Ph_Seq[j],S_Seq[j],n_sequences[j])


    X_window_Unwrapped=[x for x in X_window_Unwrapped if x!=[]]
    Y_window_Unwrapped=[x for x in Y_window_Unwrapped if x!=[]]
    Z_window_Unwrapped=[x for x in Z_window_Unwrapped if x!=[]]
    Ph_window_Unwrapped=[x for x in Ph_window_Unwrapped if x!=[]]
    R_window_Unwrapped=[x for x in R_window_Unwrapped if x!=[]]
    T_window_Unwrapped=[x for x in T_window_Unwrapped if x!=[]]
    S_window_Unwrapped=[x for x in S_window_Unwrapped if x!=[]]

    k_final=[x for x in k_final if x!=[]]

    n_windows=len(Ph_window_Unwrapped)


    return T_window_Unwrapped, X_window_Unwrapped, Y_window_Unwrapped, Z_window_Unwrapped, R_window_Unwrapped, Ph_window_Unwrapped, S_window_Unwrapped, n_windows, k_final




def unwrap_SingleWindow_pi(T_piSeq,X_piSeq,Y_piSeq,Z_piSeq,R_piSeq,Ph_piSeq,S_piSeq,n_sequences):
    global accounted_number
    global polynomial

    X_window_Unwrapped=[]
    Y_window_Unwrapped=[]
    Ph_window_Unwrapped=[]
    R_window_Unwrapped=[]
    T_window_Unwrapped=[]
    Z_window_Unwrapped=[]
    S_window_Unwrapped=[]



    X_piSeq_Unwrapped=[]
    Y_piSeq_Unwrapped=[]
    Ph_piSeq_Unwrapped=[]
    R_piSeq_Unwrapped=[]
    T_piSeq_Unwrapped=[]
    Z_piSeq_Unwrapped=[]
    S_piSeq_Unwrapped=[]


    k_final=[]
    k=0

    for i in range(0,n_sequences):


        n_measurements_sequence=len(Ph_piSeq[i])

        if n_measurements_sequence>=2:



            if len(k_final)>=1:


                step=list(range(-2,3))

                rsquare=[None]*len(step)
                k_test=[None]*len(step)
                ss_res=[None]*len(step)

                for j in range(0,len(step)):


                    testPh=[]
                    testT=[]

                    k_test[j]=k_final[-1]+step[j]

                    #accounted_number=10

                    if len(Ph_piSeq[i])<=accounted_number:

                        Ph_test=Ph_piSeq[i]+pi*k_test[j]
                        T_test=T_piSeq[i]

                    else:

                        Ph_test=Ph_piSeq[i][0:accounted_number-1]+pi*k_test[j]
                        T_test=T_piSeq[i][0:accounted_number-1]

                    if len(Ph_window_Unwrapped)<=accounted_number:
                        testPh=np.hstack((Ph_window_Unwrapped, Ph_test))
                        testT=np.hstack((T_window_Unwrapped, T_test))

                    else:
                        testPh=np.hstack((Ph_window_Unwrapped[-accounted_number:], Ph_test))
                        testT=np.hstack((T_window_Unwrapped[-accounted_number:], T_test))


                    z, f, rsquare[j], ss_res[j] = polyfit(testT,testPh,polynomial)

                index_best=rsquare.index(max(rsquare))
                if max(rsquare)<0.9:
                    index_best=ss_res.index(min(ss_res))

                k=k_test[index_best]


            k_final.append(k)

            Ph_piSeq_Unwrapped.append(Ph_piSeq[i]+pi*k_final[-1])

            X_piSeq_Unwrapped.append(X_piSeq[i])
            Y_piSeq_Unwrapped.append(Y_piSeq[i])
            R_piSeq_Unwrapped.append(R_piSeq[i])
            T_piSeq_Unwrapped.append(T_piSeq[i])
            Z_piSeq_Unwrapped.append(Z_piSeq[i])
            S_piSeq_Unwrapped.append(S_piSeq[i])

            Ph_window_Unwrapped=np.hstack((Ph_window_Unwrapped,Ph_piSeq[i]+pi*k_final[-1]))
            X_window_Unwrapped=np.hstack((X_window_Unwrapped,X_piSeq[i]))
            Y_window_Unwrapped=np.hstack((Y_window_Unwrapped,Y_piSeq[i]))
            R_window_Unwrapped=np.hstack((R_window_Unwrapped,R_piSeq[i]))
            T_window_Unwrapped=np.hstack((T_window_Unwrapped,T_piSeq[i]))
            Z_window_Unwrapped=np.hstack((Z_window_Unwrapped,Z_piSeq[i]))
            S_window_Unwrapped=np.hstack((S_window_Unwrapped,S_piSeq[i]))

    return T_window_Unwrapped, X_window_Unwrapped, Y_window_Unwrapped, Z_window_Unwrapped, R_window_Unwrapped, Ph_window_Unwrapped, S_window_Unwrapped, k_final










def unwrap_SingleWindow_2pi(T_2piSeq,X_2piSeq,Y_2piSeq,Z_2piSeq,R_2piSeq,Ph_2piSeq,S_2piSeq):
    global accounted_number
    global polynomial

    X_window_Unwrapped=[]
    Y_window_Unwrapped=[]
    Ph_window_Unwrapped=[]
    R_window_Unwrapped=[]
    T_window_Unwrapped=[]
    Z_window_Unwrapped=[]
    S_window_Unwrapped=[]

    n_sequences=len(Ph_2piSeq)

    X_2piSeq_Unwrapped=[]
    Y_2piSeq_Unwrapped=[]
    Ph_2piSeq_Unwrapped=[]
    R_2piSeq_Unwrapped=[]
    T_2piSeq_Unwrapped=[]
    Z_2piSeq_Unwrapped=[]
    S_2piSeq_Unwrapped=[]


    k_final=[]
    k=0

    for i in range(0,n_sequences):


        n_measurements_sequence=len(Ph_2piSeq[i])

        if n_measurements_sequence>=3:



            if len(k_final)>=1:


                step=list(range(-1,2))

                rsquare=[None]*len(step)
                k_test=[None]*len(step)
                ss_res=[None]*len(step)

                for j in range(0,len(step)):


                    testPh=[]
                    testT=[]

                    k_test[j]=k_final[-1]+step[j]

#                    accounted_number=10

                    if len(Ph_2piSeq[i])<=accounted_number:

                        Ph_test=Ph_2piSeq[i]+2*pi*k_test[j]
                        T_test=T_2piSeq[i]

                    else:

                        Ph_test=Ph_2piSeq[i][0:accounted_number-1]+2*pi*k_test[j]
                        T_test=T_2piSeq[i][0:accounted_number-1]

                    if len(Ph_window_Unwrapped)<=accounted_number:
                        testPh=np.hstack((Ph_window_Unwrapped, Ph_test))
                        testT=np.hstack((T_window_Unwrapped, T_test))

                    else:
                        testPh=np.hstack((Ph_window_Unwrapped[-accounted_number:], Ph_test))
                        testT=np.hstack((T_window_Unwrapped[-accounted_number:], T_test))


                    z, f, rsquare[j], ss_res[j] = polyfit(testT,testPh,polynomial)


                index_best=rsquare.index(max(rsquare))
                if max(rsquare)<0.9:
                    index_best=ss_res.index(min(ss_res))
                k=k_test[index_best]


            k_final.append(k)

            Ph_2piSeq_Unwrapped.append(Ph_2piSeq[i]+2*pi*k_final[-1])

            X_2piSeq_Unwrapped.append(X_2piSeq[i])
            Y_2piSeq_Unwrapped.append(Y_2piSeq[i])
            R_2piSeq_Unwrapped.append(R_2piSeq[i])
            T_2piSeq_Unwrapped.append(T_2piSeq[i])
            Z_2piSeq_Unwrapped.append(Z_2piSeq[i])
            S_2piSeq_Unwrapped.append(S_2piSeq[i])

            Ph_window_Unwrapped=np.hstack((Ph_window_Unwrapped,Ph_2piSeq[i]+2*pi*k_final[-1]))
            X_window_Unwrapped=np.hstack((X_window_Unwrapped,X_2piSeq[i]))
            Y_window_Unwrapped=np.hstack((Y_window_Unwrapped,Y_2piSeq[i]))
            R_window_Unwrapped=np.hstack((R_window_Unwrapped,R_2piSeq[i]))
            T_window_Unwrapped=np.hstack((T_window_Unwrapped,T_2piSeq[i]))
            Z_window_Unwrapped=np.hstack((Z_window_Unwrapped,Z_2piSeq[i]))
            S_window_Unwrapped=np.hstack((S_window_Unwrapped,S_2piSeq[i]))

    return T_window_Unwrapped, X_window_Unwrapped, Y_window_Unwrapped, Z_window_Unwrapped, R_window_Unwrapped, Ph_window_Unwrapped, S_window_Unwrapped, k_final











def Sequences2pi(T_window, X_window, Y_window, Z_window, R_window, Ph_window, S_window, n_windows):

    X_2piSeq=[None]*n_windows
    Y_2piSeq=[None]*n_windows
    Ph_2piSeq=[None]*n_windows
    R_2piSeq=[None]*n_windows
    T_2piSeq=[None]*n_windows
    Z_2piSeq=[None]*n_windows
    S_2piSeq=[None]*n_windows
    n_sequences=[None]*n_windows

    for j in range(0,n_windows):

        n_measurements_window=len(T_window[j])

        if n_measurements_window:
            T_2piSeq[j], X_2piSeq[j], Y_2piSeq[j], Z_2piSeq[j], R_2piSeq[j], Ph_2piSeq[j],S_2piSeq[j], n_sequences[j] = Sequences2pi_SingleWindow(T_window[j],X_window[j],Y_window[j],Z_window[j],R_window[j],Ph_window[j],S_window[j],n_measurements_window)



    return T_2piSeq, X_2piSeq, Y_2piSeq, Z_2piSeq, R_2piSeq, Ph_2piSeq, S_2piSeq, n_sequences




def Sequencespi(T_2piSeq, X_2piSeq, Y_2piSeq, Z_2piSeq, R_2piSeq, Ph_2piSeq, S_2piSeq,n_sequences):

    X_piSeq=[]
    Y_piSeq=[]
    Ph_piSeq=[]
    T_piSeq=[]
    R_piSeq=[]
    Z_piSeq=[]
    S_piSeq=[]

    D_Phase_max=0.5

    for j in range(0,n_sequences):
        Ph_2piSeq[j]=np.mod(Ph_2piSeq[j],pi)

        n_measurements_sequence=len(Ph_2piSeq[j])

        D_Phase=np.diff(Ph_2piSeq[j])
        index=np.where((abs(D_Phase)>=D_Phase_max) )
        idx=index[0]
        idx=np.insert(idx,0,-1)
        idx=np.append(idx,n_measurements_sequence-1)



        for i in range(0,len(idx)-1):
            #print(i)
            X_piSeq.append(X_2piSeq[j][idx[i]+1:idx[i+1]+1])
            Y_piSeq.append(Y_2piSeq[j][idx[i]+1:idx[i+1]+1])
            Ph_piSeq.append(Ph_2piSeq[j][idx[i]+1:idx[i+1]+1])
            R_piSeq.append(R_2piSeq[j][idx[i]+1:idx[i+1]+1])
            T_piSeq.append(T_2piSeq[j][idx[i]+1:idx[i+1]+1])
            Z_piSeq.append(Z_2piSeq[j][idx[i]+1:idx[i+1]+1])
            S_piSeq.append(S_2piSeq[j][idx[i]+1:idx[i+1]+1])

    n_sequences=len(X_piSeq)

    return T_piSeq, X_piSeq, Y_piSeq, Z_piSeq, R_piSeq, Ph_piSeq, S_piSeq,n_sequences





def Sequences2pi_SingleWindow(T_data,X_data,Y_data,Z_data,R_data,Ph_data,S_data,n_measurements_window):
    global Dt_max

    D_Phase=np.diff(Ph_data)
    D_Phase_max=1

    D_Time=np.diff(T_data)
    D_Time_max=Dt_max

    index=np.where((abs(D_Phase)>=D_Phase_max) | (abs(D_Time)>=D_Time_max) )
    idx=index[0]
    idx=np.insert(idx,0,-1)
    idx=np.append(idx,n_measurements_window-1)

    X_2piSeq=[]
    Y_2piSeq=[]
    Ph_2piSeq=[]
    T_2piSeq=[]
    R_2piSeq=[]
    Z_2piSeq=[]
    S_2piSeq=[]

    for i in range(0,len(idx)-1):
        #print(i)
        X_2piSeq.append(X_data[idx[i]+1:idx[i+1]+1])
        Y_2piSeq.append(Y_data[idx[i]+1:idx[i+1]+1])
        Ph_2piSeq.append(Ph_data[idx[i]+1:idx[i+1]+1])
        R_2piSeq.append(R_data[idx[i]+1:idx[i+1]+1])
        T_2piSeq.append(T_data[idx[i]+1:idx[i+1]+1])
        Z_2piSeq.append(Z_data[idx[i]+1:idx[i+1]+1])
        S_2piSeq.append(S_data[idx[i]+1:idx[i+1]+1])

    n_sequences=len(X_2piSeq)

    return T_2piSeq, X_2piSeq, Y_2piSeq, Z_2piSeq, R_2piSeq, Ph_2piSeq, S_2piSeq,n_sequences










def removeOutliers(T_window, X_window, Y_window, Z_window, R_window,Ph_window,S_window, n_windows):


    Ph_window_NoOutliers=[]
    X_window_NoOutliers=[]
    Y_window_NoOutliers=[]
    R_window_NoOutliers=[]
    T_window_NoOutliers=[]
    Z_window_NoOutliers=[]
    S_window_NoOutliers=[]


    for j in range(0,n_windows):

        T_NoOutliers, X_NoOutliers, Y_NoOutliers, Z_NoOutliers, R_NoOutliers, Ph_NoOutliers, S_NoOutliers  = removeOutliers_SingleWindow( T_window[j], X_window[j], Y_window[j], Z_window[j], R_window[j],Ph_window[j],S_window[j])

        if len(T_NoOutliers):

            T_window_NoOutliers.append(T_NoOutliers)
            X_window_NoOutliers.append(X_NoOutliers)
            Y_window_NoOutliers.append(Y_NoOutliers)
            Z_window_NoOutliers.append(Z_NoOutliers)
            R_window_NoOutliers.append(R_NoOutliers)
            Ph_window_NoOutliers.append(Ph_NoOutliers)
            S_window_NoOutliers.append(S_NoOutliers)

    n_windows=len(T_window_NoOutliers)

    return T_window_NoOutliers, X_window_NoOutliers, Y_window_NoOutliers, Z_window_NoOutliers, R_window_NoOutliers, Ph_window_NoOutliers,S_window_NoOutliers, n_windows










def removeOutliers_SingleWindow(T_window, X_window, Y_window, Z_window, R_window, Ph_window,S_window):
    global Dt_max

    limit=2

    Ph_window_NoOutliers=[]
    X_window_NoOutliers=[]
    Y_window_NoOutliers=[]
    R_window_NoOutliers=[]
    T_window_NoOutliers=[]
    Z_window_NoOutliers=[]
    S_window_NoOutliers=[]

    D_Phase=np.diff(Ph_window)
    D_Phase_max=2

    D_Time=np.diff(T_window)
    D_Time_max=Dt_max

    seqX=[X_window[0]]
    seqY=[Y_window[0]]
    seqPh=[Ph_window[0]]
    seqR=[R_window[0]]
    seqT=[T_window[0]]
    seqZ=[Z_window[0]]
    seqS=[S_window[0]]

    for i in range(0,len(Ph_window)-1):
        #print(i)


        if (abs(D_Phase[i])>=D_Phase_max) or (abs(D_Time[i])>=D_Time_max):

            if len(seqX)>=limit:

                X_window_NoOutliers=np.hstack((X_window_NoOutliers,seqX))
                Y_window_NoOutliers=np.hstack((Y_window_NoOutliers,seqY))
                Ph_window_NoOutliers=np.hstack((Ph_window_NoOutliers,seqPh))
                T_window_NoOutliers=np.hstack((T_window_NoOutliers,seqT))
                R_window_NoOutliers=np.hstack((R_window_NoOutliers,seqR))
                Z_window_NoOutliers=np.hstack((Z_window_NoOutliers,seqZ))
                S_window_NoOutliers=np.hstack((S_window_NoOutliers,seqS))

            seqX=[]
            seqX=[X_window[i+1]]
            seqY=[]
            seqY=[Y_window[i+1]]
            seqPh=[]
            seqPh=[Ph_window[i+1]]
            seqR=[]
            seqR=[R_window[i+1]]
            seqT=[]
            seqT=[T_window[i+1]]
            seqZ=[]
            seqZ=[Z_window[i+1]]
            seqS=[]
            seqS=[S_window[i+1]]


        else:


            seqX.append(X_window[i+1])
            seqY.append(Y_window[i+1])
            seqPh.append(Ph_window[i+1])
            seqR.append(R_window[i+1])
            seqT.append(T_window[i+1])
            seqZ.append(Z_window[i+1])
            seqS.append(S_window[i+1])

    if len(seqX)>=limit:
        X_window_NoOutliers=np.hstack((X_window_NoOutliers,seqX))
        Y_window_NoOutliers=np.hstack((Y_window_NoOutliers,seqY))
        Ph_window_NoOutliers=np.hstack((Ph_window_NoOutliers,seqPh))
        T_window_NoOutliers=np.hstack((T_window_NoOutliers,seqT))
        R_window_NoOutliers=np.hstack((R_window_NoOutliers,seqR))
        Z_window_NoOutliers=np.hstack((Z_window_NoOutliers,seqZ))
        S_window_NoOutliers=np.hstack((S_window_NoOutliers,seqS))

    return T_window_NoOutliers, X_window_NoOutliers, Y_window_NoOutliers, Z_window_NoOutliers, R_window_NoOutliers, Ph_window_NoOutliers, S_window_NoOutliers












def segments(T_data,X_data,Y_data,Z_data,R_data,Ph_data,S_data,n_measurements):
    global Dt_max

    Dt=np.diff(T_data)

    index=np.where(Dt>Dt_max)
    idx=index[0]
    idx=np.insert(idx,0,-1)
    idx=np.append(idx,n_measurements-1)

    Ph_window=[]
    T_window=[]
    R_window=[]
    Y_window=[]
    X_window=[]
    Z_window=[]
    S_window=[]

    for j in range(0,len(idx)-1):

        #print(j)
        Ph_window.append(Ph_data[idx[j]+1:idx[j+1]+1])
        T_window.append(T_data[idx[j]+1:idx[j+1]+1])
        X_window.append(X_data[idx[j]+1:idx[j+1]+1])
        Y_window.append(Y_data[idx[j]+1:idx[j+1]+1])
        R_window.append(R_data[idx[j]+1:idx[j+1]+1])
        Z_window.append(Z_data[idx[j]+1:idx[j+1]+1])
        S_window.append(S_data[idx[j]+1:idx[j+1]+1])


#        plt.figure()
#        plt.scatter(T_window[j], Ph_window[j])
#        plt.show()

    X_window=[x for x in X_window if x!=[]]
    Y_window=[x for x in Y_window if x!=[]]
    Ph_window=[x for x in Ph_window if x!=[]]
    R_window=[x for x in R_window if x!=[]]
    T_window=[x for x in T_window if x!=[]]
    Z_window=[x for x in Z_window if x!=[]]
    S_window=[x for x in S_window if x!=[]]

    n_windows=len(Ph_window)



    return T_window, X_window, Y_window, Z_window, R_window, Ph_window, S_window, n_windows









def nonlinear_fit(xdata,ydata,zdata,phdata,wdata,start,lamda):


    param =least_squares(theoretical,start,args=(xdata,ydata,zdata,phdata,wdata,lamda),method='trf')
    #print(popt)

    residuals=param.fun
    jac=param.jac
    #residuals=Ph_data-theoretical(XY_data, *popt)

    # calculate rsquare
    ss_res=np.sum(residuals**2)
    ss_tot=np.sum(np.multiply(wdata,(phdata-np.average(phdata,weights=wdata))**2))
    rsquare=1-ss_res/ss_tot
    #print(rsquare)

    #Hessian matrix
    Hessian=np.dot(jac.T,jac)

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

    return param, rsquare, CI







def theoretical(x,xdata,ydata,zdata,phdata,wdata,lamda):


    return (np.sqrt( (x[0]-xdata)**2 + (x[1]-ydata)**2 +(zdata[0]-zdata)**2)/lamda*4*pi+ x[2] -phdata)*wdata







def findStartPoint(xdata,ydata,polarity):

    #polyfit(x,y,1)


    param,f,r,ss=polyfit(xdata,ydata,1)


    theta=pi/2
    R_matrix=np.matrix([ [math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)] ])
    point=np.matrix([[xdata[-1]],[f(xdata[-1])]])
    center_point=np.matrix([[np.median(xdata)],[f(np.median(xdata))]])

    start_point=R_matrix*(point-center_point)+center_point


    start=[start_point[0,0],start_point[1,0],0]

    return start






def remove_static_data(T_window_Unwrapped, X_window_Unwrapped, Y_window_Unwrapped, Z_window_Unwrapped, R_window_Unwrapped, Ph_window_Unwrapped, S_window_Unwrapped, n_windows):

    for i in range(0,n_windows):

        Ph_window_Unwrapped[i]=Ph_window_Unwrapped[i][S_window_Unwrapped[i]==1]
        X_window_Unwrapped[i]=X_window_Unwrapped[i][S_window_Unwrapped[i]==1]
        Y_window_Unwrapped[i]=Y_window_Unwrapped[i][S_window_Unwrapped[i]==1]
        Z_window_Unwrapped[i]=Z_window_Unwrapped[i][S_window_Unwrapped[i]==1]
        R_window_Unwrapped[i]=R_window_Unwrapped[i][S_window_Unwrapped[i]==1]
        T_window_Unwrapped[i]=T_window_Unwrapped[i][S_window_Unwrapped[i]==1]

    return  T_window_Unwrapped, X_window_Unwrapped, Y_window_Unwrapped, Z_window_Unwrapped, R_window_Unwrapped, Ph_window_Unwrapped, n_windows






def polyfit(x,y,degree):


    coeffs=np.polyfit(x,y,degree)
    p=np.poly1d(coeffs)


    residuals=y-p(x)
    ss_res=np.sum(residuals**2)
    ss_tot=np.sum((y-np.mean(y))**2)
    rsquare=1-ss_res/ss_tot

    return coeffs, p, rsquare, ss_res




def add_pi(T_window_Unwrapped,Ph_window_Unwrapped,k_final,Time,Phase,n_windows):

    for i in range(0,n_windows):

        Ph_window_Unwrapped[i],k_final[i] = add_pi_SingleWindow(T_window_Unwrapped[i],Ph_window_Unwrapped[i],k_final[i],Time,Phase)

    return Ph_window_Unwrapped,k_final




def add_pi_SingleWindow(T_window,Ph_window,k_final,T_data,Ph_data):


    Ph_data=Ph_data[np.nonzero(np.in1d(T_data,T_window))[0]]

    Ph_plus=np.mod(Ph_window+pi,2*pi)
    Ph_noplus=np.mod(Ph_window,2*pi)

    if np.sum(np.in1d(Ph_plus,Ph_data))>np.sum(np.in1d(Ph_noplus,Ph_data)):
        Ph_window_Unwrapped=Ph_window+pi
    else:
        Ph_window_Unwrapped=Ph_window

    k_final[0]=0
    k_final[-1]=math.floor(Ph_window_Unwrapped[-1]/(2*pi))

#    plt.figure
#    plt.scatter(T_window,Ph_window_Unwrapped)
#    plt.show()
#    print(k_final)

    return Ph_window_Unwrapped, k_final


def readTable(Table):


    Time = Table[:, 0]
    X_Antenna = 100 * Table[:, 4]
    Y_Antenna = 100 * Table[:, 5]
    Z_Antenna = 100 * Table[:, 6]
    RSSI = Table[:, 3]
    Phase = -Table[:, 2] +2*pi
    Speed=Table[:,8]


    n_measurements=len(Time)


    return Time,X_Antenna,Y_Antenna,Z_Antenna,RSSI,Phase,Speed,n_measurements


def makeTable(Time,X_Antenna,Y_Antenna,Z_Antenna,RSSI,Phase):

    tag_measurements=np.empty(shape=(len(Time),6))

    tag_measurements[:,0]=Time
    tag_measurements[:,1]=X_Antenna
    tag_measurements[:,2]=Y_Antenna
    tag_measurements[:,3]=Z_Antenna
    tag_measurements[:,4]=RSSI
    tag_measurements[:,5]=Phase


    return tag_measurements



#global lamda
#lamda=(100*299.792458/865.7)

global Dt_max
Dt_max=5

global flag_segments
flag_segments=0

global version
version=1

global power
power=1

global flag_pi
flag_pi=0

global flag_k
flag_k=1

global accounted_number
accounted_number=10

global polynomial
polynomial=1



def PhaseUnwrapping(tag_measurements,lamda,polarity):

    global flag_segments
    global Dt_max
    global version
    global flag_pi


    Time,X_Antenna,Y_Antenna,Z_Antenna,RSSI,Phase,Speed,n_measurements=readTable(tag_measurements)


    T_window, X_window, Y_window, Z_window, R_window, Ph_window,S_window, n_windows = segments(Time,X_Antenna,Y_Antenna,Z_Antenna,RSSI,Phase,Speed,n_measurements)

    if flag_pi==0:

        T_window, X_window, Y_window, Z_window, R_window, Ph_window, S_window, n_windows = removeOutliers(T_window, X_window, Y_window, Z_window, R_window,Ph_window,S_window,n_windows)


    T_2piSeq, X_2piSeq, Y_2piSeq, Z_2piSeq, R_2piSeq, Ph_2piSeq, S_2piSeq, n_sequences  = Sequences2pi(T_window, X_window, Y_window, Z_window, R_window, Ph_window, S_window, n_windows)


    T_window_Unwrapped, X_window_Unwrapped, Y_window_Unwrapped, Z_window_Unwrapped, R_window_Unwrapped, Ph_window_Unwrapped, S_window_Unwrapped, n_windows, k_final = unwrap(T_2piSeq, X_2piSeq, Y_2piSeq, Z_2piSeq, R_2piSeq, Ph_2piSeq, S_2piSeq, n_windows,n_sequences)

    if flag_pi==1:
        Ph_window_Unwrapped, k_final = add_pi(T_window_Unwrapped,Ph_window_Unwrapped,k_final,Time,Phase,n_windows)

    T_window_Unwrapped, X_window_Unwrapped, Y_window_Unwrapped, Z_window_Unwrapped, R_window_Unwrapped, Ph_window_Unwrapped, n_windows=remove_static_data( T_window_Unwrapped, X_window_Unwrapped, Y_window_Unwrapped, Z_window_Unwrapped, R_window_Unwrapped, Ph_window_Unwrapped, S_window_Unwrapped, n_windows)


    initial_idx = findInitialSegment(R_window_Unwrapped,n_windows)

    if (flag_segments):
        target_segments = findTargetSegments_Center(initial_idx,n_windows)
    else:
        target_segments=[]

    if version==1:
        T_Unwrapped, X_Unwrapped, Y_Unwrapped, Z_Unwrapped, R_Unwrapped, Ph_Unwrapped = unwrapSegments_1(initial_idx,target_segments,T_window_Unwrapped, X_window_Unwrapped, Y_window_Unwrapped, Z_window_Unwrapped, R_window_Unwrapped, Ph_window_Unwrapped,lamda,polarity)
    elif version==2:
        T_Unwrapped, X_Unwrapped, Y_Unwrapped, Z_Unwrapped, R_Unwrapped, Ph_Unwrapped = unwrapSegments_2(initial_idx,target_segments,T_window_Unwrapped, X_window_Unwrapped, Y_window_Unwrapped, Z_window_Unwrapped, R_window_Unwrapped, Ph_window_Unwrapped,k_final,lamda,polarity)



    tag_unwrapped_measurements=makeTable(T_Unwrapped, X_Unwrapped, Y_Unwrapped, Z_Unwrapped, R_Unwrapped, Ph_Unwrapped)


    return tag_unwrapped_measurements


