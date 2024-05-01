#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 26 20:32:41 2020

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
import PhaseUnwrapping



def PhaseReLock3D(tdata,xdata,ydata,zdata,rdata,phdata,ndata,lamda,polarity):

    start=findStartPoint_3D(xdata,ydata,zdata,ndata,polarity)
#    print(start)
    param,rsquare,CI=nonlinear_fit_3D(xdata,ydata,zdata,phdata,start,lamda)

    return param.x[0], param.x[1], param.x[2],rsquare,CI



def findStartPoint_3D(xdata,ydata,zdata,ndata,polarity):



    max_idx=ndata.index(max(ndata))

    param,f,r,ss=PhaseUnwrapping.polyfit(xdata[max_idx],ydata[max_idx],1)


    theta=pi/2
    R_matrix=np.matrix([ [math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)] ])
    point=np.matrix([[xdata[max_idx][-1]],[f(xdata[max_idx][-1])]])
    center_point=np.matrix([[np.median(xdata[max_idx])],[f(np.median(xdata[max_idx]))]])
    start_point=R_matrix*(point-center_point)+center_point

    z=[]
    for i in range(0,len(ndata)):
        z.append(zdata[i][0])


    start=[start_point[0,0],start_point[1,0],np.mean(z)]

    for i in range(0,len(ndata)):

        start.append(0)


    return start



#def polyfit(x,y,degree):
#
#
#    coeffs=np.polyfit(x,y,degree)
#    p=np.poly1d(coeffs)
#
#
#    residuals=y-p(x)
#    ss_res=np.sum(residuals**2)
#    ss_tot=np.sum((y-np.mean(y))**2)
#    rsquare=1-ss_res/ss_tot
#
#    return coeffs, p, rsquare, ss_res


def evaluate_side(xdata,ydata,ndata,x_est,y_est):

    max_idx=ndata.index(max(ndata))

    param,f,r,ss=PhaseUnwrapping.polyfit(xdata[max_idx],ydata[max_idx],1)

    end_point=[xdata[max_idx][-1],f(xdata[max_idx][-1])]
    center_point=[np.median(xdata[max_idx]),f(np.median(xdata[max_idx]))]

    value=np.sign((end_point[0]-center_point[0])*(y_est-center_point[1]) - (x_est-center_point[0])*(end_point[1]-center_point[1]))

    return value




def nonlinear_fit_3D(xdata,ydata,zdata,phdata,start,lamda):


    param=least_squares(theoretical_3D,start,args=(xdata,ydata,zdata,phdata,lamda),method='trf')

    residuals=param.fun
    jac=param.jac
    #residuals=Ph_data-theoretical(XY_data, *popt)

    # calculate rsquare
    ss_res=np.sum(residuals**2)
    ss_tot=0
    for i in range(0,len(phdata)):
        ss_tot=ss_tot+np.sum((phdata[i]-np.mean(phdata[i]))**2)

    rsquare=1-ss_res/ss_tot
#    print(rsquare)

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


def theoretical_3D(x,xdata,ydata,zdata,phdata,ldata):


    F=[]

    for i in range(0,len(xdata)):

        fun= np.sqrt( (x[0]-xdata[i])**2 + (x[1]-ydata[i])**2 +(x[2]-zdata[i])**2)/ldata[i]*4*pi+ x[i+3] - phdata[i]
        F=np.hstack((F,fun))

    return F


def findStartPoint_2D(xdata,ydata,ndata,polarity):

    #polyfit(x,y,1)


    param,f,r,ss=PhaseUnwrapping.polyfit(xdata,ydata,1)


    theta=pi/2
    R_matrix=np.matrix([ [math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)] ])
    point=np.matrix([[xdata[-1]],[f(xdata[-1])]])
    center_point=np.matrix([[np.median(xdata)],[f(np.median(xdata))]])

    start_point=R_matrix*(point-center_point)+center_point


    start=[start_point[0,0],start_point[1,0],0]

    return start


def PhaseReLock2D(tdata,xdata,ydata,zdata,rdata,phdata,ndata,lamda,polarity):

    start=findStartPoint_2D(xdata,ydata,ndata,polarity)
    #print(start)
    param,rsquare,CI = nonlinear_fit_2D(xdata,ydata,zdata,phdata,start,lamda)

    return param.x[0], param.x[1], zdata[0],rsquare,CI

def nonlinear_fit_2D(xdata,ydata,zdata,phdata,start,lamda):

    param=least_squares(theoretical_2D,start,args=(xdata,ydata,zdata,phdata,lamda),method='trf')

    residuals=param.fun
    jac=param.jac
    #residuals=Ph_data-theoretical(XY_data, *popt)

    # calculate rsquare
    ss_res=np.sum(residuals**2)
    ss_tot=np.sum((phdata-np.mean(phdata))**2)
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


def theoretical_2D(x,xdata,ydata,zdata,phdata,lamda):


    return np.sqrt( (x[0]-xdata)**2 + (x[1]-ydata)**2 +(zdata[0]-zdata)**2)/lamda*4*pi+ x[2] -phdata


def create_data(tag_measurements,reader_antenna_lamda,reader_antenna_polarities):

    tdata_unwrapped=[]
    xdata_unwrapped=[]
    ydata_unwrapped=[]
    zdata_unwrapped=[]
    rdata_unwrapped=[]
    phdata_unwrapped=[]
    ldata_unwrapped=[]
    pdata_unwrapped=[]

    ndata_unwrapped=[]

    for i in range(0,len(tag_measurements)):
        for j in range(0,len(tag_measurements[i])):

            if len(tag_measurements[i][j]) >= 5:

                tdata_unwrapped.append(tag_measurements[i][j][:,0])
                xdata_unwrapped.append(tag_measurements[i][j][:,1])
                ydata_unwrapped.append(tag_measurements[i][j][:,2])
                zdata_unwrapped.append(tag_measurements[i][j][:,3])
                rdata_unwrapped.append(tag_measurements[i][j][:,4])
                phdata_unwrapped.append(tag_measurements[i][j][:,5])
                ldata_unwrapped.append(reader_antenna_lamda[i][j])
                pdata_unwrapped.append(reader_antenna_polarities[i][j])

                ndata_unwrapped.append(len(tag_measurements[i][j]))

    return tdata_unwrapped, xdata_unwrapped, ydata_unwrapped, zdata_unwrapped, rdata_unwrapped, phdata_unwrapped, ndata_unwrapped, ldata_unwrapped,  pdata_unwrapped




def PhaseReLock(tag_measurements,reader_antenna_lamda,reader_antenna_polarities):


    tdata_unwrapped, xdata_unwrapped, ydata_unwrapped, zdata_unwrapped, rdata_unwrapped, phdata_unwrapped, ndata_unwrapped, ldata_unwrapped, polarities=create_data(tag_measurements,reader_antenna_lamda,reader_antenna_polarities)



    if len(tdata_unwrapped)>=2:
        xtag,ytag,ztag,rsquare,CI=PhaseReLock3D(tdata_unwrapped,xdata_unwrapped,ydata_unwrapped,zdata_unwrapped,rdata_unwrapped,phdata_unwrapped,ndata_unwrapped,ldata_unwrapped,polarities)

    elif len(tdata_unwrapped)==1:
        xtag,ytag,ztag,rsquare,CI=PhaseReLock2D(tdata_unwrapped[0],xdata_unwrapped[0],ydata_unwrapped[0],zdata_unwrapped[0],rdata_unwrapped[0],phdata_unwrapped[0],ndata_unwrapped[0],ldata_unwrapped[0],polarities[0])
#        print(xtag,ytag,ztag)

    else:
      return [-666, -666, -666]

    value=evaluate_side(xdata_unwrapped,ydata_unwrapped,ndata_unwrapped,xtag,ytag)

    if value!=1:
        print('wrong side')


    return [xtag/100,ytag/100,ztag/100]
