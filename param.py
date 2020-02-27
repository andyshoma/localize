#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

from scipy.stats import norm

PATH = "/Users/andy/Wifi_rtt/analize/offset/"
READ_FILE_NAME = "full"
WRITE_FILE_NAME = "rewrite"
CSV = ".csv"

def main():
    '''error = []
    for num in range(20):
        with open(PATH + str(num + 1) + CSV, 'r') as fr:
            fr.readline()
            for line in fr:
                data = line.split(',')
                error.append(float(data[1].rstrip('\n')))

    mean = np.mean(error)
    sigma = np.var(error)'''

    mean = 0

    sigma = 10000

    sigma1 = 554220.0583
    sigma2 = 286763.5153
    sigma3 = 1526865.125
    #sigma4 = 387539.393

    fig = plt.figure()
    linex = np.arange(-6000, 6000, 0.1)

    est = norm(loc=mean, scale=np.sqrt(sigma))
    plt.plot(linex, est.pdf(linex), color='red')

    '''
    est1 = norm(loc=mean, scale=np.sqrt(sigma1))
    plt.plot(linex, est1.pdf(linex), color='red')

    est2 = norm(loc=mean, scale=np.sqrt(sigma2))
    plt.plot(linex, est2.pdf(linex), color='blue')

    est3 = norm(loc=mean, scale=np.sqrt(sigma3))
    plt.plot(linex, est3.pdf(linex), color='green')

    #est4 = norm(loc=mean, scale=np.sqrt(sigma4))
    #plt.plot(linex, est4.pdf(linex), color='black')
    '''

    plt.show()

    '''with open(PATH + WRITE_FILE_NAME + CSV, 'a') as fw:
        fw.write(READ_FILE_NAME + ',' + str(mean) + ',' + str(sigma) + '\n')'''
        
if __name__ == '__main__':
  main()
