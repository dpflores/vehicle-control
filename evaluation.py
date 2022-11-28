#!/usr/bin/python3

# grading script given to learners, this file should work in both Python 2 and 3

from __future__ import division
import sys
import numpy as np
import matplotlib.pyplot as plt

def l2_squared(v, w):
    '''
    Compute the squared distance between 2 1D vectors v and w
    Output D is a NxM matrix, where N = dim(v) and M = dim(w)
    D(n,m) = (v_n - w_m)^2
    '''
    # shape v into a column, w into a row    
    v = v.reshape([-1,1])
    w = w.reshape([1,-1])
    
    # duplicate v and w to vectorize distance calculation
    v_mat = np.tile(v, w.shape[0])
    w_mat = np.tile(w, (v.shape[0], 1))
    
    # compute the distance and return
    D = (v_mat - w_mat)**2
    return D


def grade(waypoints, solution, visualize = False):
    '''
    Compares a set of solution data to waypoint data
    solution is a Mx4 matrix, each row contains [x,y,v,t]
    waypoints is a Nx3 matrix, each row contains [x,y,v]
    
    Solution passes if:
    the path is close to waypoints
    speed at the points closest to the path are close to waypoint speeds
    pass conditions are set by a distance and speed threshold and % of 
    waypoints correct
    
    Returns a dictionary containing the solution, waypoints, and performance
    performance contains: 
        nearest distance to waypoints
        index of points on solution path closest to waypoints
        speeds at the closest points
        whether the solution passes or fails
        
    Visualizing the solution shows the solution path and speed at waypoints
    '''
    N = waypoints.shape[0]

    results = {
    'd_thresh' : 0.2,           # distance to waypoints
    'v_thresh' : 0.22,           # speed difference at waypoints
    'pass_percentage': 50,    # percentage of correct waypoints required
    'solution' : solution,
    'waypoints': waypoints,
    'dists2'   : np.zeros((N,1)),
    'vels'     : np.zeros((N,1)),
    'inds'     : np.zeros((N,1), dtype='int32'),
    'pass_dist': True, 
    'pass_vel' : True}

    # compute the distance from the waypoints to the solution
    results['dists2'] = l2_squared(waypoints[:,0], solution[:,0]) + l2_squared(waypoints[:,1], solution[:,1])
    
    results['inds'] = np.argmin(results['dists2'], 1)    
    results['dists2'] = np.min(results['dists2'], 1)  
    results['vels'] = solution[results['inds'], 2].reshape([-1,1])

    # check the distance and speed at waypoints, find path locations that satisfy both
    dists_correct = results['dists2']**0.5 <= results['d_thresh']
    vels_correct = results['vels'][dists_correct,0] - results['waypoints'][dists_correct,2] 
    vels_correct = np.abs(vels_correct) <= results['v_thresh']
    percent_correct = (np.sum(vels_correct)/N)*100
    
    if percent_correct < results['pass_percentage']:
        print('Assessment failed, only {0:.2f}% of waypoints completed. {1:.2f}% required to pass'.format(percent_correct, results['pass_percentage']))
    else:
        print('Assessment passed!, {0:.2f}% of waypoints completed.'.format(percent_correct))

    if visualize:
        display_path(results)
    
    return results


def display_path(results):
    '''
    Generates 2 plots from the waypoints and solution
    
    1) Plot the path and waypoints, show distance tolerance circles
    2) Plot speed at waypoints and path speed, show speed tolerance bounds
    '''
    
    fig, ax = plt.subplots()
    plt.plot(results['waypoints'][:,0], results['waypoints'][:,1], '*', label='puntos de trayectoria')
    plt.plot(results['solution'][:,0], results['solution'][:,1], label='trayectoria real')
    for i in range(results['waypoints'].shape[0]):
        center = (results['waypoints'][i,0],results['waypoints'][i,1])
        circle = plt.Circle(center, results['d_thresh'], color='g', fill=False)
        ax.add_artist(circle)

    x0 = results['waypoints'][:,0][0]
    y0 = results['waypoints'][:,1][0]

    x1 = results['waypoints'][:,0][-1]
    y1 = results['waypoints'][:,1][-1]

    ax.plot([x0],[y0],marker="o", markersize=7, markeredgecolor="red", markerfacecolor="red")
    ax.annotate('A', (x0+0.5, y0))
    ax.plot([x1],[y1],marker="o", markersize=7, markeredgecolor="blue", markerfacecolor="blue")
    ax.annotate('B', (x1-0.2, y1))
    
    ax.invert_xaxis()   
    plt.legend(fontsize=12)
    plt.xlabel(r'posici\'on x (m)', fontsize=12)
    plt.ylabel(r'posici\'on y (m)', fontsize=12)
    # plt.title('Treyectoria real y trayectoria deseada', fontsize=16)
    
    plt.subplots()
    plt.plot(results['waypoints'][:,2], label='velocidad de referencia')
    plt.plot(results['solution'][results['inds'],2], label=r'velocidad del veh\'iculo')
    plt.plot(results['waypoints'][:,2] + results['v_thresh'], '--', color = 'C2')
    plt.plot(results['waypoints'][:,2] - results['v_thresh'], '--', color = 'C2')
    
    plt.legend(fontsize=12)
    plt.xlabel('\# de punto de trayectoria', fontsize=12)
    plt.ylabel('Velocidad (m/s)', fontsize=12)
    # plt.title('Perfiles de velocidad', fontsize=16)
    

    plt.subplots()
    plt.plot(results['solution'][:,3],2*np.ones_like(results['solution'][:,3]), label='velocidad de referencia')
    plt.plot(results['solution'][:,3], results['solution'][:,2], label=r'velocidad del veh\'iculo')
    
    plt.legend(fontsize=12)
    plt.xlabel('tiempo (s)', fontsize=12)
    plt.ylabel('Velocidad (m/s)', fontsize=12)
    # plt.title('Respuesta de velocidad', fontsize=16)

    plt.show(block=True)


    

if __name__ == '__main__':

    # For latex rendering
    try:
        plt.rcParams.update({
            "text.usetex": True,
            "font.family": "DejaVu Sans"
        })
    except:
        pass
    
    files = sys.argv
    waypoint_file = files[1]
    waypoints = np.genfromtxt(waypoint_file, delimiter=',')
    
    solution_file = files[2]
    solution = np.genfromtxt(solution_file, delimiter=',')
    
    results = grade(waypoints, solution, True)
