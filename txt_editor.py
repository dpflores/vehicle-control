
import pandas as pd
import numpy as np

def remove_column(df, index):
    df.drop(index, inplace=True, axis=1)

def append_constant_column(df, constant):
    df.shape[1]
    df[df.shape[1]+1] = constant*np.ones(df.shape[0])

def write_file(df, filename):
    df.to_csv(filename, header=None, index=None, sep=',', mode='w')


df = pd.read_csv('racetrack_waypoints.txt', sep=",", header=None)
remove_column(df,2)
append_constant_column(df, 35)
write_file(df,'constant_speed.txt')
