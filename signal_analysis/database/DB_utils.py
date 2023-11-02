#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 13:28:10 2023

@author: keenan
"""

import json, os



db_path = '/home/keenan/Documents/rice_device/signal_analysis/database/'

def read_most_recent_json_file_from(folder_path):
    
    full_folder_path = db_path + folder_path + "/"
    
    files_in_folder = [name[:-5] for name in os.listdir(full_folder_path)]
    
    most_recent_file = full_folder_path + max(files_in_folder) + ".json"

    with open(most_recent_file, 'r') as json_file:
        samples = json.load(json_file)
        
    return samples

def read_json_file(file_path):
    
    full_file_path = db_path + file_path + ".json"

    with open(full_file_path, 'r') as json_file:
        samples = json.load(json_file)
        
    return samples



if __name__ == "__main__": # Just for debugging functions in DB_utils.py

    try:
        from IPython import get_ipython
        get_ipython().magic('clear')
        get_ipython().magic('reset -f')
    except:
        pass
    
    read_most_recent_json_file_from('accelerometer')