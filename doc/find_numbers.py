 #!/usr/bin/env python3


import os
import glob
import re
import sys
import csv


path = "./"
header = []
keys=[]
# with open('template.csv', newline='') as inputfile:
# 	r = csv.reader(inputfile,delimiter=';')
# 	header = list(r)
	
# with open('config.txt', newline='') as inputfile:
# 	r = csv.reader(inputfile,delimiter=';')
# 	l = list(r)
# 	keys.extend(l[0])

def file_handler(path,idx,t_range):
    with open(path, newline='') as inputfile:
        r = csv.reader(inputfile,delimiter=',')
        l = []
        print(f'Column: {idx} Range: {t_range}')
        for row in r:
            if row[idx] not in l:
                if(is_integer(row[idx])):
                    l.append(int(row[idx]))
       
       
        print(f'not in list: ')
        for i in range(0,t_range):
            if not(i in l):
                print(f'pos: {i} item: {i}')

        # 


def main(path):
    
    ROOT_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))
    PATH_TO_FILE = ROOT_DIR+'/'+path[0];
    ROW_INDEX = int(path[1])
    RANGE = int(path[2])
    
    if os.path.exists(PATH_TO_FILE):
        file_handler(PATH_TO_FILE,ROW_INDEX, RANGE)
    else:
        print(f'Not Valid: {PATH_TO_FILE}')



	

def is_integer(n):
	try:
		float(n)
	except ValueError:
		return False
	else:
		return float(n).is_integer()
		
if __name__ == "__main__":
    if (len(sys.argv) == 4):
        main(sys.argv[1:])
    else:
        print(f'not enough arguments: {len(sys.argv)}')

