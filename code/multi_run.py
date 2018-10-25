#!/usr/bin/python

# 
# This script written by Mgwoo.
# 2018.02.22.
# 
# mgwoo@unist.ac.kr

import os
import sys
import subprocess as sp
import time
import argparse
from multiprocessing import Process, Pool
from datetime import datetime


benchDirList = [ "../bench", "../bench_big", "../bench_new" ]
dirpos =  "../bench_official"
binaryName = "./bus_router"
evalpos = "../eval/eval_1.0-a8"
evaluator = "eval"
outpos = "../output"
logpos = "../log"
outDir = ""
logDir = ""

def Tokenizer( line, delim ):
    vals = line.split()
    vals = [ elem for elem in vals if elem not in delim ]
    return vals


def ReadScore( outputFile ):
    inFile = open(outputFile, 'r')
    lines = inFile.readlines()
    score = {}
    target = [ 'CR', 'Ps', 'Pf', 'cost' ]

    for i in range(len(lines)):
        line = lines[i]
        delim = [ '[', ']', '=', '+', '*' ]
        vals = Tokenizer(line, delim)

        if vals[2] in target:
            score[vals[2]] = vals[len(vals)-1]

    return score

def ExecuteBinary( benchName ):
    resultDic = {}
    benchName = benchName.split('.')[0]

    runCommand = "%s %s/%s.input %s/%s.out > %s/%s.log" % (binaryName, dirpos, benchName, outDir, benchName, logDir, benchName)
    startTime = time.time()
    ExecuteCommand(runCommand)
    endTime = time.time()
    runTime = endTime - startTime

    evalCommand = "%s/%s %s/%s.input %s/%s.out > %s/%s.eval" % (evalpos, evaluator, dirpos, benchName, outDir, benchName, logDir, benchName)
    ExecuteCommand(evalCommand)
    
    evalFileName = "%s/%s.eval" % (logDir, benchName)
    score = ReadScore(evalFileName)

    resultDic['Bench'] = benchName
    for key, value in score.items():
        resultDic[key] = value
    resultDic['RT'] = runTime
    return resultDic

def ExecuteBinarySingle( benchName ):
    runCommand = "%s %s/%s.input %s/%s.out | tee %s/%s.log" % (binaryName, dirpos, benchName, outDir, benchName, logDir, benchName)
    ExecuteCommand(runCommand)
    evalCommand = "%s/%s %s/%s.input %s/%s.out > %s/%s.eval" % (evalpos, evaluator, dirpos, benchName, outDir, benchName, logDir, benchName)
    ExecuteCommand(evalCommand)

def ExecuteCommand( curCmd ):
    print( curCmd )
    sp.call( curCmd , shell=True)

if __name__ == '__main__':
    if len(sys.argv) <=1:
        print("usage:   ./multi_run.py <benchname or number>")
        print("Example: ")
        print("         ./multi_run.py 4 ")
        print("         ./multi_run.py example_2 ")
        sys.exit(1)


    benchNum = -1
    benchName = ""
    execProcs = []
    evalProcs = []
    evalFiles = []
    benchList = sorted(os.listdir(dirpos))


    runAll = False
    if sys.argv[1].isdigit():
        benchNum = int(sys.argv[1])
        benchName = benchList[benchNum]
    elif sys.argv[1] == "all":
        runAll = True
    else:
        benchName = sys.argv[1]

    #curTime = datetime.now().strftime('%Y-%m-%d_%H:%M:%S')
    curTime = datetime.now().strftime('%m_%d_%H_%M_%S')
    logDir = "%s/%s" % (logpos, curTime)
    outDir = "%s/%s" % (outpos, curTime)
    os.makedirs(logDir)
    os.makedirs(outDir)
    
    # read benchmark list
    if runAll == False: 
        benchList = []
        benchList.append(benchName)

    if len(benchList) != 1:
        # multiprocess start
        pool = Pool(processes=len(benchList))
        dics = pool.map(ExecuteBinary, [bn for bn in benchList])
        pool.close()

        keys = ['Bench', 'CR', 'Ps', 'Pf', 'cost', 'RT']
        summary = open("%s/summary.txt" % (logDir), "w")

        for dic in dics:
            for key in keys:
                if key == 'Bench':
                    summary.write("< %s >\n" % (dic[key]))
                elif key == 'RT':
                    summary.write("%4s :    %3.4f(s)\n" % (key, float(dic[key])))
                else:
                    summary.write("%4s :    %7s\n" % (key, dic[key]))
            summary.write("\n\n")
    
    else:
        # run single
        ExecuteBinarySingle( benchList[0] )
    

    print("Done")

