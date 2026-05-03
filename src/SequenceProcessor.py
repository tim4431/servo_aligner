import random
import scipy
import numpy
import time
import math
import cmath
from math import sqrt
#import matplotlib.pyplot as plt
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve

#THIS IS THE CODE FOR PREPARING THE DDS SEQUENCE TO DOWNLOAD TO THE FPGA- FILLS GAPS, ETC!

# Define the indexing format of the input sequence!
startTIME = 0
stopTIME  = 2
startVAL  = 1
stopVAL   = 3

def FixHoles(seqin):
	#This code assumes it takes the a sorted array of non- overlapping intervals (at most they have coincident temporal edges), with tstart<tstop, and fills in the gaps with ramps as necessary to get the anticipated behaviour for undefined intervals-- I DO NOT REMEMBER IF THIS BEHAVIOUR IS RETURNING TO THE DEFAULT VALUE, OR REMAINING AT THE CURRENT VALUE # TOCHECK #*)
	outseq=[]
	outseq.append(list(seqin[0]))

	ii=1
	while ii<len(seqin):
		curel=list(outseq[len(outseq)-1])
		if (curel[stopTIME]!=seqin[ii][startTIME]):
			curel[startVAL]=curel[stopVAL]
			curel[startTIME]=curel[stopTIME]
			curel[stopTIME]=seqin[ii][startTIME]
			outseq.append(curel)
		else:
			outseq.append(list(seqin[ii]))
			ii+=1
	return outseq

def FixImplicitSteps(seqin):
	#This codes assumes it takes the output of FixHoles as input, and removes jumps by turning them into sharp, zero - length ramps. The output should be post - processed with FixExplicitSteps!
	outseq=[]
	outseq.append(list(seqin[0]))
	
	ii=1
	while ii<len(seqin):
		curel=list(outseq[len(outseq)-1])
		if curel[stopVAL]!=seqin[ii][startVAL]:
			curel[startVAL]=curel[stopVAL]
			curel[stopVAL]=seqin[ii][startVAL]
			curel[startTIME]=curel[stopTIME]
			outseq.append(curel)
		else:
			outseq.append(list(seqin[ii]))
			ii+=1
	return outseq

def FixExplicitSteps(seqin):
	#This code takes the output of FixImplicitSteps as input, flattens the sequence, and removes explicit steps by shifting the edges forward as necessary. It then returns to the old formatting!
	tmpseq=[]
	for ii in range(len(seqin)):
		tmpseq.extend(list(seqin[ii])) #list might not be necessary but I'm not taking any chances! Hopefully this isn't too slow :)

	for ii in range(1,len(tmpseq)//2):
		if tmpseq[2*ii]<tmpseq[2*ii-2]+((ii)%2):
			tmpseq[2*ii]=tmpseq[2*ii-2]+((ii)%2)
	outseq=[];
	for ii in range(0,len(tmpseq)-1,4):
		outseq.append(tmpseq[ii:ii+4])
	return outseq

def ImposeIF(IFval, seqin):
	#This takes a list, and adds the initial/final values to the beginning of the sequence.
	#it needn't be added at the end because when the sequence ends for a given channel it returns immediately to the beginning to await the next trigger
	#after Imposing the I/F value, we need to rerun FixHoles, FixImplicitSteps, and FixExplicitSteps (or just run this first!)
	outseq=[[0,IFval,0,IFval]]+seqin
	return outseq
	
def DrawLines(seqin): #SOME PLOTTING STUFF FOR FUN!
	plt.close()
	for ii in range(len(seqin)):
		plt.plot([seqin[ii][startTIME],seqin[ii][stopTIME]],[seqin[ii][startVAL],seqin[ii][stopVAL]])

def GenerateFullSeq (seqin, IFval):
	return FixExplicitSteps(FixImplicitSteps(FixHoles(ImposeIF(IFval,seqin))))

if __name__ == '__main__':
#SOME TEST CODE!
	jseq=[[0,0,0,2],[1,10,2,13],[2,2,3,6],[4,30,10,8],[14,9,14,20],[14,9,20,23]]
	tt=FixExplicitSteps(FixImplicitSteps(FixHoles(ImposeIF(4,jseq))))
	print("run DrawLines(tt) to see what the code did!")
	
#DrawLines(tt)
