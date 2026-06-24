from utilities.util import *
try:
  import matplotlib.pyplot as plt
except:
  pass
import colorsys
import socket
import numpy as np
import math

import coloredlogs, logging
# Create a logger object.
logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG')

#identity tranformation, used as the default for Channel transformations
def id_trans(x):
  return x

class SetError(Exception):
  def __init__(self, msg):
    self.msg = msg
  def __str__(self):
    return repr(self.msg)
    
########################################################################
#========================= Sequence Class =============================#
########################################################################
class Sequence:
  def __init__(self, name, host, port, max_channels=32, seq_type="REGULAR", graph=1):
    # Sequence properties
    self.allChannels  = [None]*max_channels   # Array of all channels. IMPORTANT
    self.fullChannels = 0                     # Channels being used. always increments.
    self.max_channels = max_channels          # Maximum channels device can support
    self.name         = name                  # Sequence name
    # Server info
    self.IP       = str(host) # Server IP address
    self.port     = port      # Server port number
    self.seq_type = seq_type  # Server file name for plot import purpose
    # Run info and timing
    self.runname     = ""          # Run name contain loop run counter
    self.foldername  = ""          # Name of the folder to save data
    self.TIME_START  = 0.0         # Start time
    self.TIME_STOP   = 0.0         # Stop time
    # Plot
    self.graph = graph
    self.xtick_labels = ["t="+str(0)]
    self.xtick_locations = [0]
    self.channelsToGraph = [] # list of channels which will be graphed on command...\

    try:
      socket.inet_aton(host)
    except socket.error:
      printError("Error with IP: '"+str(host)+"' not a valid address.")
      
  def newChannel(self, the_id, name, steady_state_value=0, max_value=1, graph=0, transform_t=id_trans, transform_v=id_trans, system='None', ctype='Normal', master=False):
    name = str(name)
    
    if self.allChannels[the_id] != None:
      printError("Error creating new channel '"+name+"' (id "+str(the_id)+" in use)")
      return -1      
    if the_id >= self.max_channels:
      printError("Error creating new channel '"+name+"' (channels are full)")
      return -1
    elif len(name) <= 1 or len(name) >= 31:
      printError("Error creating new channel '"+name+"' (name length)")
      return -1
    elif name.lower() == "keith":
      printError("Error creating new channel '"+name+"' (pick a better name)")
      return -1
    elif self.getChannelByName(name):
      printError("Error creating new channel '"+name+"' (name in use)")
      return -1
      
    newChannel = Channel(self, the_id, name, 
                         steady_state_value, max_value, graph, 
                         transform_t, transform_v, 
                         system, ctype, master)
    self.allChannels[the_id] = newChannel
    self.fullChannels += 1

    return newChannel
    
  def getChannelByName(self, name):
    for chan in self.allChannels:
      if chan == None:
        continue
      if chan.name == name:
        return chan
    return None
    
  def getChannelById(self, theid):
    for chan in self.allChannels:
      if chan == None:
        continue
      if chan.chanid == theid:
        return chan
    return None
  
  def getChannelIdByName(self, name):
    for chan in self.allChannels:
      if chan == None:
        continue
      if chan.name == name:
        return chan.id
    return -1

  def VariableName(self):
    return "".join(self.name.split(' '))+'_'  
    
#======================================================================#
# Pause until Enter is pressed                                         #
#======================================================================#
  def Pause(self, msg='Press enter to continue...'):
    return input(msg + " ").strip()

#======================================================================#
# Verify intervals for all channels                                    #
#======================================================================#
  def VerifyIntervals(self):
    badchannels = []
    for chan in self.allChannels:
      if chan == None:
        continue
      if chan.VerifyIntervals(verbose=0) > 0:
        badchannels.append(chan.name)
    if len(badchannels) == 0:
      printComment("Channel '" + str(self.name) + "' is defined through t=" + str(lastT) + ".")
    else:
      printError("The following channels are not specified all the way through: " + ', '.join(badchannels))
    return len(badchannels)
    
#======================================================================#
# Print all channels (with 3 different levels of verbosity)            #
#======================================================================# 
  def Print(self, verbose=0):
    for chan in self.allChannels:
      if chan == None:
        continue
      chan.Print(verbose)

#======================================================================#
# Add a time tick to be printed on subsequent plots                    #
#======================================================================#
  def newTimeTick(self, listofticks):
    if type(listofticks) is tuple:
      listofticks = [listofticks]
    for tick_tuple in listofticks:
      if type(tick_tuple) is not tuple:
        printError("Error adding tick '"+str(tick_tuple)+": should be a tuple of the form ('name', time)")
        return -1
      if type(tick_tuple[0]) is not str or type(tick_tuple[1]) is str:
        printError("Error adding tick '"+str(tick_tuple)+": should be of the form ('name', time)")
        return -1
      for tick_time in self.xtick_locations:
        if tick_time == tick_tuple[1]:
          printError("Error adding tick '"+str(tick_tuple)+": time is in use")
          return -1
    for tick_tuple in listofticks:
      self.xtick_labels.append(tick_tuple[0])
      self.xtick_locations.append(tick_tuple[1])

#======================================================================#
# Remove the first time tick with a particular name                    #
#======================================================================#
  def removeTimeTick(self, listofnames):
    if type(listofnames) is str:
      listofnames = [listofnames]
    for tick_name in listofnames:
      for ii in range(0, len(self.xtick_labels)):
        if self.xtick_labels[ii] == tick_name:
          self.xtick_labels.pop(ii)
          self.xtick_locations.pop(ii)
          break

#======================================================================#
# Plot a channel (or single channel) and save it                       #
#======================================================================#
  def Plot(self, single_chan=None, filename='saved/last_sequence.png'):
    plt.rc("font", family="serif")
    plt.rc("font", size=12)
    almost_black = '#262626'
    plt_ht = 7 if single_chan == None else 3
    plt_wd = 16
    fig = plt.figure(figsize=(plt_wd, plt_ht), dpi=80, facecolor='#e0e0e0')
    ax = plt.axes()
    ax.title.set_color(almost_black)
    plt_title = self.name if single_chan == None else self.name+": "+single_chan.name
    ax.set_title(plt_title, {'size': 20})
    plt.grid()
    ax.set_xlabel(r"Time ($\mu s$)", {'size': 14}, labelpad=20)
    ax.xaxis.label.set_color(almost_black)
    ax.yaxis.label.set_color(almost_black)
    ytick_labels = []
    ytick_locations = []
    t = 0
    x = [0]
    greatestX = 0
   
    chancounter = 0  # for y-offset
    
    if single_chan != None:
      channelstograph = [single_chan]
    else:
      channelstograph = self.allChannels    
    
    for chan in channelstograph:
      if chan == None or len(chan._UserValues) == 0:
        continue
      rgb = colorsys.hls_to_rgb(1.0 * chan.chanid / self.fullChannels, .4, .8)
      rgb = '#%02x%02x%02x' % (int(255*rgb[0]), int(255*rgb[1]), int(255*rgb[2]))  # generate rgb based on position
      y = []
      x = []
      for ii in range(0, len(chan._UserValues)):
        pair = chan._UserValues[ii]
        x.append(pair[0])
        y.append(chancounter + .7*pair[1]/chan.max_v)
        x.append(pair[2])
        if pair[2] > greatestX:
          greatestX = pair[2]
        y.append(chancounter + .7*pair[3]/chan.max_v)
        if ii < len(chan._UserValues)-1:
          x.append(chan._UserValues[ii+1][0])
          y.append(chancounter + .7*pair[3]/chan.max_v)
      line1 = plt.plot(x, y)                                # plot the line!
      ax.fill_between(x, y, chancounter, color='#efefef')   # shade in the region
      plt.setp(line1, linewidth=1.25, color=rgb)
      ytick_locations.append(chancounter)
      ytick_labels.append(chan.name)
      chancounter += 1  # update y-offset

    xtick_locations = self.xtick_locations
    xtick_labels = self.xtick_labels
    if greatestX not in xtick_locations:
      xtick_locations.append(greatestX)
      xtick_labels.append("t="+str(greatestX))
    ax.set_xticks(xtick_locations)
    ax.set_yticks(ytick_locations)
    plt.tick_params(axis='both', which='major', labelsize=11)
    ax.set_yticklabels(ytick_labels)
    ax.set_xticklabels(xtick_labels)
    # ax.yaxis.grid(False)
    margin = (greatestX) / 40.0  # space before and after start
    plt.ylim(-.5, max(y)+.5)
    plt.xlim(0 - margin, greatestX + margin)

    plt.tight_layout()
    #fig.savefig(filename, facecolor=fig.get_facecolor(), edgecolor='none')
    plt.show()
    
#======================================================================#
# Reset the sequence information                                       #
#======================================================================#
  def Reset(self):
    # Reset run info and timing
    self.runname    = ''
    self.foldername = ''
    self.TIME_START = 0.
    self.TIME_STOP  = 0.
    for chan in self.allChannels:
      if chan != None:
        chan._UserValues  = []
        chan._TransValues = []

#======================================================================#
# Add a ramp to the final value at the end of each channel             #
#======================================================================#
  def SetFinalRamps(self,end_time):
    for chan in self.allChannels:
      if chan != None:
        if len(chan._UserValues) != 0:
          lastVal = chan.GetLastValue()
          lastTime = chan.GetLastTime()       
          if (end_time > lastTime):
            chan.Set([(lastTime, lastVal, end_time, lastVal)]) # Hold at final value!
        else:#no values set yet
          val = chan.ssv
          chan.Set([(0, val, end_time, val)])

#======================================================================#
# Add a ramp at the beginning of each channel                          #
#======================================================================#
  def SetInitRamps(self, ramp_time, start_time=0):
    for chan in self.allChannels:
      if chan != None:
        chan.ShiftTime(ramp_time)

        lastVal = chan.GetPreviousValue()
        firstVal = chan.GetFirstValue()
        
        chan.Set([(start_time, lastVal, ramp_time+start_time, firstVal)])

#======================================================================#
# Define missing ramps                                                 #
#======================================================================#
  def FillMissingRamps(self):
    #Add a ramp to the steady state value for unprogrammed channels
    for chan in self.allChannels:
      if chan != None:
        if len(chan._UserValues) == 0:
          chan.Set([(0, chan.ssv, 0, chan.ssv)])
          print("\""+chan.name+"\" has no ramps. Setting to steady state value of "+str(chan.ssv))
    #Add a ramp to hold the final value    
    
########################################################################
#======================================================================#
#========================== Channel Class =============================#
#======================================================================#
########################################################################
class Channel:    
  def __init__(self, seq, id, name, 
               ssv, max_value=5, graph=0, 
               transform_t=id_trans, transform_v=id_trans, 
               system='None', chan_type='Normal', master_chan=None):
    self.name      = str(name)
    self.seq       = seq
    self.chanid    = int(id)
    self.cat       = str(system) # Sub-system in which the channel is used. This is useful in putting the channels in tab during set SSV.
    self.chantype  = str(chan_type) # This can be set to slave
    self.max_v     = float(max_value)
    self.previous_value = ssv # memory of the previous run
    self.master    = master_chan

    self._transfunc_t = transform_t # A function of the form t' = f(t)
    self._transfunc_v = transform_v # A function of the form v' = f(v)
    self._UserValues  = []          # Channel's value at a series of points (directly input by user, in user-prefered scale)
    self._TransValues = []          # Values after transforming to hardware-prefered scales, these are to be used by servers
    
    self.ssv              = float(ssv)                  # Steady state value
    self._transformed_SSV = self._transfunc_v(self.ssv) # Transformed SSV

    # Add channel to channelsToGraph in the parent sequence
    if graph > 0:
      self.seq.channelsToGraph.append(self)

  def VariableName(self):
    return "".join(self.name.split(' '))+'_'
    
  def SelfName(self):
    return str(self)

  #======================================================================#
  # Set a series of linear (or constant) ramps.                          #
  #  - Expects a list of tuples: (t_start, V_start, t_stop, V_stop)      #
  #======================================================================#
  def Set(self, vals):
    for pair in vals:
      if type(pair) is int or type(pair) is float or type(pair) is str or len(pair) != 4:
        raise SetError("Invalid Set() for "+str(self.name)+": "+str(pair)+". Value should be (t_start, t_stop, V_start, V_stop).")
      pair = Interval(pair)
      if pair.start_t() > pair.end_t():
        raise SetError("Invalid Set() for "+str(self.name)+": "+str(pair)+". Time_start must be <= time_stop.")
      if abs(pair.start_V()) > self.max_v:
        raise SetError("Invalid Set() value for "+str(self.name)+": "+str(pair)+". Starting value exceeds channel's maximum.")
      if abs(pair.end_V()) > self.max_v:
        raise SetError("Invalid Set() value for "+str(self.name)+": "+str(pair)+". Ending value exceeds channel's maximum.")
      # Pair is valid, so append it
      self._UserValues.append(pair)
      # Apply the transformation and append the hardware-scale values
      t1 = self._transfunc_t(pair[0])
      t2 = self._transfunc_t(pair[2])
      v1 = self._transfunc_v(pair[1])
      v2 = self._transfunc_v(pair[3])
      newpair = Interval((t1, v1, t2, v2))
      self._TransValues.append(newpair)
    # SSV fix from ash
    if(self._UserValues):
      self.seq.TIME_STOP = max(self.seq.TIME_STOP, self._UserValues[-1][2])

  def SetInterval(self, timeInterval, *args):
    # *args is a list of either one or two values giving the start and end values
    if (len(args)==1):
      startV = args[0]
      endV = startV
    elif (len(args)==2):
      startV = args[0]
      endV = args[1]
    else:
      raise SetError('Invalid value in SetInterval for '+str(self.name)+'. Must specify one or two values in SetInterval().')
    
    try:
      startT, endT = timeInterval._start, timeInterval._stop
      self.Set([(startT,startV,endT,endV)])
    except:
      logger.exception('Invalid SetInterval() for '+str(self.name)+'.')
      raise SetError('Invalid SetInterval() for '+str(self.name)+'.')

  def SetLogRamp(self, timeInterval, v0, v1, sample_rate=0.04):
    try:
      startT, endT = timeInterval._start, timeInterval._stop
      N_sample = int((endT-startT)*sample_rate)
      if N_sample>100: printYellow('Sample size too big (>100) in log ramp. Running speed might be slowed down!')
      t_sample = 1./sample_rate
      v_sample = np.log(np.linspace(np.exp(v0), np.exp(v1), N_sample))
      for ii in range(N_sample):
        self.Set([(startT+ii*t_sample,v_sample[ii],startT+(ii+1)*t_sample,v_sample[ii])])
    except:
      raise SetError('Invalid SetLogRamp() for '+str(self.name)+'.')
    
  def SetModulation(self, timeInterval, stamp):
    '''TODO: rewrite error message'''
    if stamp != None:
      stamp_length = stamp[-1][2] - stamp[0][0]
      interval_length = timeInterval[1] - timeInterval[0]
      mod_num = math.floor(interval_length / stamp_length)
      
      pair = Interval((timeInterval[0], stamp[0][1], timeInterval[1], stamp[-1][3], stamp, stamp_length, mod_num))
      self._UserValues.append(pair)
      t1 = self._transfunc_t(pair[0])
      t2 = self._transfunc_t(pair[2])

      transtamp = []
      for substamp in stamp:
        transtamp.append((self._transfunc_t(substamp[0]), self._transfunc_v(substamp[1]), self._transfunc_t(substamp[2]), self._transfunc_v(substamp[3])))

      newpair = Interval((t1, transtamp[0][1], t2, transtamp[-1][3], transtamp, stamp_length, mod_num))
      self._TransValues.append(newpair)
      
      self.seq.TIME_STOP = max(self.seq.TIME_STOP, self._UserValues[-1][2])
    else:
      raise SetError("Stamp is not set!")

# THIS IS A DANGEROUs FUNCTION TO USE: it literally returns whatever is at the end of the list at the point
# in the sequence file when you call it. It does NOT return the value of the channel prior to the time of your new command.
# In other words, UNLESS THE SEQUENCE FILE IS IN ORDER, THESE ARE LIKELY TO DO STRANGE/BAD THINGS
  def GetLastValue(self):
    self.SortChan()
    return self._UserValues[-1][3]
  def GetLastTime(self):
    self.SortChan()
    return self._UserValues[-1][2]

  def GetFirstValue(self):
    self.SortChan()
    if len(self._UserValues) != 0:
      return self._UserValues[0][1]
    else:
      return self.ssv

  def SortChan(self):
    # Sort by interval start time 
    self._UserValues  = sorted(self._UserValues,  key=lambda x:(x[0],x[2]))
    self._TransValues = sorted(self._TransValues, key=lambda x:(x[0],x[2]))

  def GetPreviousValue(self):
    return self.previous_value
    
  def SetSteadyStateValue(self, value):
    if (abs(value) > abs(self.max_v)):
      raise SetError("Steady state value for "+str(self.name)+" (" + str(value) + ") exceeds the maximum value (" + str(self.max_v) + ") configured for this channel.")
    else:
      self.ssv = value
      self._transformed_SSV = self._transfunc_v(value)
      # Sort by interval start time 
      self.SortChan()
      # Check time confliction
      for x in range(0, len(self._UserValues)-1):
        a1 = self._UserValues[x].start_t()
        a2 = self._UserValues[x].end_t()
        b1 = self._UserValues[x+1].start_t()
        b2 = self._UserValues[x+1].end_t()
        if (b1<a2) and (a1<b2):
          raise SetError("Invalid Set() time for "+str(self.name)+". Conflicting intervals: ("+str(a1)+", "+str(a2)+") and ("+str(b1)+", "+str(b2)+").")

  def SetPreviousValue(self, value):
    self.previous_value = value
 
  def GetSteadyStateValue(self):
    return self.ssv
    
  def GetHardwareValues(self):
    return self._TransValues
  
  def GetHardwareSSV(self): 
    return self._transformed_SSV
  
  def SetValueTransformation(self, trans):
    self._transfunc_v = trans
  
  def SetTimeTransformation(self, trans):
    self._transfunc_t = trans
    
#======================================================================#
# Print a channel (with 3 different levels of verbosity)               #
#======================================================================#
  def Print(self, verbose=0):
    if verbose == 0:
      print("Channel %d: '%s'" % (self.chanid, self.name))
    elif verbose > 0:
      print("Channel %d: '%s'" % (self.chanid, self.name))
    if verbose == 1:
      print(" LERP intervals: ", self._UserValues)
    if verbose == 2:
      lastT = 0
      lastV = 0
      for pair in self._UserValues:
        if lastT < pair.start_t():
          printComment(" - DWELL at "+str(lastV)+"V until " + str(pair.start_t()) +"us")
        if pair.start_V() < pair.end_V():
          printGreen(" - RAMP up from "+str(pair.start_V())+"V to "+str(pair.end_V())+"V between " + str(pair.start_t()) + "us and " + str(pair.end_t()) +"us")
        elif pair.start_V() > pair.end_V():
          printYellow(" - RAMP down from "+str(pair.start_V())+"V to "+str(pair.end_V())+"V between " + str(pair.start_t()) + "us and " + str(pair.end_t()) +"us")
        else:
          printComment(" - REMAIN at "+str(lastV)+"V from "+str(pair.start_t())+"us until "+str(pair.end_t())+"us")
        lastT = pair.end_t()
        lastV = pair.end_V()
 
#======================================================================#
# Given a time t, return the channel's supposed value at that time     #
#======================================================================#
  def TestPoint(self, t, verbose=0):
    lastT = 0
    lastV = 0
    for pair in self._UserValues:
      if t < pair[0]:
        if verbose:
          printComment("Value for '"+self.name+"' at t="+str(t)+" is "+str(lastV)+".")
        return 0
      elif t < pair[2]:
        theval = pair[1] + (t-pair[0]) * (pair[3]-pair[1]) / (pair[2]-pair[0])
        if verbose:
          printComment("Value for '"+self.name+"' at t="+str(t)+" is "+str(theval)+".")
        return theval
      elif t == pair[2]:
        theval = pair[3]
        if verbose:
          printComment("Value for '"+self.name+"' at t="+str(t)+" is "+str(theval)+".")
        return theval
      lastV = pair[3]
    if verbose:
      printComment("Value for '"+self.name+"' at t="+str(t)+" is "+str(2))
    return 2
    
#======================================================================#
# Given a time t, return the channel's supposed value at that time     #
#======================================================================#
  def VerifyIntervals(self, verbose=1):
    failcount = 0
    lastT = 0
    for pair in self._UserValues:
      if pair[0] > lastT:
        if failcount == 0:
          if verbose:
            printError("Channel '" + str(self.name) + "' is not explicitly defined at t=" + str(lastT) + ".")
        failcount += 1
      lastT = pair[2]
    if failcount == 0:
      if verbose:
        printComment("Channel '" + str(self.name) + "' is defined through t=" + str(lastT) + ".")
    return failcount
    
#======================================================================#
# Plot the sequence of an individual channel                           #
#======================================================================#
  def Plot(self):
    self.seq.Plot(single_chan=self)
    
########################################################################
#======================================================================#
#========================= Interval Class =============================#
#======================================================================#
########################################################################
class Interval(tuple):
  def start_t(self):
    return self[0]
  def start_V(self):
    return self[1]
  def end_t(self):
    return self[2]
  def end_V(self):
    return self[3]
