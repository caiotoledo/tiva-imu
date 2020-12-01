from enum import Enum
import datetime
import re
import time
import argparse
import serial
import logging
import coloredlogs
import multiprocessing
from concurrent.futures import ThreadPoolExecutor
import queue
import matplotlib.pyplot as plt

# Initialize Logger
Logger = logging.getLogger('tiva-imu')

# Version variable:
__version__ = "v0.0.0"

# Arguments parser:
parser = argparse.ArgumentParser(description='Script to interact with tiva-imu')
parser.add_argument("--Serial",
                    "-s",
                    dest="Serial",
                    help="Serial Port connected to tiva-imu",
                    metavar="PORT",
                    required=True)
parser.add_argument('--Debug',
                    '-d',
                    dest="Debug",
                    help="Enable Debug Log",
                    action='store_true',
                    default=False)
parser.add_argument('--version',
                    '-v',
                    action='version',
                    version=__version__)


def convertString2Bytes(str):
  return bytes(str,'utf-8')


class Error_e(Enum):
  RET_OK = 0
  RET_ERROR = 1


class Tiva:
  __SerialTimeout = 1
  __EndCmd = '>>'
  def __init__(self, PortName):
    cpucount = multiprocessing.cpu_count()
    Logger.debug('Max Workers: {}'.format(cpucount))
    self.__executor = ThreadPoolExecutor(max_workers=cpucount)
    self.__ser = serial.Serial(port=PortName, baudrate=115200, rtscts=True, timeout=0)

  def __runCommand(self, cmd, CallbackLine=None, timeout=__SerialTimeout):
    ret = Error_e.RET_OK
    # Hold private variables
    defaultTimeout = self.__SerialTimeout
    serialHandler = self.__ser

    # Flush serial receive buffer
    serialHandler.reset_input_buffer()

    # Send command via serial
    Logger.debug('Running Command [{}] with timeout [{}]'.format(cmd.replace('\n',''),timeout))
    serialHandler.write(cmd.encode())

    # Update timeout + 30% (considering serial buffering)
    cmdTimeout = defaultTimeout if (timeout == defaultTimeout) else (timeout * 1.3)
    dateTimeout = datetime.timedelta(seconds=cmdTimeout)

    # Start Timer
    begin = datetime.datetime.now()
    data = '' # Initialize empty string
    line = ''
    while (datetime.datetime.now() - begin) < dateTimeout:
      char = serialHandler.read_until().decode()
      line = line + char
      if ('\n' in line) or (self.__EndCmd in line):
        data = data + line
        # Notify new line
        if (CallbackLine is not None) and (len(line) > 0): _ = self.__executor.submit(CallbackLine, line)
        line = '' # reset string line
        # Read until the end of command
        if self.__EndCmd in data:
          break

    if self.__EndCmd in data:
      ret = Error_e.RET_OK
      Logger.debug('Return command [\n{}]'.format(data.replace(self.__EndCmd,'')))
    else:
      ret = Error_e.RET_ERROR
      Logger.warning('Command [{}] not finished - Return command [\n{}]'.format(cmd.replace('\n',''), ret))
      # Clean-up serial buffer to avoid issues in next command
      maxTimeout = 1
      waitRead = defaultTimeout*10
      time.sleep(waitRead if waitRead <= maxTimeout else maxTimeout)
      serialHandler.reset_input_buffer()

    return ret, data

  def __parseImuDataLine(self, line):
    ImuName, TimePoint, AccelVal, GyroVal, AngleVal = None, None, None, None, None

    try:
      if re.search('^RawData',line) is not None:
        ImuName = line.split(';')[1]
        TimePoint = float(line.split(';')[2])
        AccelX = float(line.split(';')[3])
        AccelY = float(line.split(';')[4])
        AccelZ = float(line.split(';')[5])
        GyroX = float(line.split(';')[6])
        GyroY = float(line.split(';')[7])
        GyroZ = float(line.split(';')[8])
        AccelVal = Accel(AccelX, AccelY, AccelZ)
        GyroVal = Gyro(GyroX, GyroY, GyroZ)
      if re.search('^PureAngle',line) is not None:
        ImuName = line.split(';')[1]
        TimePoint = float(line.split(';')[2])
        AngleX = float(line.split(';')[3])
        AngleY = float(line.split(';')[4])
        AngleZ = float(line.split(';')[5])
        AngleVal = AngleEuler(AngleX, AngleY, AngleZ)
    except:
      Logger.error('Invalid line [{}]'.format(line))
      pass

    return ImuName, TimePoint, AccelVal, GyroVal, AngleVal

  def __parseImuData(self, data):
    ret = {}
    for line in data.split('\n'):
      # Parse each line
      ImuName, timepoint, accel, gyro, angle = self.__parseImuDataLine(line)
      # Store data
      if ImuName is not None and timepoint is not None:
        imu = ret[ImuName] if ImuName in ret else ImuData(accel={}, gyro={}, angle={})
        if accel is not None: imu.addAccel(timepoint, accel)
        if gyro is not None: imu.addGyro(timepoint, gyro)
        if angle is not None: imu.addAngle(timepoint, angle)
        ret[ImuName] = imu
    return ret

  def runHelp(self):
    ret, _ = self.__runCommand(cmd='help\n')
    return ret

  def runReset(self):
    ret, _ = self.__runCommand(cmd='reset\n', timeout=10)
    return ret

  def runImuSampleRate(self, t):
    ret, _ = self.__runCommand(cmd='imu-sample {}\n'.format(t))
    return ret

  def runImuSample(self, t, Callback=None):
    # Provide the data in realtime
    def cb(line):
      if Callback is not None:
        name, timepoint, accel, gyro, angle = self.__parseImuDataLine(line)
        if name is not None: Callback(name, timepoint, accel, gyro, angle)
    # Execute command
    ret, data = self.__runCommand(cmd='imu-run {}\n'.format(t), timeout=t, CallbackLine=cb)
    imudata = self.__parseImuData(data) if ret == Error_e.RET_OK else {}
    return ret, imudata

  def runRgbFreq(self, freq):
    ret, data = self.__runCommand(cmd='rgb-freq {}\n'.format(freq))
    ret = Error_e.RET_ERROR if 'Invalid value' in data else ret
    return ret


class Generic3DPoint:
  def __init__(self, x, y, z):
    self.X = x
    self.Y = y
    self.Z = z

  def __str__(self):
    return 'X = {}; Y = {}; Z = {};'.format(self.X,self.Y,self.Z)


class Accel(Generic3DPoint):
  def __init__(self, x, y, z):
    super().__init__(x,y,z)


class Gyro(Generic3DPoint):
  def __init__(self, x, y, z):
    super().__init__(x,y,z)


class AngleEuler(Generic3DPoint):
  def __init__(self, x, y, z):
    super().__init__(x,y,z)


class ImuData:
  def __init__(self, accel, gyro, angle):
    self.ArrAccel = accel
    self.ArrGyro = gyro
    self.ArrAngle = angle

  def addAccel(self, t, accel):
    self.ArrAccel[t] = accel

  def addGyro(self, t, gyro):
    self.ArrGyro[t] = gyro

  def addAngle(self, t, angle):
    self.ArrAngle[t] = angle

  def __add__(self, other):
    accel = {**self.ArrAccel, **other.ArrAccel}
    gyro = {**self.ArrGyro, **other.ArrGyro}
    angle = {**self.ArrAngle, **other.ArrAngle}
    return ImuData(accel, gyro, angle)


class ImuDataPlot():
  def __init__(self, imudata=ImuData(accel={}, gyro={}, angle={})):
    self.Data = imudata
    self.__fig, (self.__axAccel, self.__axGyro, self.__axAngle) = plt.subplots(3, 1)
    self.__line = {}

    self.__axAccel.set_title('Accelerometer')
    self.__axAccel.set_ylim(2500,-2500) # Based on ACCEL_CONFIG Register (limits +-2g)
    self.__axAccel.set(ylabel='Accel (mg)', xlabel='Time (ms)')

    self.__axGyro.set_title('Gyroscope')
    self.__axGyro.set_ylim(300,-300) # Based on GYRO_CONFIG Register (limits +-250°/s)
    self.__axGyro.set(ylabel='Gyro (°/s)', xlabel='Time (ms)')

    self.__axAngle.set_title('Euler Angle')
    self.__axAngle.set_ylim(400,-400)
    self.__axAngle.set(ylabel='Angle (°)', xlabel='Time (ms)')

  def setImuData(self, imudata):
    self.Data = imudata

  def appendImuData(self, imudata):
    self.Data = self.Data + imudata

  def __getArrDataPlot(self):
    def __parseData(Arr3DPointIn):
      dictOut = {'T':[], 'X':[], 'Y':[], 'Z':[]}
      for key,value in Arr3DPointIn.items():
        dictOut['T'].append(key)
        dictOut['X'].append(value.X)
        dictOut['Y'].append(value.Y)
        dictOut['Z'].append(value.Z)
      return dictOut

    imuAccel = __parseData(self.Data.ArrAccel)
    imuGyro =  __parseData(self.Data.ArrGyro)
    imuAngle = __parseData(self.Data.ArrAngle)

    return imuAccel, imuGyro, imuAngle

  def __plotGraph(self, ax, plotData):
    ax.plot(plotData['T'], plotData['X'], 'r.')
    ax.plot(plotData['T'], plotData['Y'], 'g.')
    ax.plot(plotData['T'], plotData['Z'], 'b.')
    ax.grid(b=True)

  def showGraph(self, title='', pause_time=0.001):
    self.__fig.suptitle(title)
    plotAccel, plotGyro, plotAngle = self.__getArrDataPlot()

    self.__plotGraph(self.__axAccel, plotAccel)
    self.__plotGraph(self.__axGyro, plotGyro)
    self.__plotGraph(self.__axAngle, plotAngle)

    plt.draw()
    plt.pause(pause_time)


imuQueue = queue.Queue()
def CallbackImuData(name, timepoint, accel, gyro, angle):
  Logger.debug('[{}][{}] / {} / {} / {}'.format(name, timepoint, accel, gyro, angle))

  imu = ImuData(accel={}, gyro={}, angle={})
  if accel is not None: imu.addAccel(timepoint, accel)
  if gyro is not None: imu.addGyro(timepoint, gyro)
  if angle is not None: imu.addAngle(timepoint, angle)
  imuQueue.put((name, imu))


def main():
  start_time = time.time()

  # Parse command line arguments
  args = parser.parse_args()
  # Store variables
  serialPort = args.Serial
  debug = args.Debug

  LoggerLevel = 'DEBUG' if debug is True else 'INFO'
  coloredlogs.install(level=LoggerLevel,logger=Logger)

  executor = ThreadPoolExecutor(max_workers=multiprocessing.cpu_count())

  tiva = Tiva(PortName=serialPort)
  Logger.info('Reset Target')
  tiva.runReset()

  rgbfreq = 500
  Logger.info('Set RGB Frequency to {}'.format(rgbfreq))
  tiva.runRgbFreq(rgbfreq)

  Logger.info('Run Help')
  tiva.runHelp()

  samplerate = 500
  Logger.info('Set Sample Rate to {} ms'.format(samplerate))
  tiva.runImuSampleRate(samplerate)

  def disableRGB(_fut):
    Logger.info('Disable RGB')
    tiva.runRgbFreq(0)

  imuSampleTimeout = 4
  Logger.info('Running imu-run for {} seconds'.format(imuSampleTimeout))
  fut = executor.submit(tiva.runImuSample, t=imuSampleTimeout, Callback=CallbackImuData)
  fut.add_done_callback(disableRGB)
  myplot = {}
  while (fut.running() is True) or (imuQueue.empty() is not True):
    try:
      # Get data from queue
      name, item = imuQueue.get(timeout=samplerate/1000)
      # Instanciate new object or get from dict the plot
      plot = myplot[name] if name in myplot else ImuDataPlot()
      # Append data
      plot.appendImuData(item)
      titleGraph = 'IMU Data [{}]'.format(name)
      plot.showGraph(titleGraph, pause_time=samplerate/1000)
      # Feedback to dict
      myplot[name] = plot
    except queue.Empty:
      # No data available, keep waiting until the future is done
      pass
  _, data = fut.result()

  for ImuName,imudata in data.items():
    print('Accelerometer Data:')
    for t,accel in imudata.ArrAccel.items():
      print('  {} -> Accel [{} ms] [{}]'.format(ImuName, t, accel))
    print('Gyroscope Data:')
    for t,gyro in imudata.ArrGyro.items():
      print('  {} -> Gyro [{} ms] [{}]'.format(ImuName, t, gyro))
    print('Euler Angle Data:')
    for t,angle in imudata.ArrAngle.items():
      print('  {} -> Angle [{} ms] [{}]'.format(ImuName, t, angle))

  Logger.info("Execution time {:.2f} seconds".format(time.time() - start_time))


if __name__ == "__main__":
  main()
