from enum import Enum
import time
import argparse
import serial
import logging
import coloredlogs

# Initialize Logger
Logger = logging.getLogger('tiva-imu')
coloredlogs.install(logger=Logger)

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
    self.__ser = serial.Serial(port=PortName, baudrate=115200, rtscts=True, timeout=self.__SerialTimeout)

  def __runCommand(self, cmd, timeout=__SerialTimeout):
    ret = Error_e.RET_OK
    # Hold private variables
    defaultTimeout = self.__SerialTimeout
    serialHandler = self.__ser

    # Flush serial receive buffer
    serialHandler.reset_input_buffer()
    # Update timeout + 30% (considering serial buffering)
    serialHandler.timeout = defaultTimeout if (timeout == defaultTimeout) else (timeout * 1.3)

    Logger.debug('Running Command [{}] with timeout [{}]'.format(cmd.replace('\n',''),timeout))
    serialHandler.write(convertString2Bytes(cmd))

    # Read until the end of command
    data = serialHandler.read_until(self.__EndCmd.encode('utf-8')).decode('utf-8')

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

    # Reset the previous timeout
    serialHandler.timeout = defaultTimeout
    return ret, data

  def __parseImuData(self, data):
    ret = {}
    for line in data.split('\n'):
      if 'RawData' in line:
        ImuName = line.split(';')[1]
        TimePoint = float(line.split(';')[2])
        AccelX = float(line.split(';')[3])
        AccelY = float(line.split(';')[4])
        AccelZ = float(line.split(';')[5])
        GyroX = float(line.split(';')[6])
        GyroY = float(line.split(';')[7])
        GyroZ = float(line.split(';')[8])
        accel = Accel(AccelX, AccelY, AccelZ)
        gyro = Gyro(GyroX, GyroY, GyroZ)

        imu = ret[ImuName] if ImuName in ret else ImuData()
        imu.addAccel(TimePoint, accel)
        imu.addGyro(TimePoint, gyro)
        ret[ImuName] = imu
      if 'PureAngle' in line:
        ImuName = line.split(';')[1]
        TimePoint = float(line.split(';')[2])
        AngleX = float(line.split(';')[3])
        AngleY = float(line.split(';')[4])
        AngleZ = float(line.split(';')[5])
        angle = AngleEuler(AngleX, AngleY, AngleZ)

        imu = ret[ImuName] if ImuName in ret else ImuData()
        imu.addAngle(TimePoint, angle)
        imu.addGyro(TimePoint, gyro)
        ret[ImuName] = imu
    return ret

  def runHelp(self):
    ret, _ = self.__runCommand(cmd='help\n')
    return ret

  def runReset(self):
    ret, _ = self.__runCommand(cmd='reset\n', timeout=10)
    return ret

  def runImuSample(self, t):
    ret, data = self.__runCommand(cmd='imu-run {}\n'.format(t), timeout=t)
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
  def __init__(self, accel={}, gyro={}, angle={}):
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


def main():
  # Parse command line arguments
  args = parser.parse_args()
  # Store variables
  serialPort = args.Serial
  debug = args.Debug

  if debug:
    coloredlogs.install(level='DEBUG',logger=Logger)

  tiva = Tiva(PortName=serialPort)
  print('Reset Target')
  tiva.runReset()
  tiva.runRgbFreq(500)
  print('Run Help')
  tiva.runHelp()
  print("Running imu-run")
  _, data = tiva.runImuSample(4)
  tiva.runRgbFreq(0)

  for ImuName,imudata in data.items():
    print('Accelerometer Data:')
    for t,accel in imudata.ArrAccel.items():
      print('  {} -> Accel [{} ms] [{}]'.format(ImuName, t, accel))
    print('Gyroscope Data:')
    for t,gyro in imudata.ArrAccel.items():
      print('  {} -> Gyro [{} ms] [{}]'.format(ImuName, t, gyro))
    print('Euler Angle Data:')
    for t,angle in imudata.ArrAngle.items():
      print('  {} -> Angle [{} ms] [{}]'.format(ImuName, t, angle))


if __name__ == "__main__":
  main()
