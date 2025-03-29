import ctypes
import os

# Load the shared library
libdshot = ctypes.CDLL(os.path.join(os.path.dirname(__file__), 'libdshot.so'))

# Define the argument and return types for the C functions
libdshot.motorImplementationInitialize.argtypes = (ctypes.POINTER(ctypes.c_int), ctypes.c_int)
libdshot.motorImplementationInitialize.restype = None

libdshot.motorImplementationFinalize.argtypes = (ctypes.POINTER(ctypes.c_int), ctypes.c_int)
libdshot.motorImplementationFinalize.restype = None

libdshot.motorImplementationSendThrottles.argtypes = (ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.POINTER(ctypes.c_double))
libdshot.motorImplementationSendThrottles.restype = None

libdshot.dshotRepeatSendCommand.argtypes = (ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int)
libdshot.dshotRepeatSendCommand.restype = None

libdshot.motorImplementationSet3dModeAndSpinDirection.argtypes = (ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_int, ctypes.c_int)
libdshot.motorImplementationSet3dModeAndSpinDirection.restype = None

# Python wrapper functions
def motorImplementationInitialize(motorPins, motorMax):
    motorPinsArray = (ctypes.c_int * len(motorPins))(*motorPins)
    libdshot.motorImplementationInitialize(motorPinsArray, motorMax)

def motorImplementationFinalize(motorPins, motorMax):
    motorPinsArray = (ctypes.c_int * len(motorPins))(*motorPins)
    libdshot.motorImplementationFinalize(motorPinsArray, motorMax)

def motorImplementationSendThrottles(motorPins, motorMax, motorThrottle):
    motorPinsArray = (ctypes.c_int * len(motorPins))(*motorPins)
    motorThrottleArray = (ctypes.c_double * len(motorThrottle))(*motorThrottle)
    libdshot.motorImplementationSendThrottles(motorPinsArray, motorMax, motorThrottleArray)

def dshotRepeatSendCommand(motorPins, motorMax, cmd, telemetry, repeatCounter):
    motorPinsArray = (ctypes.c_int * len(motorPins))(*motorPins)
    libdshot.dshotRepeatSendCommand(motorPinsArray, motorMax, cmd, telemetry, repeatCounter)

def motorImplementationSet3dModeAndSpinDirection(motorPins, motorMax, mode3dFlag, reverseDirectionFlag):
    motorPinsArray = (ctypes.c_int * len(motorPins))(*motorPins)
    libdshot.motorImplementationSet3dModeAndSpinDirection(motorPinsArray, motorMax, mode3dFlag, reverseDirectionFlag)
