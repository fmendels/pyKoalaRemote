from pyKoalaRemote import client
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import time

#create instance to class pyRemote
remote = client.pyKoalaRemoteClient()
#Connect and Login
remote.ConnectAndLoginDialog()

#Open a 1 wavelength configuration.
remote.OpenConfig(147)

#Open main display windows
remote.OpenPhaseWin()
remote.OpenIntensityWin()
remote.OpenHoloWin()
remote.OpenStroboWin()

remote.SetStroboscopeFrequencyScanEnabled(False);

#Example on how to set record parameters in stroboscopic window
#Here, we choose to record the phase and intensity data in bin format
#SetStroboscopeRecordDataType(recordPhaseAsBin, recordPhaseAsTiff, recordIntensityAsBin, recordIntensityAsTiff)
remote.SetStroboscopeRecordDataType(True, False, True, False)

#Sets the stroboscopic tool to record immediately on start.
#remote.SetStroboscopeRecordAtStartStatus(True);

#Gives the root path to record the data
remote.SetStroboscopeRecordPath(r'C:\temp')

#Modify the channel 1 parameters
#SetStroboscopeChannel1Parameters(channelEnabled, chosenWaveform, voltage_in_mV, offset_in_mV, phaseDelay_deg, offsetType)
#Example: Activate channel 1, select wavelength 1, put 8V as voltage, -2V as offset, phase delay 0 degrees, and manual offset
remote.SetStroboscopeChannel1Parameters(True, 1, 1000, 0, 0, 0)

#Example: if you have a second channel 2 available and do not use it for this experiment.
remote.SetStroboscopeChannel2Parameters(False, 1, 0, 0, 0, 0)

#Set the laser pulse duty cycle in percent
remote.SetStroboscopeLaserPulseDutyCycle(10)

#For the frequency approach to work correctly, we need to set the fixed frequency to the target frequency to ensure the stroboscope is in the correct frequency range.
#Frequency = 9.7kHz
remote.SetStroboscopeFixedFrequency(9700)

#Increase the angle step (move the green bar from the frequency slider towards higher frequencies
#remote.IncreaseStroboscopeAngleStep()

#Decrease the angle step (move the green bar from the frequency slider towards lower frequencies
#remote.DecreaseStroboscopeAngleStep()

#Modify the number of samples per period to 6
remote.SetStroboscopeNumberOfSamplesPerPeriod(6);

#Frequency Scan
#Enable the frequency scan mode
remote.SetStroboscopeFrequencyScanEnabled(True)

#Set the frequency scan from 9.7kHz to 10kHz by increasing the frequency by steps of 1kHz.
#SetStroboscopeFrequencyScanParameters(self, minimumFrequency_Hz, maximumFrequency_Hz, stepSize_Hz, numberOfPeriodsPerFrequency, isDecreasing)
remote.SetStroboscopeFrequencyScanParameters(9700, 10000, 1000, 2, False)

#Here, we choose to record the phase and intensity data in bin format
#SetStroboscopeRecordDataType(recordPhaseAsBin, recordPhaseAsTiff, recordIntensityAsBin, recordIntensityAsTiff)
remote.SetStroboscopeRecordDataType(True, False, True, False)

#Sets the stroboscopic tool to record immediately on start.
#remote.SetStroboscopeRecordAtStartStatus(True);

#Gives the root path to record the data
remote.SetStroboscopeRecordPath(r'C:\temp')

#Start the recording of the frequency scan
remote.RecordStroboscopeFrequencyScan()

#Wait 10s
time.sleep(10)

#Stop the stroboscope : Not needed
#remote.StopStroboscope()

#Disable the frequency scan mode
remote.SetStroboscopeFrequencyScanEnabled(False)

#Close the stroboscopic window
remote.CloseStroboWin();

remote.Logout()
