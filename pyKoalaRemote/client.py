# -*- coding: utf-8 -*-
"""
This class enables easy access to Koala TCP/IP remote interface
Prompt dialog for connection and login
Get functions return numpy Array
2023-03-30 Modified by TCO (all functions using Remote Manual orders)
"""
#from pythonnet import get_runtime_info
#a = get_runtime_info()

#import python package
from pyKoalaRemote import remote_utils as ru
import numpy as np
import sys
import clr
import time

#Add required dotNet reference
clr.AddReference("System")
import System
from System import Array

#Class pyRemote to manage dotNet dll in Python
class pyKoalaRemoteClient:
    
    def __init__(self, koala_8 = True):
        self.koala_8 = koala_8
        #create an instance to Koala remote Client. Version 8 and above has a new dll location
        if koala_8 :
            #Add Koala remote librairies to Path
            sys.path.append(r'C:\Program Files\LynceeTec\Koala\Remote\Remote Libraries\x64') #load x86 for 32 bits applications
            #Import KoalaRemoteClient
            clr.AddReference("LynceeTec.KoalaRemote.Client")
            from LynceeTec.KoalaRemote.Client import KoalaRemoteClient
            #Define KoalaRemoteClient host
            self.host=KoalaRemoteClient()
        
        else :
            #Add Koala remote librairies to Path
            sys.path.append(r'C:\Program Files\Koala\Remote Libraries\x64') #load x86 for 32 bits applications
            #Import KoalaRemoteClient
            clr.AddReference("TCPCLient")
            import KoalaClient
            #Define KoalaRemoteClient host
            self.host=KoalaClient.KoalaTCPClient()
        
        #init to None parameters:
        self.roiWidth = None
        self.roiHeight = None
        self.height = None
        self.width = None
        self.roiStride = None
        self.username = None
        
        #check if host is properly initialized
        try :
            self.host
        except :
            print("class not initialized")
            return
    
    def ConnectAndLoginDialog(self) :
        #Ask for IP adress
        IP = ru.get_input('Enter host IP adress','localhost')
        #Connect to Koala
        if self.Connect(IP):
            print('connected to Koala as',self.username,'on ',IP)
        else:
            print('connection to Koala failed on :',IP)
            print('Check if Koala is started or if the production window is open or if the previous session was closed')
            return False
        
        #ask for username password
        password = ru.get_input('Enter password for '+self.username+' account', self.username)
        #Login with username password
        if self.Login(password) :
            print('Log as ',self.username,)
        else :
            print('Login failed for',self.username,)
            return False
        return True
    
    def Connect(self,hostName,quiet=True):
        self.username = ''
        ret,self.username = self.host.Connect(hostName,self.username,quiet)
        return ret  
            
    def Login(self,password):
        return self.host.Login(password)
    
    def Logout(self) :
        try : self.host.Logout()
        except : 
            print("Logout failed")
            return
        print("Logout succesfull")        
        
    def __del__(self):
        self.Logout()
    
    #Open a configuration using config id
    def OpenConfigDialog(self) :
        #get config Id
        config = ru.get_input('Enter configuration number', default='137')
        #open config
        return self.OpenConfig(config)
   
    def OpenConfig(self, configNumber) :
        try : self.host.OpenConfig(configNumber)
        except: 
            print("configuration",configNumber,'do not exists')
            return
        print("Configuration",configNumber,"open")
        #wait for older koala version
        if not self.koala_8 :
            import time
            time.sleep(2) # 2 seconds to wait for OPL to move if DHM was re-init
   
    def updateROI(self) :
        self.roiWidth = self.host.GetPhaseWidth();
        self.roiHeight = self.host.GetPhaseHeight();
        if self.roiWidth % 4 == 0:
            self.roiStride = self.roiWidth
        else : 
            self.roiStride = (int(self.roiWidth / 4) * 4) + 4;
        return int(self.roiStride * self.roiHeight)
    
    def GetAxesPosMu(self) :
        #Define a dotNet (C#) Double Array
        buffer = Array.CreateInstance(System.Double,4)
        self.host.GetAxesPosMu(buffer)
        #copy and return buffer
        return ru.dn2np(buffer)
    
    def GetHoloImage(self) :
        #Define a dotNet (C#) Byte Array
        self.width = self.host.GetHoloWidth();
        self.height = self.host.GetHoloHeight();
        buffer = Array.CreateInstance(System.Byte,self.height*self.width)
        #Get holo from Koala
        self.host.GetHoloImage(buffer)
        #copy, reshape and return buffer
        return np.reshape(ru.dn2np(buffer),(self.height,self.width))
    
    def GetIntensity32fImage(self) :
        self.updateROI()
        #Define a dotNet (C#) Single Array
        buffer = Array.CreateInstance(System.Single,self.roiHeight*self.roiWidth)
        self.host.GetIntensity32fImage(buffer)
        #copy, reshape and return buffer
        return np.reshape(ru.dn2np(buffer),(self.roiHeight,self.roiWidth))
    
    def GetIntensityImage(self) :
        #Define a dotNet (C#) Byte Array
        buffer = Array.CreateInstance(System.Byte,self.updateROI())
        self.host.GetIntensityImage(buffer)
        #copy, reshape and return buffer
        return np.reshape(ru.dn2np(buffer),(self.roiHeight,self.roiStride))[:,0:self.roiWidth]

    def GetIntensityProfile(self):
        # Define a dotNet (C#) Double Array
        buffer = Array.CreateInstance(System.Double, self.GetIntensityProfileLength())
        self.host.GetIntensityProfile(buffer)
        # copy and return buffer
        return ru.dn2np(buffer)

    def GetPhase32fImage(self) :
        self.updateROI()
        #Define a dotNet (C#) Single Array
        buffer = Array.CreateInstance(System.Single,self.roiHeight*self.roiWidth)
        self.host.GetPhase32fImage(buffer)
        #copy, reshape and return buffer
        return np.reshape(ru.dn2np(buffer),(self.roiHeight,self.roiWidth))
    
    def GetPhaseImage(self) :
        #Define a dotNet (C#) Byte Array
        buffer = Array.CreateInstance(System.Byte,self.updateROI())
        self.host.GetPhaseImage(buffer)
        #copy, reshape and return buffer
        return np.reshape(ru.dn2np(buffer),(self.roiHeight,self.roiStride))[:,0:self.roiWidth]
    
    def GetPhaseProfile(self) :
        #Define a dotNet (C#) Double Array
        buffer = Array.CreateInstance(System.Double,self.GetPhaseProfileLength())
        self.host.GetPhaseProfile(buffer)
        #copy and return buffer
        return ru.dn2np(buffer)
    
    def GetPhaseProfileAxis(self):
        return np.arange(self.GetPhaseProfileLength()) * self.GetPxSizeUm()
    
    #wrapper for remote function, direct call
    def AccWDSearch(self,distUM,stepUM):
        return self.host.AccWDSearch(distUM,stepUM)
    
    def Acquisition2L(self):
        return self.host.Acquisition2L()
    
    def AddCorrSegment(self,top,left,length,orientation):
        """
        Add Correction segments
        top, left position and length
        orientation: 0 for horizontal and 1 for vertical
        """
        #left+length cannot be larger than width
        #top+length cannot be larger than height
        if self.roiWidth is None or self.roiWidth is None:
            self.updateROI()
        if orientation == 0:
            if left+length > self.roiWidth:
                length = self.roiWidth-left
        if orientation == 1:
            if top+length > self.roiHeight:
                length = self.roiHeight-top 
        return self.host.AddCorrSegment(top,left,length,orientation)
    
    def AddCorrZone(self, top, left, width, height):
        if self.roiWidth is None or self.roiWidth is None:
            self.updateROI()

        if left+width > self.roiWidth:
            width = self.roiWidth-left
        if top+height > self.roiHeight:
            height= self.roiHeight-top 
        return self.host.AddCorrZone(top, left, width, height)


    def AddPhaseOffsetAdjustmentZone(self, top, left, width, height):
        return self.host.AddPhaseOffsetAdjustmentZone(top, left, width, height)

    def AlgoResetPhaseMask(self):
        return self.host.AlgoResetPhaseMask()

    def AxisInstalled(self,axisId):
        '''
        0: X axis
        1: Y axis
        2: Z axis
        3: Theta axis (rotation, system-dependent)
        4: Phi axis (rotation, system-dependent)
        5: Psi axis (rotation, system-dependent)
        '''
        return self.host.AxisInstalled(axisId)



    def ComputePhaseCorrection(self, fitMethod, degree):
        """
        fitMethod = 0 only tilt = polynomial degree=1
        fitMethod = 1 : polynomial defined by degree
        fitMethod = 4: 2D polynomial defined by degree
        """
        if fitMethod == 0:
            degree = 1 #cannot use other order for fitMethod = 1
        return self.host.ComputePhaseCorrection(fitMethod, degree)
    
    def CloseIntensityWin(self):
        return self.host.CloseIntensityWin()


    def ClosePhaseWin(self):
        return self.host.ClosePhaseWin()

    def CloseReconstructionSettingsWin(self):
        return self.host.CloseReconstructionSettingsWin()

    
    def DigitizerAcquiring(self):
        return self.host.DigitizerAcquiring()

    def ExtractIntensityProfile(self, startX, startY, endX, endY):
        return self.host.ExtractIntensityProfile(startX, startY, endX, endY)

    def ExtractPhaseProfile(self, startX, startY, endX, endY):
        return self.host.ExtractPhaseProfile(startX, startY, endX, endY)
    
    def FastWDSearch(self):
        return self.host.FastWDSearch()
    
    def GetCameraShutterUs(self):
        '''
        Return: Int32: The shutter value, in [us]
        '''
        return self.host.GetCameraShutterUs()

    def GetChosenOPLPosition(self, oplId):
        return self.host.GetChosenOPLPosition(oplId)

    def GetDHMSerial(self):
        return self.host.GetDHMSerial()
    
    def GetHoloContrast(self):
        return self.host.GetHoloContrast()
    
    def GetHoloHeight(self):
        return self.host.GetHoloHeight()
    
    def GetHoloWidth(self):
        return self.host.GetHoloWidth()

    def GetIntensityProfileLength(self):
        return self.host.GetIntensityProfileLength()

    def GetKoalaVersion(self):
        return self.host.GetKoalaVersion()

    def GetLambdaNm(self,srcId,useLogicalId=True):
        return self.host.GetLambdaNm(srcId,useLogicalId)

    def GetMeasurementInformation(self, path, filename):
        return self.host.GetMeasurementInformation(path, filename)

    def GetOPLPos(self):
        return self.host.GetOPLPos()
    
    def GetPhaseHeight(self):
        return self.host.GetPhaseHeight()
    
    def GetPhaseWidth(self):
        return self.host.GetPhaseWidth()
    
    def GetPhaseProfileLength(self):
        return self.host.GetPhaseProfileLength()
    
    def GetPxSizeUm(self):
        return self.host.GetPxSizeUm()
    
    def GetRecDistCM(self):
        return self.host.GetRecDistCM()
    
    def GetUnwrap2DState(self):
        return self.host.GetUnwrap2DState()

    def InitXYZStage(self,withProgressBar=False,moveToCenter=False):
        return self.host.InitXYZStage(withProgressBar,moveToCenter)
    
    def KoalaShutDown(self,confirm=False):
        return self.host.KoalaShutDown(confirm)
    
    def LoadHolo(self,path,numLambda):
        return self.host.LoadHolo(path,numLambda)
    
    def ModifyFilterSwitchStatus(self, status):
        """
        Modify filter switch status to enable (True) or disable (False)
        """
        return self.host.ModifyFilterSwitchStatus(status)

    def MoveAxes(self, absMove, mvX, mvY, mvZ, mvTh, distX, distY, distZ, distTh, accX, accY, accZ, accTh, waitEnd=True):
        return self.host.MoveAxes(absMove, mvX, mvY, mvZ, mvTh, distX, distY, distZ, distTh, accX, accY, accZ, accTh, waitEnd)
    
    def MoveAxesArr(self,axes,absMove,dist,acc,waitEnd=True):
        return self.host.MoveAxesArr(axes,absMove,dist,acc,waitEnd)
    
    def MoveAxis(self, axisId, absMove, distUm, accuracyUM, waitEnd=True):
        return self.host.MoveAxis(axisId, absMove, distUm, accuracyUM, waitEnd)

    def MoveChosenOPLToPosition(self, posQc, oplId):
        return self.host.MoveChosenOPLToPosition(posQc, oplId)

    def MoveOPL(self, position):
        return self.host.MoveOPL(position)
        
    def OnDistanceChange(self):
        return self.host.OnDistanceChange()

    def OpenFrmTopography(self):
        return self.host.OpenFrmTopography()
        
    def OpenHoloWin(self):
        return self.host.OpenHoloWin()
    
    def OpenIntensityWin(self, updateXYScale=True):
        return self.host.OpenIntensityWin(updateXYScale)
    
    def OpenPhaseWin(self,withoutColorbar=False,doReconstruction=True,updateXYScale=True):
        return self.host.OpenPhaseWin(withoutColorbar,doReconstruction,updateXYScale)
    

    def OpenReconstructionSettingsWin(self):
        return self.host.OpenReconstructionSettingsWin()
    
    def ResetCorrSegment(self):
        return self.host.ResetCorrSegment()

    def ResetCorrZone(self):
        return self.host.ResetCorrZone()

    def ResetGrab(self):
        return self.host.ResetGrab()

    def ResetPhaseOffsetAdjustmentZone(self):
        return self.host.ResetPhaseOffsetAdjustmentZone()

    def ResetUserDefinedMaskZone(self):
        return self.host.ResetUserDefinedMaskZone()

    def SaveImageFloatToFile(self, winId, fileName, useBinFormat=False):
        '''
        winId
        1: hologram
        2: amplitude image
        4: phase image
        8: Fourier image
        
        useBinFormat: default, False is text, True (".bin")
        '''
        return self.host.SaveImageFloatToFile(winId,fileName,useBinFormat)
    
    def SaveImageToFile(self, winId, fileName):
        return self.host.SaveImageToFile(winId, fileName)

    def SaveReconstructionSettings(self):
        return self.host.SaveReconstructionSettings()

    def SelectDisplayWL(self, winId):
        '''
        winId
        8192: phase lambda 1 image
        16384: phase lambda 2 image
        32768: phase long synthetic wavelength image
        65536: phase short synthetic wavelength image
        2048: amplitude (intensity) lambda 1 image
        4096: amplitude (intensity) lambda 2 image
        512: Fourier lambda 1 image
        1024: Fourier lambda 2 image
        '''
        return self.host.SelectDisplayWL(winId)
    
    def SelectTopoZone(self, top, left, width, height):
        return self.host.SelectTopoZone(top, left, width, height)

    def SetCameraShutterUs(self, shutterUs):
        return self.host.SetCameraShutterUs(shutterUs)
    

    def SetIntensityProfileState(self, state):
        return self.host.SetIntensityProfileState(state)


    def SetPhaseProfileState(self, state=False):
        return self.host.SetPhaseProfileState(state)
    
    def SetRecDistCM(self,distCM):
        return self.host.SetRecDistCM(distCM)

    def SetSourceState(self, srcId, state, useLogicalId=True):
        return self.host.SetSourceState(srcId, state, useLogicalId)
    
    def SetUnwrap2DMethod(self, method):
        '''
        method
        0: Discrete Cosine Transform (DCT)
        1: Path-following (also known as Quality Path)
        '''
        return self.host.SetUnwrap2DMethod(method)
    
    def SetUnwrap2DState(self,state=False):
        return self.host.SetUnwrap2DState(state)


    def SingleReconstruction(self):
        return self.host.SingleReconstruction()


##The mask remote function
        
    def OpenMaskSettingsWin(self):
        return self.host.OpenMaskSettingsWin()
        
    def CloseMaskSettingsWin(self):
        return self.host.CloseMaskSettingsWin()
    
    def SetAutomaticIntensityThresholdFilterToEnabledState(self):
        return self.host.SetAutomaticIntensityThresholdFilterToEnabledState()
        
    def SetIntensityThresholdFilterState(self, state):
        return self.host.SetIntensityThresholdFilterState(state)

    def SetIntensityThresholdFilterValueInPercent(self, valueInPercent): #example if 30%, enter 30 as value
        return self.host.SetIntensityThresholdFilterValueInPercent(valueInPercent)
    
    def SetAutomaticPhaseGradientThresholdFilterToEnabledState(self):
        return self.host.SetAutomaticPhaseGradientThresholdFilterToEnabledState()
    
    def SetPhaseGradientThresholdFilterState(self, state):
        return self.host.SetPhaseGradientThresholdFilterState(state)

    def SetPhaseGradientThresholdFilterValueInPercent(self, valueInPercent): #example if 30%, enter 30 as value
        return self.host.SetPhaseGradientThresholdFilterValueInPercent(valueInPercent)
    
    def AddEllipticalUserDefinedMaskZoneToPhase(self, centerX, centerY, radiusX, radiusY):
        return self.host.AddEllipticalUserDefinedMaskZoneToPhase(centerX, centerY, radiusX, radiusY)

    def AddRectangularUserDefinedMaskZoneToPhase(self, top, left, width, height):
        return self.host.AddRectangularUserDefinedMaskZoneToPhase(top, left, width, height)  
    
    def SetInteractionModeWhenAddingMaskZone(self, mode): #mode is chosen between [1,2,3,4]
        return self.host.SetInteractionModeWhenAddingMaskZone(mode)

    
    def SetUserDefinedMaskState(self, state):
        return self.host.SetUserDefinedMaskState(state)

    def SetWavelengthFilterMinimalCutOffValue(self, minimalCutoffValue):
        return self.host.SetWavelengthFilterMinimalCutOffValue(minimalCutoffValue)

    def SetWavelengthFilterMaximalCutOffValue(self, maximalCutoffValue):
        return self.host.SetWavelengthFilterMaximalCutOffValue(maximalCutoffValue)

    def SetWavelengthFilterState(self, state):
        return self.host.SetWavelengthFilterState(state)

    def SetWavelengthFilterType(self, type):
        """
        type = 1 for Long-pass type filter,
        type = 2 for Short-pass type filter,
        type = 3 for Band-pass type filter,
        type = 4 for Band-stop type filter
        """
        return self.host.SetWavelengthFilterType(type)

##The sequence remote functions
        
    ### Fast Holograms Record
        
    def CloseFastHologramsRecordWin(self):
        return self.host.CloseFastHologramsRecordWin()
    
    def OpenFastHologramsRecordWin(self):
        return self.host.OpenFastHologramsRecordWin()
    
    def SetFastHologramsSequenceRecordNumberOfHolograms(self, numberOfHolograms):
        return self.host.SetFastHologramsSequenceRecordNumberOfHolograms(numberOfHolograms)
    
    def SetFastHologramsSequenceRecordingModeBuffer(self, state):
        return self.host.SetFastHologramsSequenceRecordingModeBuffer(state)
    
    def SetFastHologramsSequenceRecordPath(self, path):
        return self.host.SetFastHologramsSequenceRecordPath(path)
        
    def StartFastHologramsSequenceRecord(self):
        return self.host.StartFastHologramsSequenceRecord()
    
    def StopFastHologramsSequenceRecord(self):
        return self.host.StopFastHologramsSequenceRecord()
    
    ### Reconstruction to disk

    def CloseReconstructionToDiskSequenceWin(self):
        return self.host.CloseReconstructionToDiskSequenceWin()

    def OpenReconstructionToDiskSequenceWin(self):
        return self.host.OpenReconstructionToDiskSequenceWin()
    
    def SetReconstructionToDiskDataType(self, recordPhaseAsBin, recordPhaseAsText, recordPhaseAsTiff, recordIntensityAsBin, recordIntensityAsText, recordIntensityAsTiff):
        return self.host.SetReconstructionToDiskDataType(recordPhaseAsBin, recordPhaseAsText, recordPhaseAsTiff, recordIntensityAsBin, recordIntensityAsText, recordIntensityAsTiff)

    def SetReconstructionToDiskSequencePath(self, path):
        return self.host.SetReconstructionToDiskSequencePath(path)
    
    def StartReconstructionToDisk(self):
        return self.host.StartReconstructionToDisk()



## The stroboscope remote functions
        
    def ApplyNewDutyCycleInLiveMode(self):
        return self.host.ApplyNewDutyCycleInLiveMode()
    
    def ApplyNewVoltageAndOffsetInLiveModeForChannelNumber(self, channelNumber):
        return self.host.ApplyNewVoltageAndOffsetInLiveModeForChannelNumber(channelNumber)
    
    def CloseStroboWin(self):
        return self.host.CloseStroboWin()
    
    def DecreaseStroboscopeAngleStep(self):
        return self.host.DecreaseStroboscopeAngleStep()
    
    def IncreaseStroboscopeAngleStep(self):
        return self.host.IncreaseStroboscopeAngleStep()
    
    def MaximizeStroboscopeNumberOfSamples(self):
        return self.host.MaximizeStroboscopeNumberOfSamples()
    
    def OpenStroboWin(self):
        return self.host.OpenStroboWin()
        
    def RecordStroboscopeFixedFrequency(self, numberOfPeriods):
        return self.host.RecordStroboscopeFixedFrequency(numberOfPeriods)
        
    def RecordStroboscopeFrequencyScan(self):
        return self.host.RecordStroboscopeFrequencyScan()
    
    def SetStroboscopeChannelParameters(self, channelEnabled, chosenWaveform, voltage_mV, offset_mV, phaseDelay_deg, offsetType, chanelID=1):
        """
        SetStroboscopeChanelParameters by the chanelId (defaut = 1)
        channelEnabled (Boolean): true to enable the channel, false to disable it.
        chosenWaveform (Int32): waveform chosen in the array [1,2,3,4]
        voltage_mV (Int32): Voltage value in [mV] in the range [0,10000]
        offset_mV (Int32): Offset value in [mV] in the range [-10000,10000]
        phaseDelay_deg (Int32): Phase in [degrees] in the range [0,360]
        offsetType (Int32): Offset type [0 for "Manual", 1 for "0", 2 for "V<0", 3 for "V>0"]
        """
        if chanelID == 1:
            return self.SetStroboscopeChannel1Parameters(channelEnabled, chosenWaveform, voltage_mV, offset_mV, phaseDelay_deg, offsetType)
        if chanelID == 2:
            return self.SetStroboscopeChannel2Parameters(channelEnabled, chosenWaveform, voltage_mV, offset_mV, phaseDelay_deg, offsetType)
        if chanelID == 3:
            return self.SetStroboscopeChannel3Parameters(channelEnabled, chosenWaveform, voltage_mV, offset_mV, phaseDelay_deg, offsetType)
        if chanelID == 4:
            return self.SetStroboscopeChannel4Parameters(channelEnabled, chosenWaveform, voltage_mV, offset_mV, phaseDelay_deg, offsetType)
    
    def SetStroboscopeChannel1Parameters(self, channelEnabled, chosenWaveform, voltage_mV, offset_mV, phaseDelay_deg, offsetType):
        '''
        channelEnabled (Boolean): true to enable the channel, false to disable it.
        chosenWaveform (Int32): waveform chosen in the array [1,2,3,4]
        voltage_mV (Int32): Voltage value in [mV] in the range [0,10000]
        offset_mV (Int32): Offset value in [mV] in the range [-10000,10000]
        phaseDelay_deg (Int32): Phase in [degrees] in the range [0,360]
        offsetType (Int32): Offset type [0 for "Manual", 1 for "0", 2 for "V<0", 3 for "V>0"]
        '''
        return self.host.SetStroboscopeChannel1Parameters(channelEnabled, chosenWaveform, voltage_mV, offset_mV, phaseDelay_deg, offsetType)

    def SetStroboscopeChannel2Parameters(self, channelEnabled, chosenWaveform, voltage_mV, offset_mV, phaseDelay_deg, offsetType):
        '''
        channelEnabled (Boolean): true to enable the channel, false to disable it.
        chosenWaveform (Int32): waveform chosen in the array [1,2,3,4]
        voltage_mV (Int32): Voltage value in [mV] in the range [0,10000]
        offset_mV (Int32): Offset value in [mV] in the range [-10000,10000]
        phaseDelay_deg (Int32): Phase in [degrees] in the range [0,360]
        offsetType (Int32): Offset type [0 for "Manual", 1 for "0", 2 for "V<0", 3 for "V>0"]
        '''
        return self.host.SetStroboscopeChannel2Parameters(channelEnabled, chosenWaveform, voltage_mV, offset_mV, phaseDelay_deg, offsetType)

    def SetStroboscopeChannel3Parameters(self, channelEnabled, chosenWaveform, voltage_mV, offset_mV, phaseDelay_deg, offsetType):
        '''
        channelEnabled (Boolean): true to enable the channel, false to disable it.
        chosenWaveform (Int32): waveform chosen in the array [1,2,3,4]
        voltage_mV (Int32): Voltage value in [mV] in the range [0,10000]
        offset_mV (Int32): Offset value in [mV] in the range [-10000,10000]
        phaseDelay_deg (Int32): Phase in [degrees] in the range [0,360]
        offsetType (Int32): Offset type [0 for "Manual", 1 for "0", 2 for "V<0", 3 for "V>0"]
        '''
        return self.host.SetStroboscopeChannel3Parameters(channelEnabled, chosenWaveform, voltage_mV, offset_mV, phaseDelay_deg, offsetType)
    
    def SetStroboscopeChannel4Parameters(self, channelEnabled, chosenWaveform, voltage_mV, offset_mV, phaseDelay_deg, offsetType):
        '''
        channelEnabled (Boolean): true to enable the channel, false to disable it.
        chosenWaveform (Int32): waveform chosen in the array [1,2,3,4]
        voltage_mV (Int32): Voltage value in [mV] in the range [0,10000]
        offset_mV (Int32): Offset value in [mV] in the range [-10000,10000]
        phaseDelay_deg (Int32): Phase in [degrees] in the range [0,360]
        offsetType (Int32): Offset type [0 for "Manual", 1 for "0", 2 for "V<0", 3 for "V>0"]
        '''
        return self.host.SetStroboscopeChannel4Parameters(channelEnabled, chosenWaveform, voltage_mV, offset_mV, phaseDelay_deg, offsetType)

    def SetStroboscopeFixedFrequency(self, frequency):
        return self.host.SetStroboscopeFixedFrequency(frequency)
    
    def SetStroboscopeFrequencyScanEnabled(self, status=False):
        return self.host.SetStroboscopeFrequencyScanEnabled(status)
    
    def SetStroboscopeFrequencyScanParameters(self, minimumFrequency_Hz, maximumFrequency_Hz, stepSize_Hz, numberOfPeriodsPerFrequency, isDecreasing=False):
        '''
        minimumFrequency_Hz (Double): minimal frequency in [Hz] for the frequency scan.
        maximumFrequency_Hz (Double): maximal frequency in [Hz] for the frequency scan.
        stepSize_Hz (Double): frequency difference in [Hz] between 2 iterations (e.g with a frequency start at 1[kHz], and a step size of 200[Hz], the next frequency will be at 1.2[kHz], if we are increasing the frequency.
        numberOfPeriodsPerFrequency (Int32): number of periods applied on each frequency iteration.
        isDecreasing (Boolean): 
            true to decrease frequencies during the frequency scan (the scan will start at maximumFrequency_Hz and end at minimumFrequency_Hz while taking into account the stepSize_Hz), 
            false to increase frequencies during the frequency scan (the scan will start at minimumFrequency_Hz and end at maximumFrequency_Hz while taking into account the stepSize_Hz).
        '''
        return self.host.SetStroboscopeFrequencyScanParameters(minimumFrequency_Hz, maximumFrequency_Hz, stepSize_Hz, numberOfPeriodsPerFrequency, isDecreasing=False)
    
    def SetStroboscopeLaserPulseDutyCycle(self, dutyCycle):
        return self.host.SetStroboscopeLaserPulseDutyCycle(dutyCycle)
    
    def SetStroboscopeNumberOfSamplesPerPeriod(self, samplesPerPeriod):
        return self.host.SetStroboscopeNumberOfSamplesPerPeriod(samplesPerPeriod)
    
    def SetStroboscopeRecordAtStartStatus(self, status=False):
        return self.host.SetStroboscopeRecordAtStartStatus(status)
    
    def SetStroboscopeRecordDataType(self, recordPhaseAsBin, recordPhaseAsTiff, recordIntensityAsBin, recordIntensityAsTiff):
        return self.host.SetStroboscopeRecordDataType(recordPhaseAsBin, recordPhaseAsTiff, recordIntensityAsBin, recordIntensityAsTiff)
    
    def SetStroboscopeRecordPath(self, path):
        return self.host.SetStroboscopeRecordPath(path)
    
    def StartStroboscopeFixedFrequency(self, cycleMode, numberOfPeriods):
        return self.host.StartStroboscopeFixedFrequency(cycleMode, numberOfPeriods)
    
    def StopStroboscope(self):
        return self.host.StopStroboscope()


#test the class when running this file directly
if __name__ == '__main__' :
    remote = pyKoalaRemoteClient()
    remote.ConnectAndLoginDialog()
    remote.OpenConfigDialog()
    remote.LoadHolo(r'C:\tmp\holo.tif',1)
    remote.OpenHoloWin()
    remote.OpenIntensityWin()
    remote.OpenPhaseWin()
    remote.OpenReconstructionSettingsWin()
    time.sleep(0.1) #wait window is open
    remote.AddCorrSegment(50,10,800,0)

    remote.AddCorrSegment(50,50,200,1)
    remote.ComputePhaseCorrection(0,1)
    remote.ComputePhaseCorrection(1,2)
    remote.ResetCorrSegment()
    remote.SetUnwrap2DState(True)
    remote.AddCorrZone(100,100,900,900)
    remote.ComputePhaseCorrection(4,2)
    remote.ResetCorrZone()
    remote.SetUnwrap2DState(False)
    remote.ResetPhaseOffsetAdjustmentZone()
    remote.AddPhaseOffsetAdjustmentZone(50,50, 200,200)
    remote.Logout()