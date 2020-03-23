import serial
import time
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui

# Change the configuration file name
configFileName = 'xwr1642_profile_VitalSigns_20fps_Front.cfg'
CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2**15,dtype = 'uint8')
byteBufferLength = 0;


# ------------------------------------------------------------------

# Function to configure the serial ports and send the data from
# the configuration file to the radar
def serialConfig(configFileName):
    
    global CLIport
    global Dataport
    # Open the serial ports for the configuration and the data ports
    
    # Raspberry pi
    #CLIport = serial.Serial('/dev/ttyACM0', 115200)
    #Dataport = serial.Serial('/dev/ttyACM1', 921600)
    
    # Windows
    CLIport = serial.Serial('COM3', 115200)
    Dataport = serial.Serial('COM4', 921600)

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        CLIport.write((i+'\n').encode())
        print(i)
        time.sleep(0.01)
        
    return CLIport, Dataport

# ------------------------------------------------------------------

# Function to parse the data inside the configuration file
def parseConfigFile(configFileName):
    configParameters = {} # Initialize an empty dictionary to store the configuration parameters
    
    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        
        # Split the line
        splitWords = i.split(" ")
        
        # Hard code the number of antennas, change if other configuration is used
        numRxAnt = 4
        numTxAnt = 2
        
        # Get the information about the profile configuration
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1;
            
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2;

            digOutSampleRate = int(splitWords[11])
            
        # Get the information about the frame configuration    
        elif "frameCfg" in splitWords[0]:
            
            chirpStartIdx = int(splitWords[1])
            chirpEndIdx = int(splitWords[2])
            numLoops = int(splitWords[3])
            framePeriodicity = int(splitWords[5])

        # Get the information about the GUI configuration   
        elif "guiMonitor" in splitWords[0]:
            guiMonitor_flag1 =  int(splitWords[1])
            guiMonitor_flag2 = int(splitWords[2])  
            guiMonitor_rangeAzimuthHeatMap = int(splitWords[3])
            guiMonitor_rangeDopplerHeatMap = int(splitWords[4])

        # Get the information about the GUI configuration   
        elif "vitalSignsCfg" in splitWords[0]:
            rangeStartMeters =  float(splitWords[1])
            rangeEndMeters = float(splitWords[2])  
            winLenBreath = int(splitWords[3])
            winLenHeart = int(splitWords[4])
            
    # Combine the read data to obtain the configuration parameters           
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["chirpDuration_us"] = (1e3*numAdcSamples)/(digOutSampleRate)
    freqSlopeConst_temp = 48*freqSlopeConst* 2**26 * 1e3/((3.6*1e9)*900);  # To match the C-code 

    configParameters["chirpBandwidth_kHz"] = (freqSlopeConst_temp)*(configParameters["chirpDuration_us"])
    numTemp = (configParameters["chirpDuration_us"])*(digOutSampleRate)*(3e8)
    denTemp = 2*(configParameters["chirpBandwidth_kHz"])
    configParameters["rangeMaximum"] =  numTemp/(denTemp*1e9)
    configParameters["rangeBinSize_meter"] = configParameters["rangeMaximum"]/(configParameters["numRangeBins"])
    rangeStart_Index = int(rangeStartMeters/configParameters["rangeBinSize_meter"])
    rangeEnd_Index   = int(rangeEndMeters/configParameters["rangeBinSize_meter"])
    configParameters["lambdaCenter_mm"] = (3e8/(startFreq))/1e6
    configParameters["rangeStart_Index"] = rangeStart_Index
    configParameters["rangeEnd_Index"] = rangeEnd_Index
    configParameters["numRangeBinProcessed"] = rangeEnd_Index - rangeStart_Index + 1
    
    return configParameters
   
# ------------------------------------------------------------------

# Funtion to read and parse the incoming data
def readAndParseVitalSigns(Dataport, configParameters):
    global byteBuffer, byteBufferLength

    # Constants
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
    maxBufferSize = 2**15

    RANGE_PROFILE_TYPE = 2
    VITAL_SIGNS_OUTPUT_TYPE = 6

    LENGTH_HEADER_BYTES                  = 40   # Header + Magic Word
    LENGTH_TLV_MESSAGE_HEADER_BYTES      = 8
    LENGTH_DEBUG_DATA_OUT_BYTES          = 128   # VitalSignsDemo_OutputStats size
    MMWDEMO_OUTPUT_MSG_SEGMENT_LEN       = 32   # The data sent out through the UART has Extra Padding to make it a multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN
    LENGTH_OFFSET_BYTES                  = LENGTH_HEADER_BYTES  - len(magicWord) + LENGTH_TLV_MESSAGE_HEADER_BYTES
    LENGTH_OFFSET_NIBBLES                = 2*LENGTH_OFFSET_BYTES

    NUM_PTS_DISTANCE_TIME_PLOT        = 256 
    HEART_RATE_EST_MEDIAN_FLT_SIZE    = 200
    HEART_RATE_EST_FINAL_OUT_SIZE     = 200
    THRESH_HEART_CM                   = 0.25
    THRESH_BREATH_CM                  = 1.0
    BACK_THRESH_BPM                   = 4
    BACK_THRESH_CM                    = 0.20
    BACK_THRESH_4Hz_CM                = 0.15
    THRESH_BACK                       = 30
    THRESH_DIFF_EST                   = 20
    ALPHA_HEARTRATE_CM                = 0.2
    ALPHA_RCS                         = 0.2
    APPLY_KALMAN_FILTER               = 0.0

    totalPayloadSize_bytes = LENGTH_HEADER_BYTES;
    totalPayloadSize_bytes += LENGTH_TLV_MESSAGE_HEADER_BYTES + (4*configParameters["numRangeBinProcessed"]);
    totalPayloadSize_bytes += LENGTH_TLV_MESSAGE_HEADER_BYTES +  LENGTH_DEBUG_DATA_OUT_BYTES;
    
    # Initialize variables
    magicOK = 0 # Checks if magic number has been read
    dataOK = 0 # Checks if the data has been read correctly
    frameNumber = 0
    detObj = {}
    tlv_type = 0
    
    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
    byteCount = len(byteVec)
    
    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
        byteBufferLength = byteBufferLength + byteCount
        
    # Check that the buffer has some data
    if byteBufferLength > 16:
        
        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]

        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc+8]
            if np.all(check == magicWord):
                startIdx.append(loc)
               
        # Check that startIdx is not empty
        if startIdx:
            
            # Remove the data before the first start index
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength-startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                byteBufferLength = byteBufferLength - startIdx[0]
                
            # Check that there have no errors with the byte buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
                
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]
            
            # Read the total packet length
            totalPacketLen = np.matmul(byteBuffer[12:12+4],word)
            
            # Check that all the packet has been read
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    
    # If magicOK is equal to 1 then process the message
    if magicOK:
        # word array to convert 4 bytes to a 32 bit number
        word = [1, 2**8, 2**16, 2**24]
        
        # Initialize the pointer index
        idX = 0
        
        # Read the header
        magicNumber = byteBuffer[idX:idX+8]
        idX += 8
        version = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
        idX += 4
        totalPacketLen = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        platform = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
        idX += 4
        frameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        timeCpuCycles = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        numDetectedObj = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        numTLVs = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        # subFrameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        
        
        # Read the TLV messages
        for tlvIdx in range(numTLVs):
            
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]

            # Check the header of the TLV message
            try:
                tlv_type = np.matmul(byteBuffer[idX:idX+4],word)
                idX += 4
                tlv_length = np.matmul(byteBuffer[idX:idX+4],word)
                idX += 4
            except:
                pass
            
            # Read the data depending on the TLV message
            if tlv_type == VITAL_SIGNS_OUTPUT_TYPE:
                rangeBinIndexMax = byteBuffer[idX:idX + 2].view(dtype=np.uint16)[0]    
                idX += 2
                rangeBinIndexPhase = byteBuffer[idX:idX + 2].view(dtype=np.uint16)[0]    
                idX += 2
                maxVal = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                processingCyclesOut = byteBuffer[idX:idX + 4].view(dtype=np.uint32)[0]    
                idX += 4
                rangeBinStartIndex = byteBuffer[idX:idX + 2].view(dtype=np.uint16)[0]  
                idX += 2
                rangeBinEndIndex = byteBuffer[idX:idX + 2].view(dtype=np.uint16)[0]    
                idX += 2
                unwrapPhasePeak_mm = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                outputFilterBreathOut = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                outputFilterHeartOut = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                heartRateEst_FFT = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                heartRateEst_FFT_4Hz = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                heartRateEst_xCorr = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                heartRateEst_peakCount = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                breathingRateEst_FFT = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                breathingRateEst_xCorr = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                breathingRateEst_peakCount = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                confidenceMetricBreathOut = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                confidenceMetricBreathOut_xCorr = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                confidenceMetricHeartOut = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                confidenceMetricHeartOut_4Hz = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                confidenceMetricHeartOut_xCorr = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                sumEnergyBreathWfm = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                sumEnergyHeartWfm = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                motionDetectedFlag = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]    
                idX += 4
                Reserved = byteBuffer[idX:idX + 10*4].view(dtype=np.float32)   
                idX += 10*4

                # Store the data in the vitalSignsObj dictionary
                vitalSignsObj = {"BreathWfm": outputFilterBreathOut, "HeartWfm": outputFilterHeartOut, \
                                 "heartRateEst_FFT": heartRateEst_FFT, "heartRateEst_peakCount": heartRateEst_peakCount, \
                                 "breathingRateEst_FFT": breathingRateEst_FFT, "breathingRateEst_peakCount": breathingRateEst_peakCount}

                print("Heart rate: ", str(heartRateEst_FFT))
                print("Breath rate: ", str(breathingRateEst_FFT))

            elif tlv_type == RANGE_PROFILE_TYPE:
                numBytes = configParameters["numRangeBinProcessed"]*4

                payload = byteBuffer[idX:idX + numBytes]
                idX += numBytes 

                rangeProfile = payload.view(dtype=np.int16)
                rangeProfile_cplx = rangeProfile[::2] + rangeProfile[1::2]* 1j
                
                yRangePlot = np.absolute(rangeProfile_cplx)
                xRangePlot =  configParameters["rangeBinSize_meter"]*(configParameters["rangeStart_Index"] +np.array(range(len(rangeProfile_cplx))))

                dataOK = 1

        # Remove already processed data
        if idX > 0 and byteBufferLength > idX:
            shiftSize = totalPayloadSize_bytes
                    
            byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
            byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),dtype = 'uint8')
            byteBufferLength = byteBufferLength - shiftSize
            
            # Check that there are no errors with the buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
                

    return dataOK, frameNumber, vitalSignsObj

# ------------------------------------------------------------------

# Funtion to update the data and display in the plot
def update():
     
    dataOk = 0
    global vitalSignsObj, heartRateMat, frameMat
      
    # Read and parse the received data
    dataOk, frameNumber, vitalSignsObj = readAndParseVitalSigns(Dataport, configParameters)
    
    if dataOk:
        #print(vitalSignsObj)
        frameMat.append(frameNumber)
        heartRateMat.append(vitalSignsObj["heartRateEst_FFT"])
        
        s.setData(frameMat,heartRateMat)
        QtGui.QApplication.processEvents()
    
    return dataOk


# -------------------------    MAIN   -----------------------------------------  

# Configurate the serial port
CLIport, Dataport = serialConfig(configFileName)

# Get the configuration parameters from the configuration file
configParameters = parseConfigFile(configFileName)

# START QtAPPfor the plot
app = QtGui.QApplication([])

# Set the plot 
pg.setConfigOption('background','w')
win = pg.GraphicsWindow(title="2D scatter plot")
p = win.addPlot()
# p.setXRange(-0.5,0.5)
# p.setYRange(0,1.5)
p.setLabel('left',text = 'Heart rate (bpm)')
p.setLabel('bottom', text= 'Frames ')
s = p.plot([],[])
    
   
# Main loop 
vitalSignsObj = {}  
frameData = {}    
currentIndex = 0
heartRateMat = []
frameMat = []
while True:
    try:
        # Update the data and check if the data is okay
        dataOk = update()
        
        if dataOk:
            # Store the current frame into frameData
            frameData[currentIndex] = vitalSignsObj
            currentIndex += 1
        
        time.sleep(0.03) # Sampling frequency of 30 Hz
        
    # Stop the program and close everything if Ctrl + c is pressed
    except KeyboardInterrupt:
        CLIport.write(('sensorStop\n').encode())
        CLIport.close()
        Dataport.close()
        win.close()
        break
        
    





