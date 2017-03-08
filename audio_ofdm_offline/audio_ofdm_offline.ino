/** \mainpage FftMatlabTest.ino:

  The companion Matlab code implements the equivalent functions.

  Data is exchanged in the following sequence:
  - Receives first command to set gain (cmd = 0) or offset (cmd = 1)
  - Receives second command to set gain (cmd = 0) or offset (cmd = 1)
  - Receives input array of integers
  - Calculates the output = gain * input + offset
  - Sends output vector
*/

#include "Audio.h"
#include "serial_array.h"
#include <OLED.h>
#include <dsplib.h>
#include "sync_buffer.h"
// Namespaces
#include "QAM.h"
#include "SyncBlock.h"
#include "FEQ.h"
#include "RFFT.h"

// Helpful Definitions
#define FIXED_FBITS       15
#define WORD_LENGTH       16

// Use long long to multiply to simulate MAC behavior
static long long FIXED_MUL(int a, int b) {
    return ((long long)a*(long long)b);
}

static int FIXED_RND(long long a) {
    return ((a + (long long) (1 << (FIXED_FBITS - 1))) >> FIXED_FBITS);
}


// Baud rate
const long baudRate = 115200;

// Global variables
const int maxDataLength = 2048;


// Create instance of the serial command class
SerialCmd cmd(maxDataLength);

#pragma DATA_ALIGN(512)
int num_calls = 0;
int output[maxDataLength] = {0};
int syncSymbolFlipped[maxDataLength] = {0};
int syncSymbolFlippedLength;
int syncThresh;
int syncProportion;
int xcorrOutput[maxDataLength] = {0};
int xcorrLength;
int xcorrMaxVal;
int xcorrMaxInd;
int bufferMax;
int energy;
volatile ushort xcorrOF;

// FEQ Buffers
int FEQFrac[maxDataLength] = {0};
int FEQExp[maxDataLength] = {0};
int FEQLength;
// FEQ design
int refSymbolFFT[maxDataLength] = {0};
int refSymbolFFTLength;
int fftOutput[maxDataLength] = {0};
int fftOutputLength;

// QAM Buffers
int rx_data_qam[maxDataLength] = {0};
int rx_data_qamLength;
int demodSymbol[maxDataLength] = {0};
int demodSymbolLength;

// Control Flags
volatile bool feqdone = false;
volatile bool syncDone = false;
volatile int startOFDM = 0;
volatile int numSymbolsLeft = 0;

// Parameters
struct rx_settings {
    int cpSize;
    int fftSize;
    int qamSize;
    int dataCarrierLow;
    int dataCarrierHigh;
    int symbolsPerTx;
    int symbolLength;
    long fs;
} RS;

SyncBuffer buffer;

// Store output
int *decodedData;
int decodedDataLength;

// Audio Parameters
const int BufferLength = 512 + 32;

// Debug Buffers
int xcorrCaptureBuffer[2*BufferLength] = {0};

// Global variables used for inter-function communications
// These variables are declared as volatile so that the compiler does not optimize them away
// More specifically volatile variables are guaranteed to have visible side effects every time
// they are accessed
volatile int GetAudioBufferFlag = 0; // Trigger audio buffer capture in processAudio()
volatile int CapturedBufferFlag = 0; // Indicate that audio buffer has been captured in processAudio()
volatile int CaptureBufferIndex = 0;
volatile int CaptureTriggerFlag = 0;
volatile int CaptureThreshold   = 0;
const int    NumCaptureBuffers  = 8;

int captureTrigger(const int* leftBuffer, const int* rightBuffer, int length);

#pragma DATA_ALIGN(32)
int AudioCaptureBufferLeft[BufferLength * NumCaptureBuffers]  = {0};
#pragma DATA_ALIGN(32)
int AudioCaptureBufferRight[BufferLength * NumCaptureBuffers] = {0};

// ADC DC offset
int AdcDcOffsetLeft  = 0;
int AdcDcOffsetRight = 0;


/** \brief Setup function

  Allocate memory for input/output arrays.

  Initialize OLED display and serial communication module.

*/
void setup()
{
    // Initialize the display
    disp.oledInit();
    disp.clear();
    disp.setline(0);
    disp.setOrientation(1);
    disp.print("Ready");

    // Audio library is configured for non-loopback mode
    int status = AudioC.Audio(TRUE, BufferLength, BufferLength);
    // Set codec sampling rate:
    //   SAMPLING_RATE_8_KHZ
    //   SAMPLING_RATE_11_KHZ
    //   SAMPLING_RATE_12_KHZ
    //   SAMPLING_RATE_16_KHZ
    //   SAMPLING_RATE_22_KHZ
    //   SAMPLING_RATE_24_KHZ
    //   SAMPLING_RATE_32_KHZ
    //   SAMPLING_RATE_44_KHZ
    //   SAMPLING_RATE_48_KHZ (default)
    AudioC.setSamplingRate(SAMPLING_RATE_8_KHZ);

    // Set ADC input gain (range: 0 to 100)
    // Set to 0 for line-input, set to 100 for passive microphone
    AudioC.setInputGain(0, 0);

    // Connect to Matlab
    serial_connect(baudRate);
}

/** \brief Main application loop

  - Receives two serial commands from Matlab
    - Commands can set the gain or offset
    - 0: sets the filter coefficients and clears the filter state
    - 1: receives inout vector, filters input and sends output
*/
void loop()
{
    int command;
    int dataLength;
    const int *input;
    const long *inputLong;

    cmd.recv();
    command = cmd.getCmd();

    input = cmd.getDataIntPointer();
    inputLong = cmd.getDataLongPointer();
    dataLength = cmd.getDataIntLength();


    disp.clear();
    disp.setline(0);
    disp.print("Cmd ");
    disp.print((long)command);
    disp.print(": input = ");

    if (dataLength <= maxDataLength)
    {

        disp.setline(1);
        disp.print((long)dataLength);

        switch(command)
        {
            case 0: // Send rx param
                RS.cpSize = input[0];
                RS.fftSize = input[1];
                RS.qamSize = input[2];
                RS.dataCarrierLow = input[3];
                RS.dataCarrierHigh = input[4];
                RS.symbolsPerTx = input[5];
                // Parameter dependent setup
                RS.symbolLength = 2*RS.fftSize + RS.cpSize;
                startOFDM = 0;
                decodedData = new int[(RS.dataCarrierHigh - RS.dataCarrierLow + 1)*RS.symbolsPerTx];
                decodedDataLength = 0;
                output[0] = RS.cpSize;
                output[1] = RS.fftSize;
                output[2] = RS.qamSize;
                output[3] = RS.dataCarrierLow;
                output[4] = RS.dataCarrierHigh;
                output[5] = RS.symbolsPerTx;
                serial_send_array(output,6);
                break;
            case 1: // Setup Buffer
                buffer.setup(input[0]);
                output[0] = buffer.getNumElem();
                // Send output
                serial_send_array(output, dataLength);
                break;
            case 2: // Receive sync symbol
                syncSymbolFlippedLength = dataLength;
                for (int i = 0; i < dataLength; i++) {
                    syncSymbolFlipped[i] = input[i];
                    output[i] = input[i];
                }
                serial_send_array(output, dataLength);
                break;
            case 3: // receive fft of ref symbol for FEQ design
                refSymbolFFTLength = dataLength;
                for (int i = 0; i < refSymbolFFTLength; i++) {
                    refSymbolFFT[i] = input[i];
                    output[i] = input[i];
                }
                copyBuffer(refSymbolFFT,output,dataLength);
                serial_send_array(output, dataLength);
                break;
            case 4: // Calculate energy threshold
                energy = SyncBlock::calculateEnergy(syncSymbolFlipped,syncSymbolFlippedLength);
                syncProportion = input[0];
                syncThresh = FIXED_RND(FIXED_MUL(energy, syncProportion));
                output[0] = syncThresh;
                serial_send_array(output,1);
                break;
            /******************************************************
             ******************* DSP MODE 1 ***********************
             ******************************************************/
            case 10: // Insert data into buffer
                buffer.insert(input);
                const int* ptr1 = buffer.getFullBuffer();
                for(int n = 0; n < dataLength; n++) {
                    output[n] = ptr1[n];
                }
                // Send output
                serial_send_array(output, dataLength);
                break;
            case 20: // Calculate the cross correlation
                // The reason there is no -1 is b/c 0st elem does not overlap
                xcorrLength = 2*buffer.getNumElem();
                syncThresh = syncProportion;
                SyncBlock::xcorr(buffer.getFullBuffer(),syncSymbolFlipped,xcorrOutput,xcorrLength,syncSymbolFlippedLength);
                copyBuffer(xcorrOutput,output,xcorrLength);
                serial_send_array(output,xcorrLength);
                break;
            case 21: // find max index and value
                SyncBlock::absMax(xcorrOutput,xcorrLength,xcorrMaxVal,xcorrMaxInd);
                // Check if we need to update the Align index
                if ((xcorrMaxVal > syncThresh) && (xcorrMaxInd < RS.symbolLength + RS.cpSize + 1)) {
                    // The +1 is b/c we want syncSymbol to align with data
                    buffer.setAlignIndex(xcorrMaxInd);
                }
                output[0] = xcorrMaxVal;
                output[1] = xcorrMaxInd;
                serial_send_array(output,2);
                break;
            case 22: // Check the mean function
                copyBuffer(buffer.getFullBuffer(),output,2*buffer.getNumElem());
                serial_send_array(output,2*buffer.getNumElem());
                break;
            case 30: // Return the symbol at the aligned index of the sync_buffer
                const int *ptr = buffer.getAlignedBuffer();
                fftOutputLength = 2*RS.fftSize;
                for (int i = 0; i < 2*RS.fftSize; i++) {
                    // Store symbol in fftOutput for inplace fft
                    fftOutput[i] = ptr[i];
                    output[i] = ptr[i];
                }
                serial_send_array(output,2*RS.fftSize);
                break;
            case 40: // Take fft
                RFFT::rfft_noscale(fftOutput,fftOutputLength);
                for (int i = 0; i < fftOutputLength; i++) {
                    output[i] = fftOutput[i];
                }
                serial_send_array(output, fftOutputLength);
                break;
            case 50: // extract data carriers
                rx_data_qamLength = 2*(RS.dataCarrierHigh - RS.dataCarrierLow + 1);
                int index = 2*RS.dataCarrierLow;
                for (int i = 0; i < rx_data_qamLength; i++) {
                    rx_data_qam[i] = fftOutput[index];
                    output[i] = fftOutput[index];
                    index++;
                }
                serial_send_array(output, rx_data_qamLength);
                break;
            case 60: // FEQ design (get FEQ)
                FEQ::getFEQ(refSymbolFFT,rx_data_qam,FEQFrac,FEQExp,rx_data_qamLength);
                copyBuffer(FEQFrac,output,rx_data_qamLength);
                copyBuffer(FEQExp,output+rx_data_qamLength,rx_data_qamLength);
                serial_send_array(output, 2*rx_data_qamLength);
                break;
            case 70: // apply FEQ
                FEQ::applyFEQ(rx_data_qam,FEQFrac,FEQExp,rx_data_qamLength);
                copyBuffer(rx_data_qam,output,rx_data_qamLength);
                serial_send_array(output,rx_data_qamLength);
                break;
            case 80: // qamdemod
                QAM::demod(rx_data_qam,demodSymbol,rx_data_qamLength);
                demodSymbolLength = rx_data_qamLength/2;
                serial_send_array(demodSymbol, demodSymbolLength);
                break;
            /******************************************************
             ******************* DSP MODE 2 ***********************
             ******************************************************/
            case 210: //Send Buffer
                 buffer.insert(input);
                const int* ptr3 = buffer.getFullBuffer();
                for(int n = 0; n < dataLength; n++) {
                    output[n] = ptr3[n];
                }
                // Send output
                serial_send_array(output, dataLength);
                break;

            case 220: // Find sync (take xcorr and return max val and index)
              
                  
                  // Calculate the cross correlation
                // The reason there is no -1 is b/c 0st elem does not overlap
                xcorrLength = 2*buffer.getNumElem();
                syncThresh = syncProportion;
                SyncBlock::xcorr(buffer.getFullBuffer(),syncSymbolFlipped,xcorrOutput,xcorrLength,syncSymbolFlippedLength);
                copyBuffer(xcorrOutput,output,xcorrLength);
               
                // find max index and value
                SyncBlock::absMax(xcorrOutput,xcorrLength,xcorrMaxVal,xcorrMaxInd);
                // Check if we need to update the Align index
                if ((xcorrMaxVal > syncThresh) && (xcorrMaxInd < RS.symbolLength + RS.cpSize + 1)) {
                    // The +1 is b/c we want syncSymbol to align with data
                    buffer.setAlignIndex(xcorrMaxInd);
                }
                output[0] = xcorrMaxVal;
                output[1] = xcorrMaxInd;
                serial_send_array(output,2);
                break;
                
                

            case 230: // Take get alignedBuf + fft + extract data carrier
                //align buffers via found sync
                //demodulate given rfft
                //extract data carriers via complex mult of QAM?
             
                
                const int *ptr2 = buffer.getAlignedBuffer();
                fftOutputLength = 2*RS.fftSize;
                for (int i = 0; i < 2*RS.fftSize; i++) {
                    // Store symbol in fftOutput for inplace fft
                    fftOutput[i] = ptr2[i];
                }
                RFFT::rfft_noscale(fftOutput,fftOutputLength);

                rx_data_qamLength = 2*(RS.dataCarrierHigh - RS.dataCarrierLow + 1);
                int index2 = 2*RS.dataCarrierLow;
                for (int i = 0; i < rx_data_qamLength; i++) {
                    rx_data_qam[i] = fftOutput[index2];
                    output[i] = fftOutput[index2];
                    index2++;
                }
                serial_send_array(output, rx_data_qamLength);            
                break;

        case 240: // FEQ design (get FEQ)
            FEQ::getFEQ(refSymbolFFT,rx_data_qam,FEQFrac,FEQExp,rx_data_qamLength);
            copyBuffer(FEQFrac,output,rx_data_qamLength);
            copyBuffer(FEQExp,output+rx_data_qamLength,rx_data_qamLength);
            serial_send_array(output, 2*rx_data_qamLength);
            break;
        case 250: // apply FEQ
            FEQ::applyFEQ(rx_data_qam,FEQFrac,FEQExp,rx_data_qamLength);
            copyBuffer(rx_data_qam,output,rx_data_qamLength);
            serial_send_array(output,rx_data_qamLength);
            break;
        case 260: // qamdemod
            QAM::demod(rx_data_qam,demodSymbol,rx_data_qamLength);
            demodSymbolLength = rx_data_qamLength/2;
            serial_send_array(demodSymbol, demodSymbolLength);
            break;


            /******************************************************
             ******************* DSP MODE 3 ***********************
             ******************************************************/

              case 310: //Send Buffer
                 buffer.insert(input);
                const int* ptr4 = buffer.getFullBuffer();
                for(int n = 0; n < dataLength; n++) {
                    output[n] = ptr4[n];
                }
                
                  if( !syncDone){
                     xcorrLength = 2*buffer.getNumElem();
                  syncThresh = syncProportion;
                  SyncBlock::xcorr(buffer.getFullBuffer(),syncSymbolFlipped,xcorrOutput,xcorrLength,syncSymbolFlippedLength);
                  copyBuffer(xcorrOutput,output,xcorrLength);
                 
                  // find max index and value
                  SyncBlock::absMax(xcorrOutput,xcorrLength,xcorrMaxVal,xcorrMaxInd);
                  // Check if we need to update the Align index
                  if ((xcorrMaxVal > syncThresh) && (xcorrMaxInd < RS.symbolLength + RS.cpSize + 1)) {
                      // The +1 is b/c we want syncSymbol to align with data
                      buffer.setAlignIndex(xcorrMaxInd);
                      syncDone = true;
                  }
                  }else{
                    ptr2 = buffer.getAlignedBuffer();
                    fftOutputLength = 2*RS.fftSize;
                    for (int i = 0; i < 2*RS.fftSize; i++) {
                        // Store symbol in fftOutput for inplace fft
                        fftOutput[i] = ptr2[i];
                    }
                    RFFT::rfft_noscale(fftOutput,fftOutputLength);
        
                    rx_data_qamLength = 2*(RS.dataCarrierHigh - RS.dataCarrierLow + 1);
                    index2 = 2*RS.dataCarrierLow;
                    for (int i = 0; i < rx_data_qamLength; i++) {
                        rx_data_qam[i] = fftOutput[index2];
                        output[i] = fftOutput[index2];
                        index2++;
                    }
                   // FEQ design (get FEQ)
                     if(!feqdone){
                          FEQ::getFEQ(refSymbolFFT,rx_data_qam,FEQFrac,FEQExp,rx_data_qamLength);
                          copyBuffer(FEQFrac,output,rx_data_qamLength);
                          copyBuffer(FEQExp,output+rx_data_qamLength,rx_data_qamLength);
                          feqdone=true;
                     }                     
                  }
                      
              break;
                            
              case 320: 
                   //else{                 
                 // apply FEQ
                  FEQ::applyFEQ(rx_data_qam,FEQFrac,FEQExp,rx_data_qamLength);
                  copyBuffer(rx_data_qam,output,rx_data_qamLength);

               // qamdemod
                  QAM::demod(rx_data_qam,demodSymbol,rx_data_qamLength);
                  demodSymbolLength = rx_data_qamLength/2;
                  serial_send_array(demodSymbol, demodSymbolLength);
             
                  break;
                  
                    
              
              

            /******************************************************
             ******************* DSP MODE 4 ***********************
             *****************************************
             *************/
            case 400:
              //OPTIONAL:
               int dat_rate = *input;
               AudioC.setSamplingRate(dat_rate);
               disp.clear();
              disp.setline(0);
              disp.print("Sampling Rate:");
              disp.setline(1);
              disp.print((long)dat_rate);
              
              
               serial_send_array( &dat_rate, dataLength);
//              serial_send_array(input, dataLength);
//              serial_send_array( (Booint *)input, dataLength);
          //    serial_send_array(input, dataLength);
              break;
              
            case 410:
              startOFDM = 1;
              syncDone = 0;
              feqdone = 0;
              disp.clear();
              disp.setline(0);
              disp.print("Reciever Ready");
              num_calls = 0;
              break;
                
            case 420:
              serial_send_array(decodedData, decodedDataLength);
              break;
              

            /******************************************************
             ******************* DEBUG COMMANDS *******************
             ******************************************************/
            case 500: // Capture audio left/right buffers
                digitalWrite(LED0, HIGH);
                CaptureThreshold = input[0];
                GetAudioBufferFlag = 1;
                CapturedBufferFlag = 0;
                break;
            case 501: // Send captured audio left/right buffers
                // Send captured Audio Buffers to Matlab
                serial_send_array(AudioCaptureBufferLeft,  BufferLength * NumCaptureBuffers);
                serial_send_array(AudioCaptureBufferRight, BufferLength * NumCaptureBuffers);
                CapturedBufferFlag = 0;
                break;
            case 502: // Set ADC DC offset
                AdcDcOffsetLeft  = input[0];
                AdcDcOffsetRight = input[1];
                break;
            case 503: // Grab the xcorr output contents when sync symbol is found
                if (syncDone) {
                    serial_send_array(xcorrCaptureBuffer,2*BufferLength);
                }
              disp.clear();
              disp.setline(0);
              disp.print(long(num_calls));
              break;

            case 504:  // Set ADC input gain (range: 0 to 100)
                   // Set to 0 for line-input, set to 100 for passive microphone
                    if (dataLength == 1)
                        AudioC.setInputGain(input[0], input[0]);
                    else
                        if (dataLength == 2)
                            AudioC.setInputGain(input[0], input[1]);
                    break;


            default:
                disp.clear();
                disp.setline(0);
                disp.print("Unknown Command");
                break;
        }
    }
    else
    {
        disp.setline(1);
        disp.print("Invalid!");
    }
}

// Assume mono, only work on inputLeft
void processAudio() {

    // DC offset removal
    for(int n = 0; n < BufferLength; n++)
    {
        AudioC.inputLeft[n]  = AudioC.inputLeft[n]  - AdcDcOffsetLeft;
        AudioC.inputRight[n] = AudioC.inputRight[n] - AdcDcOffsetRight;
    }
    num_calls= num_calls+1;

    // Receiver state machine
    if (startOFDM) {
        buffer.insert(AudioC.inputLeft);
        // Calculate the cross correlation
          if( !syncDone){
                  xcorrLength = 2*buffer.getNumElem();
                  syncThresh = syncProportion;
                  SyncBlock::xcorr(buffer.getFullBuffer(),syncSymbolFlipped,xcorrOutput,xcorrLength,syncSymbolFlippedLength);
                  copyBuffer(xcorrOutput,output,xcorrLength);
                 
                  // find max index and value
                  SyncBlock::absMax(xcorrOutput,xcorrLength,xcorrMaxVal,xcorrMaxInd);
                  // Check if we need to update the Align index
                  if ((xcorrMaxVal > syncThresh) && (xcorrMaxInd < RS.symbolLength + RS.cpSize + 1)) {
                      // The +1 is b/c we want syncSymbol to align with data
                      buffer.setAlignIndex(xcorrMaxInd);
                      syncDone = true;
                  }
          }else{
            const int * ptr = buffer.getAlignedBuffer();
            fftOutputLength = 2*RS.fftSize;
            for (int i = 0; i < 2*RS.fftSize; i++) {
                // Store symbol in fftOutput for inplace fft
                fftOutput[i] = ptr[i];
            }
            RFFT::rfft_noscale(fftOutput,fftOutputLength);

            rx_data_qamLength = 2*(RS.dataCarrierHigh - RS.dataCarrierLow + 1);
            int index = 2*RS.dataCarrierLow;
            for (int i = 0; i < rx_data_qamLength; i++) {
                rx_data_qam[i] = fftOutput[index];
                output[i] = fftOutput[index];
                index++;
            }
           // FEQ design (get FEQ)
             if(!feqdone){
                  FEQ::getFEQ(refSymbolFFT,rx_data_qam,FEQFrac,FEQExp,rx_data_qamLength);
                  copyBuffer(FEQFrac,output,rx_data_qamLength);
                  copyBuffer(FEQExp,output+rx_data_qamLength,rx_data_qamLength);
                  feqdone=true;
             }else{
               // Apply FEQ
                FEQ::applyFEQ(rx_data_qam,FEQFrac,FEQExp,rx_data_qamLength);
                // Apply qamdemod
                demodSymbolLength = rx_data_qamLength/2;
                QAM::demod(rx_data_qam,demodSymbol,rx_data_qamLength);
                // Copy the result into the decodedData vector
                copyBuffer(demodSymbol,decodedData+decodedDataLength,demodSymbolLength);
                decodedDataLength += demodSymbolLength;
                numSymbolsLeft--;
                if (numSymbolsLeft == 0) {
                    startOFDM = 0;
                    // Disp that we've finished
                    disp.clear();
                    disp.setline(0);
                    disp.print("Done!");
                }
                
                  
               
             }


               
          }
    
                             
    }

    // Audio buffer capture
    if (GetAudioBufferFlag)
    {
        if (CaptureTriggerFlag == 0)
        {
            CaptureTriggerFlag = captureTrigger(AudioC.inputLeft, AudioC.inputRight, BufferLength);
        }

        if (CaptureTriggerFlag == 1)
        {
            CaptureBufferIndex += 1;
            for(int n = 0; n < BufferLength; n++)
            {
                AudioCaptureBufferLeft[n  + CaptureBufferIndex * BufferLength] = AudioC.inputLeft[n];
                AudioCaptureBufferRight[n + CaptureBufferIndex * BufferLength] = AudioC.inputRight[n];
            }

            if (CaptureBufferIndex >= NumCaptureBuffers - 1)
            {
                GetAudioBufferFlag = 0;
                CapturedBufferFlag = 1;
                CaptureBufferIndex = 0;
                CaptureTriggerFlag = 0;
                digitalWrite(LED0, LOW);
            }
        }
        else
        {
            for(int n = 0; n < BufferLength; n++)
            {
                AudioCaptureBufferLeft[n]  = AudioC.inputLeft[n];
                AudioCaptureBufferRight[n] = AudioC.inputRight[n];
            }
        }
    }
    else
    {   // Record the last buffer in the background
        for(int n = 0; n < BufferLength; n++)
        {
            AudioCaptureBufferLeft[n]  = AudioC.inputLeft[n];
            AudioCaptureBufferRight[n] = AudioC.inputRight[n];
        }
    }
}

int captureTrigger(const int* leftBuffer, const int* rightBuffer, int length)
{
    int flag = 0;

    for(int n = 0; n < length; n++)
    {
        if ((abs(leftBuffer[n]) >= CaptureThreshold) || (abs(rightBuffer[n]) >= CaptureThreshold))
        {
            flag = 1;
        }
    }
    return flag;
}

void copyBuffer(const int* source, int* dest, int length) {
    for (int i = 0; i < length; i++) {
        dest[i] = source[i];
    }
}
