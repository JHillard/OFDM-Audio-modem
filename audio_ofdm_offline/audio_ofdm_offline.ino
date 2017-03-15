/** \file audio_ofdm_offline.ino
/** \mainpage audio_ofdm_offline.ino:

  OFDM Modem; Project API.

  This project implements an orthogonal frequency division multiplexing OFDM
  modem. These modems are a staple in telecommunications infrastructure and
  provide an excellent tech demonstration for the power of the Fast Fourier
  Transform FFT (please see companion report for details.)

  This project holds four distinct modes of operation. Modes one through three
  primarily verify individual system functionality via the Matlab
  test harness (discussed in a moment). These modes are distinguished by serial
  commands in the range of 10's, 200's, and 300's respectively. Mode 4 executes
  commands in 400 range, and here actual OFDM control occurs. All
  other commands are built for the matlab test harness.

  The matlab companion code test harness gives serial commands to the OFDM modem.
  It is also the transmitter side of the telecommunications demonstration, encoding
  the information and translating that into audio for the laptop to play.


\section Commands
  - command 0: Setup RX parameters.
  - command 1: Setup buffer.
  - command 2: Recieve sync symbol and store for later use.
  - command 3: Receive fft of reference symbol for FEQ design
  - command 4: Calculate the energy threshold for  the sync symbol.

  \subsection Mode1 DSP Mode 1 Commands:
  - command 10: Insert data into buffer.
  - command 20: Calculate the cross correlation between the buffer and the syncSymbol.
  - command 21: Find the max value and index of the cross correlation to compare to threshold.
  - command 22: Return the mean value of the buffer.
  - command 30: Return the FFT of the symbol at the synchronized index of the buffer.
  - command 40: Take the FFT of the variable output.
  - command 50: extract the data carriers by slicing the array containing the FFT of the output variable. These slices correspond to the carrier frequencies.
  - command 60: Designs the FEQ and returns it as an array.
  - command 70: Applies the FEQ to the extracted data carriers.  Returns an array of the result
  - command 80: Demodulates the QAM and sends the result as an array.
 \subsection Mode2 DSP Mode 2 Commands:
  - command 210: Sends the values found in the output buffer.
  - command 220: Synchronizes by taking the correlation and returning the max value and index of the operation.
  - command 230: Gets the synchronized buffer, performs a FFT, and extracts the data carriers.
  - command 240: Designs the FEQ and returns it as an array.
  - command 250: Applies the FEQ to the extracted data carriers.  Returns an array of the result
  - command 260: Demodulates the QAM and sends the result as an array.
  \subsection Mode3 DSP Mode 3 commands:
  - command 310: Applies state machine for the OFDM modem. The command retrieves the buffer and synchronizes on it if it hasn't already. Upon synchronization the command swithces states and takes the FFT of the buffer, and extracts the carrier frequencies. These carrier frequencies are then passed to the FEQ design block (assuming the FEQ has not already been designed). Essentially, this command prepares the modem to demodulate the data symbols inside a single transmission.
  - command 320: Applies the designed FEQ, and then demodualtes the filtered signal with the QAM:demod command. Command returns a demodulated data symbol.
  \subsection Mode4 DSP Mode 4 commands:
  - command 400: Sets the sampling rate for the Audio.C library.
  - command 410: Sets the state machine appropriately for the OFDM modem to begin its process of syncing, designing the FEQ, and demodulating the QAM data. The bulk of this work is done in the processAudio() ISR.
  - command 420: Sends the decoded data to the matlab test harness.
  \subsection ModeD  Debug Commands (used to tune the volumes and gains of the transmitter and reciever.):
  - command 500: Sets the states to begin capturing data from the AudioC library.
  - command 501: Sends the captured data to the matlab interface. Used to test for clipping and adjusting volumes.
  - command 502: Sets the offset of the ADC since its zero isn't truly at zero.
  - command 503: returns to matlab the outputs of the correlation used to find the sync symbol.
  - command 504: Sets the gain of the ADC input. Set to 0 for line-input (audio cord), set to 100 for a passive microphone.

  Note: This experiment uses Q.15 fixed-point format throughout its code.

  \section Mathematics

  \subsection fft RFFT:
    DFT algorithm used by RFFT.
    \f$ y[k] = \frac{1}{scale_factor}  \sum_{i=0}^{nx-1}( cos(\frac{-2\pi ik}{nx}) + j sin( \frac{-2\pi i k}{nx} )) \f$

    IDFT algorithm used by RIFFT.
    \f$ y[k] = \frac{1}{scale_factor}  \sum_{i=0}^{nx-1}( cos(\frac{2\pi ik}{nx}) + j sin( \frac{2\pi i k}{nx} )) \f$

    Complex mulitiplication algorithm:

    Input \f$in1\f$ and \f$in2\f$ are broken into \f$(re1,im1)\f$ and \f$(re2,im2)\f$ respectively. The output
    \f$ y[n] = (rout[n],iout[n]) \f$  is defined as follows:

    \f$ rout[n] = re1[n]*re2[n] -im1[n]*im2[n] \f$

    \f$ iout[n] = re1[n]*im2[n] + re2[n]*im1[n] \f$

  Windowing is accomplished via complex multiplication of an input sequence and
  a window via the above method.

  Circular convolution is accomplished via DFTs of the input sequence,
  a complex multiplication between each other, and an IDFT to pull it
  back into the time domain. The algorithms behind each individual  step is
   described above.

   \subsection cor Correlation:
The Cross-correlation is used to find the sync pulse when initial clock reovery
is required. Typically, we'll use this to perform the cross-correlation on an input data stream
and the stored syncSymbol. The cross-correlation is defined as:

\f$(f\star g)[n] = \sum_{n = -\infty}^{\infty} f^{*}[m]g[m+n] \f$

But because our signals are real, we can ignore the complex congugate. Therefore, for our application:

\f$(f\star g)[n] = \sum_{n = -\infty}^{\infty} f[m]g[m+n] \f$

And specifically, we have stored our syncSymbol in reversed order, which means if we set g[n] = syncSymbol, then
the cross-correlation can be computed with a simple convolution;

\f$(f\star g)[n] = (f * g)[n] =  \sum_{n = -\infty}^{\infty} f[m]g[n-m] \f$

This greatly simplifies our compute constraints because the TI library has a highly efficient implementation of a convolution.
We use convol1() to perform this operation quickly and effectively.

\section Scaling
  Each transform function above has an option to SCALE or NOSCALE.

   FFT transformations (fft(), cfft(), rfft())
  apply scaling as the following:
   - NOSCALE:  \f$X[k] = \sum_{n = 0}^{N-1} x[n] e^{-j 2 k n / N}\f$
   - SCALE:  \f$X[k] = \frac{1}{N} \sum_{n = 0}^{N-1} x[n] e^{-j 2 k n / N}\f$

  For inverse FFt transromations (ifft(), rifft(), cifft() ):
   - NOSCALE: \f$x[n] = \sum_{k = 0}^{N-1} X[k] e^{j 2 k n / N}\f$
   - SCALE: \f$x[n] = \frac{1}{N} \sum_{k = 0}^{N-1} X[k] e^{j 2 k n / N}\f$




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

// Use long long data types, but round when appropriate. Typically used after
//the MAC operation simulated by FIXED_MUL().
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
    AudioC.setSamplingRate(SAMPLING_RATE_48_KHZ);

    // Set ADC input gain (range: 0 to 100)
    // Set to 0 for line-input, set to 100 for passive microphone
    AudioC.setInputGain(0, 0);

    // Connect to Matlab
    serial_connect(baudRate);
}

/** \brief Main application loop

  - Receives all serial commands from Matlab. For a list of all commands, please visit the main page, section 'Commands'.
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
                numSymbolsLeft = RS.symbolsPerTx;
                // Parameter dependent setupymbol
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
/** \brief
ISR triggered by the AudioC library. The processAudio() method by default migrates all the captured
audio data into buffers useable by the OFDM. When triggered, this method also contains calls to the
OFDM methods. It will perform the synchronization, design the FEQ and apply and demodulate the data
symbols appropriately.

It accepts and returns no inputs. All varaibles are modified in place and access by the serial commands outlined in the commands sections.
*/
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
        if (!syncDone) {
            xcorrLength = 2*buffer.getNumElem();

            syncThresh = syncProportion;

            xcorrOF = SyncBlock::xcorr(buffer.getFullBuffer(),syncSymbolFlipped,xcorrOutput,xcorrLength,syncSymbolFlippedLength);

            SyncBlock::absMax(xcorrOutput,xcorrLength,xcorrMaxVal,xcorrMaxInd);
            // Check if we need to update the Align index
            if ((xcorrMaxVal > syncThresh) && (xcorrMaxInd < RS.symbolLength + RS.cpSize + 1)) {
                // First Check if there was overflow in the cross correlation calculation
                if (xcorrOF == 1) {
                    disp.clear();
                    disp.setline(0);
                    disp.print("Xcorr OFlow");
                    disp.setline(1);
                    disp.print("Lower Vol.");
                    disp.clear();
                    disp.setline(0);
                    disp.print("OFDM: RUNNING");
                } else {
                    buffer.setAlignIndex(xcorrMaxInd);
                    syncDone = true;

                    // Capture cross correlation output
                    for(int i = 0; i < 2*BufferLength; i++) {
                        xcorrCaptureBuffer[i] = xcorrOutput[i];
                    }
                }
            }
        } else {
            // Get aligned buffer
            const int *ptr = buffer.getAlignedBuffer();
            fftOutputLength = 2*RS.fftSize;
            for (int i = 0; i < 2*RS.fftSize; i++) {
                // Store symbol in fftOutput for inplace fft
                fftOutput[i] = ptr[i];
            }
            // Take fft
            RFFT::rfft_noscale(fftOutput,fftOutputLength);
            // extract data carriers
            rx_data_qamLength = 2*(RS.dataCarrierHigh - RS.dataCarrierLow + 1);
            int index = 2*RS.dataCarrierLow;
            for (int i = 0; i < rx_data_qamLength; i++) {
                rx_data_qam[i] = fftOutput[index];
                index++;
            }
            // Design FEQ if necessary
            if (!feqdone) {
                FEQ::getFEQ(refSymbolFFT,rx_data_qam,FEQFrac,FEQExp,rx_data_qamLength);
                feqdone = true;
            } else {
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
/** \brief
captureTrigger monitors the values found in the audio buffers. When any value
found is above CaptureThreshold, the flag is set and processAudio begins sampling.
This helps filter out all the low level noise we might see when nothing is being transmitted.

It recieves pointers to the left and right buffers, along with their length. It outputs a 1 if
the modem should start capturing the data, otherwise, returns 0.
*/
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
/** \brief
copyBuffer copies the buffer of one array to another. It recieves two input pointers and a length, and copies all values source[0<n<length-1] to the destination.
*/
void copyBuffer(const int* source, int* dest, int length) {
    for (int i = 0; i < length; i++) {
        dest[i] = source[i];
    }
}
