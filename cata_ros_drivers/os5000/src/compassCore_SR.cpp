/*------------------------------------------------------------------------------
 *  Title:        compassCore_SR.cpp
 *  Description:  Sending and receiving data with the Ocean Server compass. The
 * 		   		  API is described in the document
 *         		  OS5000_Compass_Manual.pdf. See
 *         		  <a href="http://www.ocean-server.com/download/OS5000_Compass_Manual.pdf">Ocean Server</a>
 *         		  for more details.
 *----------------------------------------------------------------------------*/

/*
 *
 *      Copyright (c) 2010 <iBotics -- www.sdibotics.org>
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of the Stingray, iBotics nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <os5000/compassCore_SR.h>


/*------------------------------------------------------------------------------
 * Compass()
 * Constructor.
 *----------------------------------------------------------------------------*/

Compass::Compass(string _portname, int _baud, int _rate, int _initTime) : Serial::Serial(_portname, _baud)
{
    // Initialize variables.
    portname = _portname;
    baud = _baud;
    rate = _rate;
    initTime = _initTime;

    // Start a new timer.
    timer = new Timing(initTime);

    // Set up the compass.
    Setup();
} // end Compass()


/*------------------------------------------------------------------------------
 * ~Compass()
 * Destructor.
 *----------------------------------------------------------------------------*/

Compass::~Compass()
{
    // Close the open file descriptors.
    if (fd > 0)
    {
        close(fd);
    }
    if (Serial::fd > 0)
    {
        close(Serial::fd);
    }
    delete timer;
} // end ~Compass()


/*------------------------------------------------------------------------------
 * void Setup()
 * Initializes communications with the Ocean Server compass. Sets up a file
 * descriptor for further communications.
 *----------------------------------------------------------------------------*/

void Compass::Setup()
{
    // Declare variables.
    bCompassInitialized = false;

    // Check if the compass serial port was opened correctly.
    if (Serial::fd > 0)
    {
        // Set the message rate to control how often the compass will send out updates.
        SetRate();
        
        // Initialize the timer.
        timer->Set();
        
        // Check for valid compass data until the timer expires or valid data is found.
        while (!timer->CheckExpired())
        {
            // Try to initialize the compass to make sure that we are actually getting valid data from it.
            Init();
            if (bCompassInitialized)
            {
                break;
            }
        }

        // No valid compass data found so simulate compass data in the main loop.
        if (!bCompassInitialized)
        {
            // Seed the random number generator to use when simulating data.
            srand((unsigned int)time(NULL));
            Serial::fd = -1;
        }
    }
    else
    {
        // Seed the random number generator to use when simulating data.
        srand((unsigned int)time(NULL));
    }

    // Set the file descriptor.
    fd = Serial::fd;
} // end Setup()


/*------------------------------------------------------------------------------
 * void GetData()
 * Get orientation data from compass.
 *----------------------------------------------------------------------------*/

void Compass::GetData()
{
    // Declare variables.
    int bytesToDiscard = 0;
    bFoundCompleteMessage = false;

    // Get the number of bytes available on the serial port.
    GetBytesAvailable();
    
    // Make sure we don't read too many bytes and overrun the buffer.
    if (bytesAvailable >= SERIAL_MAX_DATA)
    {
        bytesAvailable = SERIAL_MAX_DATA - 1;
    }
    if (bytesAvailable + strlen(bufRecv) >= SERIAL_MAX_DATA)
    {
        bytesToDiscard = bytesAvailable + strlen(bufRecv) - SERIAL_MAX_DATA - 1;
        memmove(bufRecv, &bufRecv[bytesToDiscard], bytesToDiscard);
    }

    // Read data off the serial port.
    if (bytesAvailable > 0)
    {
        Recv();
        // Look for entire message.
        FindMsg();
        if (bFoundCompleteMessage)
        {
            // Parse the message.
            ParseMsg();
        }
    }
} // end GetData()


/*------------------------------------------------------------------------------
 * void SetRate()
 * Set the rate at which messages are sent by the compass.
 *----------------------------------------------------------------------------*/

void Compass::SetRate()
{
    // Declare variables.
    char cmd = 0;
    Serial::bufSend = &cmd;

    // Send command.
    if (Serial::fd > 0)
    {
        // Send out escape and R commands.
        cmd = OS5000_CMD_ESC;
        Serial::lengthSend = 1;
        Send();
        usleep(OS5000_SERIAL_DELAY);
        cmd = OS5000_CMD_SET_RATE;
        lengthSend = 1;
        Send();
        usleep(OS5000_SERIAL_DELAY);

        // Compute the rate command.
        if (rate <= 0)
        {
            cmd = '0';
            Serial::lengthSend = 1;
            Send();
            usleep(OS5000_SERIAL_DELAY);
        }
        else if (rate >= 40)
        {
            cmd = '4';
            Serial::lengthSend = 1;
            Send();
            usleep(OS5000_SERIAL_DELAY);
            cmd = '0';
            Serial::lengthSend = 1;
            Send();
            usleep(OS5000_SERIAL_DELAY);
        }
        else if (rate >= 30)
        {
            cmd = '3';
            Serial::lengthSend = 1;
            Send();
            usleep(OS5000_SERIAL_DELAY);
            cmd = '0';
            Serial::lengthSend = 1;
            Send();
            usleep(OS5000_SERIAL_DELAY);
        }
        else if (rate >= 20)
        {
            cmd = '2';
            Serial::lengthSend = 1;
            Send();
            usleep(OS5000_SERIAL_DELAY);
            cmd = '0';
            Serial::lengthSend = 1;
            Send();
            usleep(OS5000_SERIAL_DELAY);
        }
        else if (rate >= 10)
        {
            cmd = '1';
            Serial::lengthSend = 1;
            Send();
            usleep(OS5000_SERIAL_DELAY);
            cmd = rate + 38;
            Serial::lengthSend = 1;
            Send();
            usleep(OS5000_SERIAL_DELAY);
        }
        else
        {
            cmd = rate + 48;
            Serial::lengthSend = 1;
            Send();
            usleep(OS5000_SERIAL_DELAY);
        }
        cmd = OS5000_CMD_END;
        lengthSend = 1;
        Send();
        usleep(OS5000_SERIAL_DELAY);
    }
} // end SetRate()


/*------------------------------------------------------------------------------
 * void FindMsg()
 * Searches a buffer looking for the start and end sequences.
 *----------------------------------------------------------------------------*/

void Compass::FindMsg()
{
    // Declare variables.
    bool bFoundStart = false;
    int i = 0;
    int msgStartLocation = 0;
    bFoundCompleteMessage = false;

    // Look for start character.
    for (i = 0; i < Serial::lengthRecv; i++)
    {
        if (bFoundStart)
        {
            if (Serial::bufRecv[i] == OS5000_RESPONSE_END)
            {
                bFoundCompleteMessage = true;
                bFoundStart = false;
                // Place the last complete message at the beginning of the buffer and throw away any data before that.
                memmove(Serial::bufRecv, Serial::bufRecv + msgStartLocation, Serial::lengthRecv - msgStartLocation);
            }
        }
        // Look for start sequence. Don't assume that there is an end response before another start response.
        if (Serial::bufRecv[i] == OS5000_RESPONSE_START)
        {
            bFoundStart = true;
            msgStartLocation = i;
        }
    }
} // end FindMsg()


/*------------------------------------------------------------------------------
 * void ParseMsg()
 * Parses a message for compass data.
 *----------------------------------------------------------------------------*/

void Compass::ParseMsg()
{
    // Declare variables.
    int i = 0;
    char *token;
    char *saveptr;
    char *delim = (char *)OS5000_DELIM;
    char *tokens[OS5000_STRING_SIZE];

    // Initialize variables.
    memset(tokens, 0, sizeof(tokens));

    // Split the incoming buffer up into tokens based on the delimiting characters.
    tokens[0] = strtok_r(Serial::bufRecv, delim, &saveptr);
    for (i = 1; i < Serial::lengthRecv; i++)
    {
        token = strtok_r(NULL, delim, &saveptr);
        if (token == NULL)
        {
            break;
        }
        tokens[i] = token;
    }

    // Store Euler angle and temperature values.
    yaw   = atof(tokens[0]);
    pitch = atof(tokens[1]);
    roll  = atof(tokens[2]);
    temperature  = atof(tokens[3]);
} // end ParseMsg()


/*------------------------------------------------------------------------------
 * void SimulateData()
 * Simulates compass data when no compass is available.
 *----------------------------------------------------------------------------*/

void Compass::SimulateData()
{
    pitch = pitch + rand() / (float)RAND_MAX;
    roll  = roll  + rand() / (float)RAND_MAX;
    yaw   = yaw   + rand() / (float)RAND_MAX;
    temperature  = 0     + rand() / (float)RAND_MAX;

    return;
} // end SimulateData()


/*------------------------------------------------------------------------------
 * void Init()
 * Looks for valid data from the compass.
 *----------------------------------------------------------------------------*/

void Compass::Init()
{
    // Initialize variables.
    bCompassInitialized = false;

    // Get the number of bytes available on the serial port.
    tcflush(Serial::fd, TCIFLUSH);
    usleep(OS5000_SERIAL_DELAY);
    GetBytesAvailable();

    // Read data off the serial port.
    if (Serial::bytesAvailable > 0)
    {
        Serial::lengthRecv = Serial::bytesAvailable;
        Recv();
        // Look for entire message.
        FindMsg();
        if (bFoundCompleteMessage)
        {
            // Parse the message.
            ParseMsg();
            bCompassInitialized = true;
        }
    }
} // end Init()
