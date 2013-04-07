#include <WProgram.h>
#include "./RadioTerminal.h"
#include "./nRF24L01P_defs.h"
#include "./PITimer.h"
#include "./SPI.h"
#include <cctype>
#include <cstring>



namespace RadioTerminal
{
    // Internal functions and variables
    int getRegister(int address);
    int getStatus();
    void setRegister(int address, int data);
    void receive();
    void clear();
	void transmit(uint32_t data);
	void receiveChar(char c);

    int _csnPin;
    int _cePin;
    int _irqPin;
    volatile uint32_t rx_controller;
	
	uint8_t channel = 38;
	uint8_t controller = 0;
	uint32_t controllerBaseAddress = 0x0001a4;
	uint32_t rxAddress = 0xd091bb;
	uint32_t txAddress = 0xe7e1fa;
	
	char inputBuffer[INPUT_BUFFER_MAX];
	CmdHandler* runningCmd;
    
    struct cmd
    {
        char cmdString[INPUT_BUFFER_MAX];
        int stringLength;
        CmdHandler* (*fpointer)(const char*);
        cmd() { stringLength = 0; }
    };
    
    cmd cmdList[NUM_COMMANDS_MAX];
    int numCommands;



    void initialize(int csnPin, int cePin, int irqPin)
    {
        _csnPin = csnPin;
        _cePin = cePin;
        _irqPin = irqPin;
        
        // Set up pins
        pinMode(_csnPin, OUTPUT);
        pinMode(_cePin, OUTPUT);
        pinMode(_irqPin, INPUT);

        // Disable nRF24L01+
        digitalWrite(_cePin, 0);
        
        // Disable chip select
        digitalWrite(_csnPin, 1);
        
        // Set up SPI
        SPI.begin();
        SPI.setClockDivider(SPI_CLOCK_DIV8);
        
        // Set up IRQ
        attachInterrupt(irqPin, receive, FALLING);
        
        // Set up clear timer
        PITimer0.period(0.5f);
        PITimer0.start(&clear);
		
		// These values need to be initialized
		numCommands = 0;
		runningCmd = NULL;
		inputBuffer[0] = '\0';
    }



    void reset()
    {
        // Wait for power on reset
        delayMicroseconds(TIMING_Tpor);

        // Put into standby
        digitalWrite(_cePin, 0);
        
        // Configure registers
        setRegister(CONFIG, CONFIG_MASK_TX_DS | CONFIG_MASK_MAX_RT | CONFIG_EN_CRC | CONFIG_PWR_UP | CONFIG_PRIM_RX);
        setRegister(EN_AA, 0x00);
        setRegister(EN_RXADDR, ERX_P0 | ERX_P1);
        setRegister(SETUP_AW, SETUP_AW_3BYTES);
        setRegister(SETUP_RETR, 0x00);
        setRegister(RF_CH, channel);
        setRegister(RF_SETUP, RF_SETUP_RF_DR_HIGH | RF_SETUP_RF_PWR_0);
        setRegister(STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
        setRegister(RX_PW_P0, 4);
        setRegister(RX_PW_P1, 4);
        setRegister(DYNPD, 0x00);
        setRegister(FEATURE, 0x00);
        
        // Set addresses
        digitalWrite(_csnPin, 0);
        SPI.transfer(W_REGISTER | RX_ADDR_P0);
        SPI.transfer(controller & 0x0f);
        SPI.transfer((controllerBaseAddress >>  8) & 0xff);
        SPI.transfer((controllerBaseAddress >> 16) & 0xff);
        digitalWrite(_csnPin, 1);
        digitalWrite(_csnPin, 0);
        SPI.transfer(W_REGISTER | RX_ADDR_P1);
        SPI.transfer((rxAddress >>  0) & 0xff);
        SPI.transfer((rxAddress >>  8) & 0xff);
        SPI.transfer((rxAddress >> 16) & 0xff);
        digitalWrite(_csnPin, 1);
        digitalWrite(_csnPin, 0);
        SPI.transfer(W_REGISTER | TX_ADDR);
        SPI.transfer((txAddress >>  0) & 0xff);
        SPI.transfer((txAddress >>  8) & 0xff);
        SPI.transfer((txAddress >> 16) & 0xff);
        digitalWrite(_csnPin, 1);
        
        // Put into PRX
        digitalWrite(_cePin, 1);
        delayMicroseconds(TIMING_Tstby2a);
        
        // Flush FIFOs
        digitalWrite(_csnPin, 0);
        SPI.transfer(FLUSH_TX);
        digitalWrite(_csnPin, 1);
        digitalWrite(_csnPin, 0);
        SPI.transfer(FLUSH_RX);
        digitalWrite(_csnPin, 1);
		
		write("\n> ");
    }
	
	
	
	void addCommand(const char* cmdString, CmdHandler* (*fpointer)(const char*) )
	{
		// Check if there's room in the command array before adding a command
		if (numCommands < NUM_COMMANDS_MAX)
		{
			strncpy(cmdList[numCommands].cmdString, cmdString, INPUT_BUFFER_MAX);
			cmdList[numCommands].cmdString[INPUT_BUFFER_MAX - 1] = '\0'; // Make sure that the command string is null terminated
			cmdList[numCommands].stringLength = strlen(cmdList[numCommands].cmdString);
			cmdList[numCommands].fpointer = fpointer;
			numCommands++;
		}
		else
		{
			write("error: too many commands\n");
		}
	}



    void transmit(uint32_t data)
    {
        // Put into standby
        digitalWrite(_cePin, 0);
        
        // Configure for PTX
        int config = getRegister(CONFIG);
        config &= ~CONFIG_PRIM_RX;
        setRegister(CONFIG, config);
        
        // Write packet data
        digitalWrite(_csnPin, 0);
        SPI.transfer(W_TX_PAYLOAD);
        SPI.transfer( (data>>0) & 0xff );
        SPI.transfer( (data>>8) & 0xff );
        SPI.transfer( (data>>16) & 0xff );
        SPI.transfer( (data>>24) & 0xff );
        digitalWrite(_csnPin, 1);
        
        // Put into PTX
        digitalWrite(_cePin, 1);
        delayMicroseconds(TIMING_Tstby2a);
        digitalWrite(_cePin, 0);
        
        // Wait for message transmission and put into PRX
        delayMicroseconds(TIMING_Toa);
        config = getRegister(CONFIG);
        config |= CONFIG_PRIM_RX;
        setRegister(CONFIG, config);
        setRegister(STATUS, STATUS_TX_DS);
        digitalWrite(_cePin, 1);
    }



    int getRegister(int address)
    {
        digitalWrite(_csnPin, 0);
        int rc = R_REGISTER | (address & REGISTER_ADDRESS_MASK);
        SPI.transfer(rc);
        int data = SPI.transfer(NOP);
        digitalWrite(_csnPin, 1);
        return data;
    }



    int getStatus()
    {
        digitalWrite(_csnPin, 0);
        int status = SPI.transfer(NOP);
        digitalWrite(_csnPin, 1);
        return status;
    }



    void setRegister(int address, int data)
    {
        bool enabled = false;
        if (digitalRead(_cePin) == 1)
        {
            enabled = true;
            digitalWrite(_cePin, 0);
        }
        
        digitalWrite(_csnPin, 0);
        int rc = W_REGISTER | (address & REGISTER_ADDRESS_MASK);
        SPI.transfer(rc);
        SPI.transfer(data & 0xff);
        digitalWrite(_csnPin, 1);
        
        if (enabled)
        {
            digitalWrite(_cePin, 1);
            delayMicroseconds(TIMING_Tpece2csn);
        }
    }



    void receive()
    {
        uint32_t data = 0;
        int pipe;
        
        while (!(getRegister(FIFO_STATUS) & FIFO_STATUS_RX_EMPTY))
        {
            // Check data pipe
            pipe = getStatus() & STATUS_RN_P_MASK;
            
            // Read data
            digitalWrite(_csnPin, 0);
            SPI.transfer(R_RX_PAYLOAD);
            data |= SPI.transfer(NOP)<<0;
            data |= SPI.transfer(NOP)<<8;
            data |= SPI.transfer(NOP)<<16;
            data |= SPI.transfer(NOP)<<24;
            digitalWrite(_csnPin, 1);
            
            // Sort into recieve buffer
            switch(pipe)
            {
            case STATUS_RN_P_NO_P0:
                rx_controller = data;
                PITimer0.reset();
                break;
                
            case STATUS_RN_P_NO_P1:
				// Break data message into four chars and send to terminal, stop if a null terminator is found
                for (int i = 0; i < 4; ++i)
				{
					if ( ((data>>(8*i)) & 0xff) == '\0') break;
					receiveChar( (char)((data>>(8*i)) & 0xff) );
					Serial.println((char)((data>>(8*i)) & 0xff));
				}
                break;
                
            default:
                break;
            }
        }
        
        // Reset IRQ pin
        setRegister(STATUS, STATUS_RX_DR);
    }



    void clear()
    {
        rx_controller = 0;
    }
	
	
	
	void receiveChar(char c)
	{
		// Check if a command is currently running
		if (runningCmd == NULL) // No command is currently running
		{
			int len = strlen(inputBuffer);
			
			if (isprint(c)) // If c is a printable character
			{
				if (len < INPUT_BUFFER_MAX - 1)
				{
					// Add the new character to the input buffer and display it
					inputBuffer[len] = c;
					inputBuffer[len + 1] = '\0';
					transmit(c);
				}
			}
			else if (c == '\b' || c == 127) // Backspace or DEL
			{
				if (len > 0)
				{
					inputBuffer[len - 1] = '\0';
					write("\b \b");
				}
			}
			else if (c == '\n' || c == '\r') // Execute input command
			{
				transmit('\n');
				
				bool matchFound = false; // Initialize flag
				
				// Try to match the input string to a command
				for (int i = 0; i < NUM_COMMANDS_MAX; i++)
				{
					if (cmdList[i].stringLength && !strncmp(inputBuffer, cmdList[i].cmdString, cmdList[i].stringLength))
					{
						// Match found, call the associated command
						runningCmd = cmdList[i].fpointer(inputBuffer);
						matchFound = true;
						break;
					}
				}
				
				if (matchFound)
				{
					// If the command finishes immediately, it should return null instead of a pointer to a CmdHandler
					if (runningCmd == NULL)
					{
						// Terminate the command here, because there is no CmdHandler to terminate it later
						terminateCmd();
					}
				}
				else
				{
					// No match was found
					write("unrecognized command");
					terminateCmd();
				}
			}
			// Control characters other than \n, \b, and 127 (DEL) are ignored
		}
		else // A command is running, so pass the character to the CmdHandler
		{
			runningCmd->sendChar(c);
		}
	}
    
	
	
	void terminateCmd()
	{
		// Delete the running command's CmdHandler, if it exists
		if (runningCmd != NULL)
		{
			delete runningCmd;
			runningCmd = NULL;
		}
		
		write("\n> ");
		inputBuffer[0] = '\0'; // Clear the input buffer
	}



	void write(const char* string)
	{
		const int maxsize = 256;
		int i = 0;
		
		// Send string across radio link 1 character at a time
		while (i < maxsize)
		{
			if (string[i] == '\0') break;
			delayMicroseconds(20);
			transmit(string[i++]);
		}
	}
}
