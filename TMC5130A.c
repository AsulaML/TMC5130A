#include "TMC5130A.h"
#include "Spi2.h"


// Config HW :
// SDMODE   GND
// SPIMODE  NC 
// SWSEL    NC 



// GLOBAL MEMO WHEN EARLY RAMP TERMINATION IS USED
int32_t VMAX_MEMO;
int32_t VSTART_MEMO = 0;


void CS_Select_X(void)    { TMC5130A_CS_X = 0; }
void CS_Deselect_X(void)  { TMC5130A_CS_X = 1; }
void Enable_X(void)       { TMC5130A_EN_X = 0; }
void Disable_X(void)      { TMC5130A_EN_X = 1; }

void CS_Select_Y(void)    { TMC5130A_CS_Y = 0; }
void CS_Deselect_Y(void)  { TMC5130A_CS_Y = 1; }
void Enable_Y(void)       { TMC5130A_EN_Y = 0; }
void Disable_Y(void)      { TMC5130A_EN_Y = 1; }

void CS_Select_Z(void)    { TMC5130A_CS_Z = 0; }
void CS_Deselect_Z(void)  { TMC5130A_CS_Z = 1; }
void Enable_Z(void)       { TMC5130A_EN_Z = 0; }
void Disable_Z(void)      { TMC5130A_EN_Z = 1; }


/**
 * \fn TMC5130A_Read_Write_Reg
 * \brief Fonction de communication du TMC5130A.
 *
 * \param    StepperDriver* driver : Pointeur sur le driver
 * \param    uint8_t RW : 1 pour écrire, 0 pour lire
 * \param    uint8_t reg_adr : Adresse du registre
 * \param    uint8_t pTransmitData : Données à écrire
 * \param    uint8_t pReceiveData : Données à lire

 * \return   uint8_t : status_reg
 */
uint8_t TMC5130A_Read_Write_Reg(StepperDriver* driver, uint8_t RW, uint8_t reg_adr, uint8_t *pTransmitData, uint8_t *pReceiveData)
{
    uint8_t first_byte = (RW << 7) | (reg_adr & 0x7F);
    uint8_t status_reg = 0;

    driver->CS_Select();
    
    // first transaction containt the addr and RW bit
    SPI2_Exchange(&first_byte, &status_reg);
    
    for (uint8_t i = 0; i < 4; i++) 
    {
        SPI2_Exchange(pTransmitData + i, pReceiveData);
    }

    driver->CS_Deselect();

    
    
    if (RW == TMC5130A_READ) 
    {
        driver->CS_Select();

        // first transaction containt the addr and RW bit
        SPI2_Exchange(&first_byte, &status_reg);

        for (uint8_t i = 0; i < 4; i++) 
        {
            SPI2_Exchange(pTransmitData + i, pReceiveData + i);
        }

        driver->CS_Deselect();
    }    
    
    return(status_reg);
}


/**
 * \fn TMC5130A_Init
 * \brief Fonction de communication du TMC5130A.
 *
 * \param    StepperDriver* driver : Pointeur sur le driver
 * \param    uint8_t mode : ramp mode 3 -> Hold 2-> Velocity negative 1-> Velocity positive 0-> positionning mode (A,D,V params)

 * \return   void
 */
void TMC5130A_Init(StepperDriver* driver, uint8_t mode)
{
    uint8_t status_reg = 0;
    uint8_t DataReaded[4] = {0};
    
    // Driver EN X CONFIG
    TRISGbits.TRISG14 = 0;
    
    // Driver EN Y CONFIG
    TRISBbits.TRISB4 = 0;
    ANSELBbits.ANSB4 = 0;
    
    // Driver EN Z CONFIG
    TRISBbits.TRISB7 = 0;
    ANSELBbits.ANSB7 = 0;
		
    // CLK DRIVER TIED low for internal operation
    TRISEbits.TRISE5 = 0;
    ANSELEbits.ANSE5 = 0;

    CLK_DRV = 0;
    
    
    driver->Disable();
    TMC5130A_Config_Ramp_Mode(driver, mode);
}



/**
 * \fn TMC5130A_Config_Ramp_Mode
 * \brief Fonction de config du mode du TMC5130A.
 *
 * \param    StepperDriver* driver : Pointeur sur le driver
 * \param    uint8_t mode : ramp mode 3 -> Hold 2-> Velocity negative 1-> Velocity positive 0-> positionning mode (A,D,V params)

 * \return   void
 */
void TMC5130A_Config_Ramp_Mode(StepperDriver* driver, uint8_t mode)
{
    uint32_t data = mode & 0x03;  

    TMC5130A_Write_32b_Reg(driver, TMC5130A_REG_ADDR_RAMPMODE, data);
}

/**
 * \fn TMC5130A_Config_Ramp_Mode
 * \brief Fonction de config du mode du TMC5130A.
 *
 * \param    StepperDriver* driver : Pointeur sur le driver
 * \param    uint8_t mode : ramp mode 3 -> Hold 2-> Velocity negative 1-> Velocity positive 0-> positionning mode (A,D,V params)
 * \param    uint8_t irun : 0 � 31 
 * \param    uint8_t ihold : 0 � 31 
 * \return   void
 */
void TMC5130A_Config_Courants(StepperDriver* driver, uint8_t  irun, uint8_t  ihold, uint8_t mode)
{
    // GCONF : Push-pull (bit 13), PDN disable (bit 2)
    TMC5130A_Write_4B_Reg(driver, TMC5130A_REG_ADDR_GCONF, 0x00, 0x00, 0x20, 0x04);

    // IHOLD_IRUN : Set run and hold current (5-bit values)
    TMC5130A_Write_4B_Reg(driver, TMC5130A_REG_ADDR_IHOLD_IRUN, 0x00, 0x00, irun & 0x1F, ihold & 0x1F);

    // RAMPMODE : motion mode selection (only 2 LSB used)
    TMC5130A_Write_4B_Reg(driver, TMC5130A_REG_ADDR_RAMPMODE, 0x00, 0x00, 0x00, mode & 0x03);

    // CHOPCONF : microstepping, blank time, etc.
    TMC5130A_Write_4B_Reg(driver, TMC5130A_REG_ADDR_CHOPCONF, 0x10, 0x00, 0x00, 0xC3);

    // PWMCONF : PWM mode configuration
    TMC5130A_Write_4B_Reg(driver, TMC5130A_REG_ADDR_PWM_CONF, 0x00, 0x04, 0x00, 0x00);

    // TPWMTHRS : threshold for PWM mode (default 0)
    TMC5130A_Write_4B_Reg(driver, TMC5130A_REG_ADDR_TPWMTHRS, 0x00, 0x00, 0x00, 0x00);

    // V_DC_MIN : minimum DC voltage limit (disabled)
    TMC5130A_Write_4B_Reg(driver, TMC5130A_REG_ADDR_V_DC_MIN, 0x00, 0x00, 0x00, 0x00);

    // DC_CTRL : DC Stepper settings
    TMC5130A_Write_4B_Reg(driver, TMC5130A_REG_ADDR_DC_CTRL, 0x00, 0x00, 0x00, 0x00);

    // COOLCONF : StallGuard and smart energy control
    TMC5130A_Write_4B_Reg(driver, TMC5130A_REG_ADDR_COOLCONF, 0x01, 0x7F, 0x00, 0x00);

    // TCOOLTHRS : Threshold for switching to CoolStep (default 0)
    TMC5130A_Write_4B_Reg(driver, TMC5130A_REG_ADDR_TCOOLTHRS, 0x00, 0x00, 0x00, 0x00);
}

/**
 * \fn TMC5130A_Config_Ramp_Mode
 * \brief Fonction de config du mode du TMC5130A.
 *
 * \param    StepperDriver* driver : Pointeur sur le driver
 * \param    uint32_t Vtarget : 
 * \param    uint32_t Vstart : 
 * \param    uint32_t Vstop : vitesse en fin de rampe
 * \param    uint32_t V1 : vitesse interm�diaire s�parant les deux phases d'acc�l�ration si = 0 alors A1 D1 sont NA
 * \param    uint32_t Amax : deuxi�me phase d'acc�l�ration (2/2) ou (1/1) si V1 = 0
 * \param    uint32_t A1 : premi�re phase d'acc�l�ration (1/2) si V1 diff�rent de 0 
 * \param    uint32_t Dmax : deuxi�me phase de d�c�l�ration (2/2) ou (1/1) si V1 = 0 
 * \param    uint32_t D1 : premi�re phase de d�c�l�ration (1/2) si V1 diff�rent de 0 sinon mettre 0x10 
 * \return   void
 */
void TMC5130A_Config_Ramp(StepperDriver* driver, uint32_t Vtarget, uint32_t Vstart, uint32_t Vstop, uint32_t V1, uint32_t Amax, uint32_t A1, uint32_t Dmax, uint32_t D1)
{
 	uint8_t status_reg = 0;
    uint8_t DataToWrite[4] = {0};
    uint8_t DataReaded[4] = {0};

    TMC5130A_Write_32b_Reg(driver, TMC5130A_REG_ADDR_RAMPMODE, 0);

    TMC5130A_Write_32b_Reg(driver, TMC5130A_REG_ADDR_V_STOP, Vstop);
    TMC5130A_Write_32b_Reg(driver, TMC5130A_REG_ADDR_A_1, A1);
    TMC5130A_Write_32b_Reg(driver, TMC5130A_REG_ADDR_V_1, V1);
    TMC5130A_Write_32b_Reg(driver, TMC5130A_REG_ADDR_A_MAX, Amax);


    // Memorize for future use
    VMAX_MEMO = Vtarget;
    VSTART_MEMO = Vstart;
    
    TMC5130A_Write_32b_Reg(driver, TMC5130A_REG_ADDR_D_MAX, Dmax);
    TMC5130A_Write_32b_Reg(driver, TMC5130A_REG_ADDR_D_1, D1);
}

/**
 * \fn TMC5130A_Config_uStep_Pos_Direction
 * \brief Fonction de pilotage déplacement du TMC5130A.
 *
 * \param    StepperDriver* driver : Pointeur sur le driver
 * \param    int32_t microstep_number : number microstep 256 ustep -> 1 step
 * \param    uint8_t rot_dir : sens de rotation 0 ou 1 
 * \param    int32_t offset : ajouter un offstep de microstep 
 * \return   void
 */
void TMC5130A_Config_uStep_Pos_Direction(StepperDriver* driver, int32_t microstep_number, uint8_t rot_dir, int32_t offset)
{
    // signed microstep position
    int32_t pos = microstep_number;
		
    TMC5130A_Write_32b_Reg(driver, TMC5130A_REG_ADDR_X_TARGET, pos);
    TMC5130A_Write_32b_Reg(driver, TMC5130A_REG_ADDR_VSTART, VSTART_MEMO);
    TMC5130A_Write_32b_Reg(driver, TMC5130A_REG_ADDR_V_MAX, VMAX_MEMO);
    
    
    
    // Sort de la fonction lorsque la vitesse du moteur est non nulle (100ms max)
    int time = 100;
    while(time--) 
    {
        delay_ms(1);
				
				// 	Proceed the transfert to the good Stepper
				switch(WichStepper)
				{
					case STEPPER_X :
						if (!TMC5130A_Is_Motor_Stopped(STEPPER_X)) return;
					break;

					case STEPPER_Y :
						if (!TMC5130A_Is_Motor_Stopped(STEPPER_Y)) return;
					break;

					case STEPPER_Z :
						if (!TMC5130A_Is_Motor_Stopped(STEPPER_Z)) return;
					break;
				}
        
    };
}

/**
 * \fn TMC5130A_EarlyRampTermination_InPosMode
 * \brief Fonction pour arreter le moteur TMC5130A.
 *  For a stop in positioning mode, set VSTART=0 and VMAX=0. VSTOP is not used in this case. The
 *  driver will use AMAX and A1 (as determined by V1) for going to zero velocity.
 * \param    StepperDriver* driver : Pointeur sur le driver
 * \return   void
 */
void TMC5130A_EarlyRampTermination_InPosMode(StepperDriver* driver)
{
    uint8_t status_reg = 0;
    uint8_t DataToWrite[4] = {0};
    uint8_t DataReaded[4] = {0};
    
    status_reg = TMC5130A_Read_Write_Reg(driver, TMC5130A_WRITE, TMC5130A_REG_ADDR_VSTART, &DataToWrite[0], &DataReaded[0]);
    status_reg = TMC5130A_Read_Write_Reg(driver, TMC5130A_WRITE, TMC5130A_REG_ADDR_V_MAX, &DataToWrite[0], &DataReaded[0]);
}


/**
 * \fn TMC5130A_Is_Motor_Stopped
 * \brief Fonction pour savoir si le moteur ne bouge plus.
 * \param    StepperDriver* driver : Pointeur sur le driver
 * \return   TRUE -> v = 0 ; FALSE -> v != 0
 */
bool TMC5130A_Is_Motor_Stopped(StepperDriver* driver) 
{
    // Test flags to be sure that the velocity zero flag is set
    if (TMC5130A_Is_Motor_Velocity_Zero(driver))
    {
        return true;
    }

    return false;   
}


/**
 * \fn TMC5130A_Is_Motor_Velocity_Zero
 * \brief Fonction pour savoir si le moteur ne bouge plus.
 * \param    StepperDriver* driver : Pointeur sur le driver
 * \return   TRUE -> v = 0 ; FALSE -> v != 0
 */
bool TMC5130A_Is_Motor_Velocity_Zero(StepperDriver* driver)
{
    delay_ms(1);
    
    // Read & Reset events flags 
    uint32_t flags = TMC5130A_Read_RAMP_STAT(driver);
    
    if ((flags & TMC5130A_MASK_VEL_ZERO) != 0) return true;
    else return false;
}


/**
 * \fn TMC5130A_Is_Motor_Position_Reached
 * \brief Fonction pour savoir si la position est atteinte.
 * \param    StepperDriver* driver : Pointeur sur le driver
 * \return   TRUE -> reached; FALSE -> not reached
 */
bool TMC5130A_Is_Motor_Position_Reached(StepperDriver* driver)
{
    delay_ms(1);
    
    // Read & Reset events flags 
    uint32_t flags = TMC5130A_Read_RAMP_STAT(driver);
    
    if ((flags & TMC5130A_MASK_POS_REACHED) != 0) return true;
    else return false;
}


/**
 * \fn TMC5130A_Read_RAMP_STAT
 * \brief Fonction pour lire le registre de status.
 * \param    StepperDriver* driver : Pointeur sur le driver
 * \return   status
 */
uint32_t TMC5130A_Read_RAMP_STAT(StepperDriver* driver)
{
    uint8_t status_reg = 0;
    uint8_t DataToWrite[4] = {0};
    uint8_t DataReaded[4] = {0};
    uint32_t read_reg = 0;
    
    status_reg = TMC5130A_Read_Write_Reg(driver, TMC5130A_READ, TMC5130A_REG_ADDR_RAMP_STAT, &DataToWrite[0], &DataReaded[0]);
    
    read_reg =  BytesToUint32(&DataReade[0]]);
    
    return(read_reg);
}


/**
 * \fn TMC5130A_Read_X_ACTUAL
 * \brief Fonction pour lire la position du moteur.
 * \param    StepperDriver* driver : Pointeur sur le driver
 * \return   XACTUAL signed
 */
int32_t TMC5130A_Read_X_ACTUAL(StepperDriver* driver)
{
    uint8_t status_reg = 0;
    uint8_t DataToWrite[4] = {0};
    uint8_t DataReaded[4] = {0};
    uint32_t read_reg = 0;

    status_reg = TMC5130A_Read_Write_Reg(driver, TMC5130A_READ, TMC5130A_REG_ADDR_X_ACTUAL, &DataToWrite[0], &DataReaded[0]);

    read_reg =  BytesToUint32(&DataReade[0]]);
    
    return(read_reg);
}



/**
 * \fn TMC5130A_Clear_X_ACTUAL
 * \brief Fonction pour réaliser le zero de la position du moteur.
 * \param    StepperDriver* driver : Pointeur sur le driver
 * \return   read_reg
 */
uint32_t TMC5130A_Clear_X_ACTUAL(StepperDriver* driver)
{
    uint8_t status_reg = 0;
    uint8_t DataToWrite[4] = {0};
    uint8_t DataReaded[4] = {0};
    int32_t read_reg = 0;
    
    status_reg = TMC5130A_Read_Write_Reg(driver, TMC5130A_WRITE, TMC5130A_REG_ADDR_X_TARGET, &DataToWrite[0], &DataReaded[0]);
    status_reg = TMC5130A_Read_Write_Reg(driver, TMC5130A_WRITE, TMC5130A_REG_ADDR_X_ACTUAL, &DataToWrite[0], &DataReaded[0]);
    status_reg = TMC5130A_Read_Write_Reg(driver, TMC5130A_READ, TMC5130A_REG_ADDR_X_ACTUAL, &DataToWrite[0], &DataReaded[0]);

    read_reg =  BytesToUint32(&DataReade[0]]);
    
    if ( (read_reg <= 50) || (read_reg >= -50) ) return 0;
    else return ERROR_DRIVER_MOTOR;   
}

/**
 * \fn TMC5130A_Config_X_TARGET_Only
 * \brief Fonction pour configurer X target.
 * \param    StepperDriver* driver : Pointeur sur le driver
 * \param    int pos_microstep_number : position in number microstep 256 ustep -> 1 step
 * \return   void
 */
void TMC5130A_Config_X_TARGET_Only(StepperDriver* driver, int pos_microstep_number)
{
    TMC5130A_Write_32b_Reg(driver, TMC5130A_REG_ADDR_X_TARGET, pos_microstep_number);
}


void TMC5130A_Write_4B_Reg(StepperDriver* driver, uint8_t reg_addr, uint8_t b3, uint8_t b2, uint8_t b1, uint8_t b0) 
{
    uint8_t DataToWrite[4] = {b3, b2, b1, b0};
    uint8_t DataReaded[4];
    TMC5130A_Read_Write_Reg(driver, TMC5130A_WRITE, reg_addr, DataToWrite, DataReaded);
}

void TMC5130A_Write_32b_Reg(StepperDriver* driver, uint8_t reg_addr, uint32_t value)
{
    uint8_t DataToWrite[4] = {
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF
    };
    uint8_t DataReaded[4] = {0};
    TMC5130A_Read_Write_Reg(driver, TMC5130A_WRITE, reg_addr, DataToWrite, DataReaded);
}

uint32_t BytesToUint32(uint8_t* data)
{
    return  ((uint32_t)data[0] << 24) |
            ((uint32_t)data[1] << 16) |
            ((uint32_t)data[2] << 8)  |
            ((uint32_t)data[3]);
}