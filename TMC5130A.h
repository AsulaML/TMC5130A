#ifndef TMC5130A_H
#define	TMC5130A_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    //  Section: Included Files
    #include "main.h"
    


    // PORTs to be defined
    
    #define TMC5130A_CS_X LATGbits.LATG7                     
    #define TMC5130A_CS_Y LATGbits.LATG8                    
    #define TMC5130A_CS_Z LATGbits.LATG9                  

    #define TMC5130A_EN_X LATGbits.LATG14
    #define TMC5130A_EN_Y LATBbits.LATB4   
    #define TMC5130A_EN_Z LATBbits.LATB7  



    

    #define CLK_DRV  LATEbits.LATE5                // same for all



    // Others :
    #define	uSTEP_COURSE_PP 268000//268000
    #define	uSTEP_COURSE_X 426667
    #define	uSTEP_COURSE_X_ALL 65000000
    #define	uSTEP_SMALL_COURSE_X 50000
    
    
    #define	uSTEP_COURSE_Z 550000 

    
    #define	uSTEP_COURSE_Z_1 400000
    #define	uSTEP_COURSE_Z_2 250000  // PDISTRI
    #define	uSTEP_COURSE_Z_ALL 650000
    #define	uSTEP_COURSE_Z_init_up   100000
    
    #define uSTEP_COURSE_Y 0x2E6B4  
    #define uSTEP_SMALL_COURSE_Y 0x4522  


    #define AMAX_EARLYRAMP 12000

    #define STEPPER_X 0
    #define STEPPER_Y 1
    #define STEPPER_Z 2

    // TRINAMIC TMC5130 Logics val Defines

    #define X_DROITE 0
    #define X_GAUCHE 1
    
    #define Z_HAUT 0
    #define Z_BAS 1

    #define NOT_USED 0
    #define POS_MODE 0
    #define V_PLUS_MODE 1
    #define V_MINUS_MODE 2
    #define HOLD_MODE 3
    
    #define TMC5130A_READ 0
    #define TMC5130A_WRITE 1 

    // TRINAMIC TMC5130 Register Address Defines

    #define TMC5130A_REG_ADDR_GCONF                 0x00 	// Global configuration flags
    #define TMC5130A_REG_ADDR_GSTA                  0x01    // Global status flag
    #define TMC5130A_REG_ADDR_IOIN                  0x04
    #define TMC5130A_REG_ADDR_X_COMPARE             0x05	// Position  comparison  register
    #define TMC5130A_REG_ADDR_IHOLD_IRUN            0x10	// Driver current control
    #define TMC5130A_REG_ADDR_TPOWER_DOWN           0x11 
    #define TMC5130A_REG_ADDR_TPWMTHRS              0x13
    #define TMC5130A_REG_ADDR_TCOOLTHRS             0x14
    #define TMC5130A_REG_ADDR_THIGH                 0x15
    #define TMC5130A_REG_ADDR_RAMPMODE              0x20	// Driving mode (Velocity, Positioning, Hold)
    #define TMC5130A_REG_ADDR_X_ACTUAL              0x21	// Actual motor position
    #define TMC5130A_REG_ADDR_VACTUAL               0x22	// Actual  motor  velocity  from  ramp  generator
    #define TMC5130A_REG_ADDR_VSTART                0x23	// Motor start velocity
    #define TMC5130A_REG_ADDR_A_1                   0x24	// First  acceleration  between  VSTART  and  V1
    #define TMC5130A_REG_ADDR_V_1                   0x25	// First  acceleration  /  deceleration  phase  target velocity
    #define TMC5130A_REG_ADDR_A_MAX                 0x26	// Second  acceleration  between  V1  and  VMAX
    #define TMC5130A_REG_ADDR_V_MAX                 0x27	// This is the target velocity in velocity mode. It can be changed any time during a motion.
    #define TMC5130A_REG_ADDR_D_MAX                 0x28	// Deceleration between VMAX and V1
    #define TMC5130A_REG_ADDR_D_1                   0x2A 	// Deceleration  between  V1  and  VSTOP
    #define TMC5130A_REG_ADDR_V_STOP                0x2B	// Motor stop velocity (unsigned)
    #define TMC5130A_REG_ADDR_TZEROWAIT             0x2C	// Defines  the  waiting  time  after  ramping  down

    #define TMC5130A_REG_ADDR_X_TARGET              0x2D	// Target position for ramp mode
    #define TMC5130A_REG_ADDR_V_DC_MIN              0x33	// Minimum velocity for deceleration 
    #define TMC5130A_REG_ADDR_SW_MODE               0x34	// Switch mode configuration
    #define TMC5130A_REG_ADDR_RAMP_STAT             0x35	// Ramp status and switch event status
    #define TMC5130A_REG_ADDR_X_LATCH               0x36	// Latches  XACTUAL  upon  a programmable switch event
    #define TMC5130A_REG_ADDR_CHOPCONF              0x6C	// Chopper and driver configuration
    #define TMC5130A_REG_ADDR_COOLCONF              0x6D	// coolStep smart current control register and stallGuard2 configuration
    #define TMC5130A_REG_ADDR_DC_CTRL               0x6E
    #define TMC5130A_REG_ADDR_DRV_STATUS            0x6F	// stallGuard2 value and driver error flags
    #define TMC5130A_REG_ADDR_PWM_CONF              0x70
    #define TMC5130A_REG_ADDR_LOST_STEPS            0x73



typedef struct {
    uint8_t id;
    void (*CS_Select)(void);
    void (*CS_Deselect)(void);
    void (*Enable)(void);
    void (*Disable)(void);
} StepperDriver;


void CS_Select_X(void);
void CS_Deselect_X(void);
void CS_Select_Y(void);
void CS_Deselect_Y(void);
void CS_Select_Z(void);
void CS_Deselect_Z(void);


uint8_t TMC5130A_Read_Write_Reg(StepperDriver* driver, uint8_t RW, uint8_t reg_adr, uint8_t *pTransmitData, uint8_t *pReceiveData);
void TMC5130A_Write_4B_Reg(StepperDriver* driver, uint8_t reg_addr, uint8_t b3, uint8_t b2, uint8_t b1, uint8_t b0);
void TMC5130A_Write_32b_Reg(StepperDriver* driver, uint8_t reg_addr, uint32_t value);

void TMC5130A_Init(StepperDriver* driver, uint8_t mode);
void TMC5130A_Config_Ramp_Mode(StepperDriver* driver, uint8_t mode);
void TMC5130A_Config_Courants(StepperDriver* driver, uint8_t  irun, uint8_t  ihold, uint8_t mode);
void TMC5130A_Config_Ramp(StepperDriver* driver, uint32_t Vtarget, uint32_t Vstart, uint32_t Vstop, uint32_t V1, uint32_t Amax, uint32_t A1, uint32_t Dmax, uint32_t D1);
void TMC5130A_Config_uStep_Pos_Direction(StepperDriver* driver, int32_t microstep_number, uint8_t rot_dir, int32_t offset);
void TMC5130A_EarlyRampTermination_InPosMode(StepperDriver* driver);

uint32_t TMC5130A_Read_RAMP_STAT(StepperDriver* driver);
int32_t TMC5130A_Read_X_ACTUAL(StepperDriver* driver);
uint32_t TMC5130A_Clear_X_ACTUAL(StepperDriver* driver);
void TMC5130A_Config_X_TARGET_Only(StepperDriver* driver, int pos_microstep_number);

bool TMC5130A_Is_Motor_Stopped(uint8_t WichStepper);
bool TMC5130A_Is_Motor_Velocity_Zero(uint8_t WichStepper);
bool TMC5130A_Is_Motor_Position_Reached(uint8_t WichStepper);

    
#ifdef	__cplusplus
}
#endif

#endif