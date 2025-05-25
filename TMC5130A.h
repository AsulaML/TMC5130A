#ifndef TMC5130A_H
#define	TMC5130A_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    //  Section: Included Files
    #include "main.h"
    


    // PORTs to be defined
    
    #define CS_X LATGbits.LATG7                     
    #define CS_Y LATGbits.LATG8                    
    #define CS_Z LATGbits.LATG9                  

    #define DRIVER_X_AXIS LATGbits.LATG14
    #define DRIVER_X_AXIS_ENABLE DRIVER_X_AXIS=0
    #define DRIVER_X_AXIS_DISABLE DRIVER_X_AXIS=1
    
    #define DRIVER_Y_AXIS LATBbits.LATB4    
    #define DRIVER_Y_AXIS_ENABLE DRIVER_Y_AXIS=0
    #define DRIVER_Y_AXIS_DISABLE DRIVER_Y_AXIS=1

    #define DRIVER_Z_AXIS LATBbits.LATB7     
    #define DRIVER_Z_AXIS_ENABLE DRIVER_Z_AXIS=0
    #define DRIVER_Z_AXIS_DISABLE DRIVER_Z_AXIS=1
    

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
    
    #define READ 0
    #define WRITE 1 

    // TRINAMIC TMC5130 Register Address Defines

    #define GCONF				0x00 	//Global configuration flags
    #define GSTAT               0x01    //Global status flag
    #define IOIN                0x04
    #define X_COMPARE 			0x05	//Position  comparison  register
    #define IHOLD_IRUN			0x10	//Driver current control
    #define TPOWER_DOWN         0x11 
    #define TPWMTHRS            0x13
    #define TCOOLTHRS           0x14
    #define THIGH               0x15
    #define RAMPMODE			0x20	//Driving mode (Velocity, Positioning, Hold)
    #define XACTUAL				0x21	//Actual motor position
    #define VACTUAL 			0x22	//Actual  motor  velocity  from  ramp  generator
    #define VSTART				0x23	//Motor start velocity
    #define A_1					0x24	//First  acceleration  between  VSTART  and  V1
    #define V_1					0x25	//First  acceleration  /  deceleration  phase  target velocity
    #define AMAX				0x26	//Second  acceleration  between  V1  and  VMAX
    #define VMAX 				0x27	//This is the target velocity in velocity mode. It can be changed any time during a motion.
    #define DMAX				0x28	//Deceleration between VMAX and V1
    #define D_1					0x2A 	//Deceleration  between  V1  and  VSTOP
                                        //Attention:  Do  not  set  0  in  positioning  mode, even if V1=0!
    #define VSTOP				0x2B	//Motor stop velocity (unsigned)
                                        //Attention: Set VSTOP > VSTART!
                                        //Attention:  Do  not  set  0  in  positioning  mode, minimum 10 recommend!
    #define TZEROWAIT			0x2C	//Defines  the  waiting  time  after  ramping  down
                                        //to  zero  velocity  before  next  movement  or
                                        //direction  inversion  can  start.  Time  range  is about 0 to 2 seconds.
    #define XTARGET				0x2D	//Target position for ramp mode
    #define VDCMIN              0x33
    #define SW_MODE 			0x34	//Switch mode configuration
    #define RAMP_STAT			0x35	//Ramp status and switch event status
    #define XLATCH				0x36	//Latches  XACTUAL  upon  a programmable switch event
    #define CHOPCONF			0x6C	//Chopper and driver configuration
    #define COOLCONF			0x6D	//coolStep smart current control register and stallGuard2 configuration
    #define DCCTRL              0x6E
    #define DRV_STATUS 			0x6F	//stallGuard2 value and driver error flags
    #define PWMCONF             0x70
    #define LOST_STEPS          0x73



uint8_t WriteReg(uint8_t WichStepper, uint8_t RW, uint8_t reg_adr, uint8_t *pTransmitData, uint8_t *pReceiveData);
void Stepper_Init(uint8_t WichStepper, uint8_t mode);
void Stepper_Config_RampMode_Only(uint8_t WichStepper, uint8_t mode);
void Stepper_Config_Courants_GeneralPurpose(uint8_t WichStepper, uint8_t  irun, uint8_t  ihold, uint8_t mode);
void Stepper_Config_RAMP(uint8_t WichStepper, uint32_t Vtarget, uint32_t Vstart, uint32_t Vstop, uint32_t V1, uint32_t Amax, uint32_t A1, uint32_t Dmax, uint32_t D1);
void Stepper_Config_uStep_Pos_Direction(uint8_t WichStepper, int32_t microstep_number, uint8_t rot_dir, int32_t offset);
void EarlyRampTermination_InPosMode(uint8_t WichStepper);
bool Is_Motor_Stopped(uint8_t WichStepper);
bool Is_Motor_Velocity_Zero(uint8_t WichStepper);
bool Is_Motor_Position_Reached(uint8_t WichStepper);
uint32_t Read_RAMP_STAT(uint8_t WichStepper);
int32_t Read_XACTUAL(uint8_t WichStepper);
uint32_t Clear_XACTUAL(uint8_t WichStepper);
void Stepper_Config_XTARGET_Only(uint8_t WichStepper, int pos_microstep_number);

    
#ifdef	__cplusplus
}
#endif

#endif