#include "msp.h"

uint8_t bumpVal;
uint8_t launchIn;
/*************************************
 *  Launchpad init
 ************************************/
void LaunchPad_Init(void){
  P1->SEL0 &= ~0x13;
  P1->SEL1 &= ~0x13;    // 1) configure P1.4 and P1.1 as GPIO
  P1->DIR &= ~0x12;     // 2) make P1.4 and P1.1 in
  P1->DIR |= 0x01;      //    make P1.0 out
  P1->REN |= 0x12;      // 3) enable pull resistors on P1.4 and P1.1
  P1->OUT |= 0x12;      //    P1.4 and P1.1 are pull-up
  P2->SEL0 &= ~0x07;
  P2->SEL1 &= ~0x07;    // 1) configure P2.2-P2.0 as GPIO
  P2->DIR |= 0x07;      // 2) make P2.2-P2.0 out
  P2->DS |= 0x07;       // 3) activate increased drive strength
  P2->OUT &= ~0x07;     //    all LEDs off
}

void Motor_InitSimple(void){
// Initializes the 6 GPIO lines and puts driver to sleep
// Returns right away
// initialize P1.6 and P1.7 and make them outputs

    P1->SEL0 &= ~0xC0; // Set P1.6, and P1.7 as GPIO
    P1->SEL1 &= ~0xC0; // DIR
    //P1->DIR &= ~0x12;  // Set P1.1 and P1.4 as input
    P1->DIR |= 0xC0;   // Set P1.6 and P1.7 as output
    P1->REN |= 0xC0;   // Set resistor

    P2->SEL0 &= ~0xC0; // Set P2.6 and P2.7 as GPIO
    P2->SEL1 &= ~0xC0; // PWM
    P2->DIR |= 0xC0;   // Set P2.6 and P2.7 as output
    P2->REN |= 0xC0;  // Set resistor

    P3->SEL0 &= ~0xC0; // Set P3.6 and P3.7 as GPIO
    P3->SEL1 &= ~0xC0; // SLP
    P3->DIR |= 0xC0;   // Set P3.6 and P3.7 as output
    P3->OUT &= ~0xC0;  // Put drivers to sleep
    P3->REN |= 0xC0;  // Set resistor

}

void Motor_StopSimple(void){
// Stops both motors, puts driver to sleep
// Returns right away
  P1->OUT &= ~0xC0;
  P2->OUT &= ~0xC0;   // off
  P3->OUT &= ~0xC0;   // low current sleep mode
}
void Motor_ForwardSimple(uint16_t duty, uint32_t time){
// Drives both motors forward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Stop the motors and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit
    P1->OUT &= ~0xC0; //PH(0 forward)
    P3->OUT |= 0xC0;  //SLP
    uint16_t duty1=10000-duty;
    do{
        if(time <= duty){
            uint32_t i;
            for(i = 0; i < time; i++){
                P2->OUT|=0xC0;
                SysTick_Wait10ms(1);
                if(Bump_Read() != 0xFF){
                    P2->OUT &= ~0xC0;
                    Motor_StopSimple();
                    return;
                }
            }
            P2->OUT &= ~0xC0;
            time = 0;
        }
        else{
            time  -= 10000;
            P2->OUT |= 0xC0;//EN
            SysTick_Wait10ms(duty);
            if(Bump_Read() != 0xFF){
                P2->OUT &= ~0xC0;
                Motor_StopSimple();
                return;
            }
            P2->OUT &= ~0xC0;
            SysTick_Wait10ms(duty1);
            if(Bump_Read() != 0xFF){
                P2->OUT &= ~0xC0;
                Motor_StopSimple();
                return;
            }
        }
    }while( (time > 0) );
    Motor_StopSimple();
    SysTick_Wait10ms(time);



}
void Motor_BackwardSimple(uint16_t duty, uint32_t time){
// Drives both motors backward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Runs even if any bumper switch is active
// Returns after time*10ms
    P1->OUT |= 0xC0; //PH(1 backward)
    P3->OUT |= 0xC0;  //SLP
    uint16_t duty1=10000-duty;
    do{
        if(time <= duty){
            uint32_t i;
            for(i = 0; i < time; i++){
                P2->OUT|=0xC0;
                SysTick_Wait10ms(1);
                if(Bump_Read() != 0xFF){
                    P2->OUT &= ~0xC0;
                    Motor_StopSimple();
                    Clock_Delay1ms(500);
                    return;
                }
            }
            P2->OUT &= ~0xC0;
            time = 0;
        }
        else{
            time  -= 10000;
            P2->OUT |= 0xC0;//EN
            SysTick_Wait10ms(duty);
            if(Bump_Read() != 0xFF){
                P2->OUT &= ~0xC0;
                Motor_StopSimple();
                return;
            }
            P2->OUT &= ~0xC0;
            SysTick_Wait10ms(duty1);
            if(Bump_Read() != 0xFF){
                P2->OUT &= ~0xC0;
                Motor_StopSimple();
                return;
            }
        }
    }while( (time > 0)  );
    Motor_StopSimple();
    SysTick_Wait10ms(time);

}
void Motor_LeftSimple(uint16_t duty, uint32_t time){
// Drives just the left motor forward at duty (100 to 9900)
// Right motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit
    P1->OUT &= ~0x80; //PH(0 forward)
    P3->OUT |= 0x80;  //wake left, SLP right
    uint16_t duty1=10000-duty;
    do{
        if(time <= duty){
            uint32_t i;
            for(i = 0; i < time; i++){
                P2->OUT|=0x80;
                SysTick_Wait10ms(1);
                if(Bump_Read() != 0xFF){
                    P2->OUT &= ~0x80;
                    Motor_StopSimple();
                    Clock_Delay1ms(500);
                    return;
                }
            }
            P2->OUT &= ~0x80;
            time = 0;
        }
        else{
            time  -= 10000;
            P2->OUT |= 0x80;//EN
            SysTick_Wait10ms(duty);
            if(Bump_Read() != 0xFF){
                P2->OUT &= ~0x80;
                Motor_StopSimple();
                return;
            }
            P2->OUT &= ~0x80;
            SysTick_Wait10ms(duty1);
            if(Bump_Read() != 0xFF){
                P2->OUT &= ~0x80;
                Motor_StopSimple();
                return;
            }
        }
    }while( (time > 0) );
    Motor_StopSimple();
    SysTick_Wait10ms(time);


}
void Motor_RightSimple(uint16_t duty, uint32_t time){
// Drives just the right motor forward at duty (100 to 9900)
// Left motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit
    P1->OUT &= ~0x40; //PH(0 forward)
    P3->OUT |= 0x40;  //wake right, SLP left
    uint16_t duty1=10000-duty;
    do{
        if(time <= duty){
            uint32_t i;
            for(i = 0; i < time; i++){
                P2->OUT|=0x40;
                SysTick_Wait10ms(1);
                if(Bump_Read() != 0xFF){
                    P2->OUT &= ~0x40;
                    Motor_StopSimple();
                    Clock_Delay1ms(500);
                    return;
                }
            }
            P2->OUT &= ~0x40;
            time = 0;
        }
        else{
            time  -= 10000;
            P2->OUT |= 0x40;//EN
            SysTick_Wait10ms(duty);
            if(Bump_Read() != 0xFF){
                P2->OUT &= ~0x40;
                Motor_StopSimple();
                return;
            }
            P2->OUT &= ~0x40;
            SysTick_Wait10ms(duty1);
            if(Bump_Read() != 0xFF){
                P2->OUT &= ~0x40;
                Motor_StopSimple();
                return;
            }
        }
    }while( (time > 0) );
    Motor_StopSimple();
    SysTick_Wait10ms(time);
}



//------------LaunchPad_Input------------
// Input from Switches
// Input: none
// Output: 0x00 none
//         0x01 Button1
//         0x02 Button2
//         0x03 both Button1 and Button2
uint8_t LaunchPad_Input(void){
  //return ((((~(P1->IN))&0x10)>>3)|(((~(P1->IN))&0x02)>>1));   // read P1.4,P1.1
  Clock_Delay1ms(50);
  return (P1->IN | 0xED);
}

//------------LaunchPad_Output------------
// Output to LaunchPad LEDs
// Input: 0 off, bit0=red,bit1=green,bit2=blue
// Output: none
void LaunchPad_Output(uint8_t data){  // write three outputs bits of P2
  P2->OUT = (P2->OUT&0xF8)|data;
}


// Driver test
void Pause(void){
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}



//--------------------------
// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
void Bump_Init(void){
    P4->SEL0 &= ~0xED;  // Configure P4.0, P4.2, P4.3, P4.5, P4.6, and P4.7 as GPIO
    P4->SEL1 &= ~0xED;
    P4->DIR &= ~0xED;   // Input
    P4->REN |= 0xED;    // Internal resistor

}
// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 7 Bump5
// bit 6 Bump4
// bit 5 Bump3
// bit 3 Bump2
// bit 2 Bump1
// bit 0 Bump0
uint8_t Bump_Read(void){
    uint8_t value = P4->IN;
    return value|0x12;
}

/***************************
 * main.c
 ***************************/


void main(void)
{
    Clock_Init48MHz();
     LaunchPad_Init();   // built-in switches and LEDs
     Bump_Init();        // bump switches
     Motor_InitSimple(); // initialization
     SysTick_Init();
     uint8_t isForward = 0;
     while(1){
         if(LaunchPad_Input()==0xED && isForward == 0){
             Motor_ForwardSimple(5000,350);
             P2->OUT |= 0x01;
             isForward = 1;
         }
         else if (LaunchPad_Input()==0xEF){
             Motor_LeftSimple(3000,200);
             P2->OUT |= 0x03; //right switch, left motor, Turn right
         }
         else if(LaunchPad_Input()==0xFD){
             Motor_RightSimple(3000,200);// left switch, right motor, turn left
             P2->OUT &= ~0x07;
         }
         else if(LaunchPad_Input()==0xED && isForward == 1){
             Motor_BackwardSimple(3000,200);
             P2->OUT |= 0x02;
             isForward = 0;

         }




     }



// Lab 5 part 1  - check that bump_int and bump_read work
/*         while(1){
             bumpVal=Bump_Read();
         }
*/
// Lab 5 part 2 - drive motors . Comment part 1 out and uncomment this part
        while(1){
            launchIn = LaunchPad_Input();
            /*Pause(); // start on SW1 or SW2
            LaunchPad_Output(0x02);
            Motor_ForwardSimple(5000,350);  // 3.5 seconds and stop
            LaunchPad_Output(0x00);
            Motor_StopSimple(); Clock_Delay1ms(500);

            LaunchPad_Output(0x01);
            Motor_BackwardSimple(3000,200); // reverse 2 sec
            LaunchPad_Output(0x03);
            Motor_LeftSimple(3000,200);     // right turn 2 sec
            if(Bump_Read()){
                LaunchPad_Output(0x01);
                Motor_BackwardSimple(3000,100);// reverse 1 sec
                LaunchPad_Output(0x03);
                Motor_LeftSimple(3000,200);   // right turn 2 sec
                }*/
        }

}

/****************************************
 * CLOCK Init
 ***************************************/
uint32_t ClockFrequency = 3000000; // cycles/second

int32_t Prewait = 0;                   // loops between BSP_Clock_InitFastest() called and PCM idle (expect 0)
uint32_t CPMwait = 0;                   // loops between Power Active Mode Request and Current Power Mode matching requested mode (expect small)
uint32_t Postwait = 0;                  // loops between Current Power Mode matching requested mode and PCM module idle (expect about 0)
uint32_t IFlags = 0;                    // non-zero if transition is invalid
uint32_t Crystalstable = 0;             // loops before the crystal stabilizes (expect small)

void Clock_Init32KHz(void){
  // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
  while(PCM->CTL1&0x00000100){
//  while(PCMCTL1&0x00000100){
    Prewait = Prewait + 1;
    if(Prewait >= 100000){
      return;                           // time out error
    }
  }
  // request power active mode LDO VCORE1 to support the 48 MHz frequency
  PCM->CTL0 = (PCM->CTL0&~0xFFFF000F) |     // clear PCMKEY bit field and AMR bit field
//  PCMCTL0 = (PCMCTL0&~0xFFFF000F) |     // clear PCMKEY bit field and AMR bit field
            0x695A0000 |                // write the proper PCM key to unlock write access
            0x00000001;                 // request power active mode LDO VCORE1
  // check if the transition is invalid (see Figure 7-3 on p344 of datasheet)
  if(PCM->IFG&0x00000004){
    IFlags = PCM->IFG;                    // bit 2 set on active mode transition invalid; bits 1-0 are for LPM-related errors; bit 6 is for DC-DC-related error
    PCM->CLRIFG = 0x00000004;             // clear the transition invalid flag
    // to do: look at CPM bit field in PCMCTL0, figure out what mode you're in, and step through the chart to transition to the mode you want
    // or be lazy and do nothing; this should work out of reset at least, but it WILL NOT work if Clock_Int32kHz() or Clock_InitLowPower() has been called
    return;
  }
  // wait for the CPM (Current Power Mode) bit field to reflect a change to active mode LDO VCORE1
  while((PCM->CTL0&0x00003F00) != 0x00000100){
    CPMwait = CPMwait + 1;
    if(CPMwait >= 500000){
      return;                           // time out error
    }
  }
  // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
  while(PCM->CTL1&0x00000100){
    Postwait = Postwait + 1;
    if(Postwait >= 100000){
      return;                           // time out error
    }
  }
  // initialize PJ.3 and PJ.2 and make them HFXT (PJ.3 built-in 48 MHz crystal out; PJ.2 built-in 48 MHz crystal in)
  PJ->SEL0 |= 0x03;        // 0000 0011
  PJ->SEL1 &= ~0x03;   // 1111 1100  configure built-in 32 kHz crystal for LFXT

  CS->KEY = 0x695A;            // unlock CS module for register access
  CS->CTL2 = (CS->CTL2 &~0x00000003)|0x00000003|0x00000100;
  CS->CTL2 &= ~0x00000200;     //disable low-frequency crystal bypass

  // wait for LXFT clock to stabilize
  while(CS->IFG&0x00000001){
     CS->CLRIFG = 0x00000001;       // clear the LFXT interrupt flag
     Crystalstable=Crystalstable+1;
    if(Crystalstable > 100000)
      return ;            // time out error
  }
  CS->CTL1 = 0x10000200;       // SMCLK/2 HSMCLK MCLK from LXFT
  CS->KEY = 0;         // lock CS module                         // lock CS module from unintended access
  ClockFrequency = 48000000;
//  SubsystemFrequency = 12000000;
}


void Clock_Init48MHz(void){
  // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
  while(PCM->CTL1&0x00000100){
//  while(PCMCTL1&0x00000100){
    Prewait = Prewait + 1;
    if(Prewait >= 100000){
      return;                           // time out error
    }
  }
  // request power active mode LDO VCORE1 to support the 48 MHz frequency
  PCM->CTL0 = (PCM->CTL0&~0xFFFF000F) |     // clear PCMKEY bit field and AMR bit field
//  PCMCTL0 = (PCMCTL0&~0xFFFF000F) |     // clear PCMKEY bit field and AMR bit field
            0x695A0000 |                // write the proper PCM key to unlock write access
            0x00000001;                 // request power active mode LDO VCORE1
  // check if the transition is invalid (see Figure 7-3 on p344 of datasheet)
  if(PCM->IFG&0x00000004){
    IFlags = PCM->IFG;                    // bit 2 set on active mode transition invalid; bits 1-0 are for LPM-related errors; bit 6 is for DC-DC-related error
    PCM->CLRIFG = 0x00000004;             // clear the transition invalid flag
    // to do: look at CPM bit field in PCMCTL0, figure out what mode you're in, and step through the chart to transition to the mode you want
    // or be lazy and do nothing; this should work out of reset at least, but it WILL NOT work if Clock_Int32kHz() or Clock_InitLowPower() has been called
    return;
  }
  // wait for the CPM (Current Power Mode) bit field to reflect a change to active mode LDO VCORE1
  while((PCM->CTL0&0x00003F00) != 0x00000100){
    CPMwait = CPMwait + 1;
    if(CPMwait >= 500000){
      return;                           // time out error
    }
  }
  // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
  while(PCM->CTL1&0x00000100){
    Postwait = Postwait + 1;
    if(Postwait >= 100000){
      return;                           // time out error
    }
  }
  // initialize PJ.3 and PJ.2 and make them HFXT (PJ.3 built-in 48 MHz crystal out; PJ.2 built-in 48 MHz crystal in)
  PJ->SEL0 |= 0x0C;
  PJ->SEL1 &= ~0x0C;                    // configure built-in 48 MHz crystal for HFXT operation
  CS->KEY = 0x695A;                     // unlock CS module for register access
  CS->CTL2 = (CS->CTL2&~0x00700000) |   // clear HFXTFREQ bit field
           0x00600000 |                 // configure for 48 MHz external crystal
           0x00010000 |                 // HFXT oscillator drive selection for crystals >4 MHz
           0x01000000;                  // enable HFXT
  CS->CTL2 &= ~0x02000000;              // disable high-frequency crystal bypass
  // wait for the HFXT clock to stabilize
  while(CS->IFG&0x00000002){
    CS->CLRIFG = 0x00000002;              // clear the HFXT oscillator interrupt flag
    Crystalstable = Crystalstable + 1;
    if(Crystalstable > 100000){
      return;                           // time out error
    }
  }
  // configure for 2 wait states (minimum for 48 MHz operation) for flash Bank 0
  FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL&~0x0000F000)|FLCTL_BANK0_RDCTL_WAIT_2;
  // configure for 2 wait states (minimum for 48 MHz operation) for flash Bank 1
  FLCTL->BANK1_RDCTL = (FLCTL->BANK1_RDCTL&~0x0000F000)|FLCTL_BANK1_RDCTL_WAIT_2;
  CS->CTL1 = 0x20000000 |               // configure for SMCLK divider /4
           0x00100000 |                 // configure for HSMCLK divider /2
           0x00000200 |                 // configure for ACLK sourced from REFOCLK
           0x00000050 |                 // configure for SMCLK and HSMCLK sourced from HFXTCLK
           0x00000005;                  // configure for MCLK sourced from HFXTCLK
  CS->KEY = 0;                          // lock CS module from unintended access
  ClockFrequency = 48000000;
//  SubsystemFrequency = 12000000;
}

// delay function
// which delays about 6*ulCount cycles
// ulCount=8000 => 1ms = (8000 loops)*(6 cycles/loop)*(20.83 ns/cycle)
  //Code Composer Studio Code
void delay(unsigned long ulCount){
  __asm (  "pdloop:  subs    r0, #1\n"
      "    bne    pdloop\n");
}


// ------------Clock_Delay1ms------------
// Simple delay function which delays about n milliseconds.
// Inputs: n, number of msec to wait
// Outputs: none
void Clock_Delay1ms(uint32_t n){
  while(n){
    delay(ClockFrequency/9162);   // 1 msec, tuned at 48 MHz
    n--;
  }
}


/****************************************
 *  SysTick Timer Init
 ***************************************/
void SysTick_Init(void){
  SysTick->LOAD = 0x00FFFFFF;           // maximum reload value
  SysTick->CTRL = 0x00000005;           // enable SysTick with no interrupts
}

void SysTick_Wait(uint32_t delay){
  SysTick->LOAD = (delay - 1);// count down to zero
  SysTick->VAL = 0;          // any write to CVR clears it and COUNTFLAG in CSR
  while(( SysTick->CTRL&0x00010000) == 0){};

}
// Time delay using busy wait.
// assumes 48 MHz bus clock
void SysTick_Wait10ms(uint32_t delay){
  uint32_t i;
  for(i=0; i<delay; i++){
    SysTick_Wait(480000);  // wait 10ms (assumes 48 MHz clock)
  }
}
