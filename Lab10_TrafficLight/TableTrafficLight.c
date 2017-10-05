// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

#define NVIC_ST_CTRL_R      (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile unsigned long *)0xE000E018))
#define LIGHT (*((volatile unsigned long *)0x400050FC))
	
#define goWest   0
#define waitWest 1
#define goSouth   2
#define waitSouth 3
#define walk 4
#define notWalk1 5
#define walkOff1 6
#define notWalk2 7
#define walkOff2 8

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// ***** 3. Subroutines Section *****

void Init();
void SysTick_Init(void);
void SysTick_Wait10ms(unsigned long delay);
void SysTick_Wait(unsigned long);

typedef struct {
	unsigned long out;
	unsigned long outF;
	unsigned long delay;
	unsigned long next[9];
} State;

State FSM[9] = {
 {0x0C, 0x02, 300, {goWest, goWest, waitWest, waitWest, waitWest, waitWest, waitWest, waitWest}}, 
 {0x14, 0x02, 150, {goSouth, goSouth, goSouth, goSouth, walk, goSouth, goSouth, goSouth}},
 {0x21, 0x02, 300, {goSouth, waitSouth, goSouth, waitSouth, waitSouth, waitSouth, waitSouth, waitSouth}},
 {0x22, 0x02, 150, {goWest, goWest, goWest, goWest, walk, goWest, goWest, walk}},
 {0x24, 0x08, 300, {walk,notWalk1,notWalk1,notWalk1,walk,notWalk1,notWalk1,notWalk1}},
 {0x24, 0x02, 150, {notWalk1,walkOff1,walkOff1,walkOff1,walkOff1,walkOff1,walkOff1,walkOff1}},
 {0x24, 0x00, 150, {walkOff1,notWalk2,notWalk2,notWalk2,notWalk2,notWalk2,notWalk2,notWalk2}},
 {0x24, 0x02, 150, {notWalk2,walkOff2,walkOff2,walkOff2,walkOff2,walkOff2,walkOff2,walkOff2}},
 {0x24, 0x00, 150, {walkOff2,goWest,goSouth,goWest,walk,goWest,goSouth,goWest}}
};



int main(void){ 
	unsigned char state = goWest;  // index to the current state 
  unsigned long input;
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	Init();
	SysTick_Init();
  EnableInterrupts();
	
  while(1){
		GPIO_PORTB_DATA_R = FSM[state].out;
		GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & (~0x0A)) | FSM[state].outF;
		SysTick_Wait10ms(FSM[state].delay);
		input = GPIO_PORTE_DATA_R;
		state = FSM[state].next[input];
  }
}




void Init() {
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x32;      // 1) B E F
  delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
  GPIO_PORTE_AMSEL_R &= ~0x07; // 3) disable analog function on PE2-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // 4) enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x07;   // 5) inputs on PE2-0
  GPIO_PORTE_AFSEL_R &= ~0x07; // 6) regular function on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;    // 7) enable digital on PE2-0
  GPIO_PORTB_AMSEL_R &= ~0x3F; // 3) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; // 4) enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    // 5) outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; // 6) regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    // 7) enable digital on PB5-0
	
	//port F init PF1 && PF3
	
	GPIO_PORTF_AMSEL_R &= ~0x0A;
	GPIO_PORTF_PCTL_R &= ~0x000000FF;
	GPIO_PORTF_DIR_R |= 0x0A;
	GPIO_PORTF_AFSEL_R &= ~0x0A;
	GPIO_PORTF_DEN_R |= 0x0A;
};

void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}


// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// 800000*12.5ns equals 10ms
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(80000);  // wait 10ms
  }
}