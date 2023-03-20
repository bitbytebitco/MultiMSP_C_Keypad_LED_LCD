#include <msp430.h> 

int j = 0;
char packet[]= {0x03, 0x04};
volatile unsigned char col_holding, row_holding;
volatile unsigned char pressed_key;
int lock_state = 1;

void initI2C_master(){
     UCB1CTLW0 |= UCSWRST;       // SW RESET ON

     UCB1CTLW0 |= UCSSEL_3;      // SMCLK
     UCB1BRW = 10;               // prescalar to 10

     UCB1CTLW0 |= UCMODE_3;      // Put into I2C mode
     UCB1CTLW0 |= UCMST;         // set as MASTER
     UCB1CTLW0 |= UCTR;          // Put into Tx mode
     //UCB1I2CSA = 0x0068;          // Slave address = 0x68

     UCB1CTLW1 |= UCASTP_2;      // enable automatic stop bit
     //UCB1TBCNT = sizeof(packet);  // Transfer byte count
     UCB1TBCNT = 1;  // Transfer byte count

     // setup ports
     P6DIR |= (BIT6 | BIT5 | BIT4);  // set P6.6-4 as OUTPUT
     P6OUT &= ~(BIT6 | BIT5 | BIT4); // clear P6.6-4

     P4SEL1 &= ~BIT7;            // P4.7 SCL
     P4SEL0 |= BIT7;


     P4SEL1 &= ~BIT6;            // P4.6 SDA
     P4SEL0 |= BIT6;

     PM5CTL0 &= ~LOCKLPM5;       // turn on I/O
     UCB1CTLW0 &= ~UCSWRST;      // SW RESET OFF

     //UCB1IE |= UCTXIE0;          // enable I2C B0 Tx IRQ
}

void initTimerB0compare(){
    // setup TB0
    TB0CTL |= TBCLR;        // Clear TB0
    TB0CTL |= TBSSEL__ACLK; // select SMCLK
    TB0CTL |= MC__UP;       // UP mode
    //TB0CCR0 = 5384;         // set CCR0 value (period)

    //TB0CCTL0 |= CCIE;       // local IRQ enable for CCR0
    //TB0CCTL0 &= ~CCIFG;     // clear CCR0 flag
}

void columnInput(){
    P3DIR &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // Initialize pins as input
    P3REN |= (BIT0 | BIT1 | BIT2 | BIT3);  // Enable pull up/down resistor
    P3OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3); // Configure resistors as pull down

    P3DIR |= (BIT4 | BIT5 | BIT6 | BIT7);  // init pins as outputs
    P3OUT |= (BIT4 | BIT5 | BIT6 | BIT7);  // set as outputs

    P3IES &= ~(BIT0 | BIT1 | BIT2 | BIT3); // L-H edge sensitivity
    P3IE |= (BIT0 | BIT1 | BIT2 | BIT3); // enable IRQs

    P3IFG &= ~(BIT0 | BIT1 | BIT2 | BIT3); // Clear the P3 interrupt flags
}

void rowInput(){
    P3DIR &= ~(BIT4 | BIT5 | BIT6 | BIT7);  // Initialize pins as input
    P3REN |= (BIT4 | BIT5 | BIT6 | BIT7);  // Enable pull up/down resistor
    P3OUT &= ~(BIT4 | BIT5 | BIT6 | BIT7); // Configure resistors as pull down

    P3DIR |= (BIT0 | BIT1 | BIT2 | BIT3);  // init pins as outputs
    P3OUT |= (BIT0 | BIT1 | BIT2 | BIT3);  // set as outputs

    //P3IES &= ~(BIT4 | BIT5 | BIT6 | BIT7); // L-H edge sensitivity
    //P3IE |= (BIT4 | BIT5 | BIT6 | BIT7); // enable IRQs

    //P3IFG &= ~(BIT4 | BIT5 | BIT6 | BIT7); // Clear the P3 interrupt flags
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    initI2C_master();
    initTimerB0compare();

    columnInput();

    PM5CTL0 &= ~LOCKLPM5;   // turn on Digital I/O

    __enable_interrupt();

    int i;
    while(1){
        UCB1I2CSA = 0x0068;     // Slave address = 0x58
        UCB1CTLW0 |= UCTXSTT;   // generate START condition
        while((UCB1IFG & UCSTPIFG)==0);
        UCB1IFG &= ~UCSTPIFG;

        UCB1I2CSA = 0x0058;     // Slave address = 0x68
        UCB1CTLW0 |= UCTXSTT;   // generate START condition
        while((UCB1IFG & UCSTPIFG)==0);
        UCB1IFG &= ~UCSTPIFG;

        UCB1IE &= ~UCTXIE0;
        for(i=0;i<=1000;i++){}
    }

    return 0;
}



void I2Ctransmit(int slave_command) {
    packet[0] = slave_command;
    //UCB1CTLW0 |= UCTXSTT;   // generate START condition
}

void displayUnlockPattern(){
    int i;
    for(i=0;i<=20000;i++){}
    P6OUT &= ~BIT4;
    for(i=0;i<=30000;i++){}
    P6OUT &= ~BIT5;
    for(i=0;i<=30000;i++){}
    P6OUT &= ~BIT6;
    for(i=0;i<=30000;i++){}
    P6OUT |= (BIT6 | BIT5 | BIT4);
    for(i=0;i<=30000;i++){}
    P6OUT &= ~(BIT6 | BIT5 | BIT4);
    for(i=0;i<=30000;i++){}
    P6OUT |= (BIT6 | BIT5 | BIT4);
    for(i=0;i<=30000;i++){}
    P6OUT &= ~(BIT6 | BIT5 | BIT4);
}

int is_unlocked(char key){
    if(lock_state == 0){
        return 1;
    } else {
        if(lock_state == 1){
            if(key == 0x80){
                P6OUT |= BIT6;
                lock_state = 2;
            }
        } else if(lock_state == 2){
            if(key == 0x40){
                P6OUT |= BIT5;
                lock_state = 3;
            }
        } else if(lock_state == 3){
            if(key == 0x20){
                P6OUT |= BIT4;
                displayUnlockPattern();
                lock_state = 0;
            }
        }
        return 0;
    }
}

void keyPressedAction(char pressed_key) {
    if(is_unlocked(pressed_key) == 1){
        packet[0] = pressed_key;
        UCB1IE |= UCTXIE0;
    }

    columnInput();
    P3IFG &= ~(BIT0 | BIT1 | BIT2 | BIT3); // Clear the P3 interrupt flags

}

#pragma vector=PORT3_VECTOR
__interrupt void ISR_PORT3(void){
    /* enable timer for debouncing */
    TB0CCTL0 |= CCIE;       // local IRQ enable for CCR0
    TB0CCR0 = 500;         // set CCR0 value (period) // old value = 2384
    TB0CCTL0 &= ~CCIFG;     // clear CCR0 flag to begin count
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void){

    TB0CCTL0 &= ~CCIE;  // Disable TimerB0
//    P6OUT ^= BIT6;

    col_holding = P3IN;

    rowInput();

    row_holding = P3IN;
    pressed_key = col_holding + row_holding;

    keyPressedAction(pressed_key);

}

#pragma vector=EUSCI_B1_VECTOR
__interrupt void EUSCI_B1_TX_ISR(void){
    UCB1TXBUF = packet[0];
}
