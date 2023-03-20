#include <msp430.h> 

int Rx_Command;
volatile int seq_count;
volatile int rotating_count;
volatile int bin_count;
volatile int timer_action_select;

void initI2C_slave(){
    UCB0CTLW0 |= UCSWRST;       // SW RESET enabled
    UCB0CTLW0 |= UCMODE_3 | UCSYNC;      // Put into I2C mode
    UCB0I2COA0 = 0x0058 | UCOAEN; // Set slave address + enable

    // setup ports
    P1SEL1 &= ~BIT3;            // P1.3 SCL
    P1SEL0 |= BIT3;

    P1SEL1 &= ~BIT2;            // P1.2 SDA
    P1SEL0 |= BIT2;

    PM5CTL0 &= ~LOCKLPM5;       // turn on I/O
    UCB0CTLW0 &= ~UCSWRST;      // SW RESET OFF

    UCB0IE |= UCTXIE0 | UCRXIE0 | UCSTPIE;          // enable I2C B0 Tx/Rx/Stop IRQs

    PM5CTL0 &= ~LOCKLPM5;       // turn on I/O
    UCB0CTLW0 &= ~UCSWRST;      // SW RESET OFF
}

void setLEDn(int n) {
    switch(n){
        case 7:
            P1OUT |= BIT7;
            break;
        case 6:
            P1OUT |= BIT6;
            break;
        case 3:
            P1OUT |= BIT5;
            break;
        case 2:
            P1OUT |= BIT4;
            break;
        case 4:
            P2OUT |= BIT7;
            break;
        case 5:
            P2OUT |= BIT6;
            break;
        case 0:
            P1OUT |= BIT1;
            break;
        case 1:
            P1OUT |= BIT0;
            break;
    }
}

void ResetLED() {
    P1OUT &= ~(BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7);
    P2OUT &= ~(BIT6 | BIT7);
}

void PressA() {
    setLEDn(0);
    setLEDn(2);
    setLEDn(4);
    setLEDn(6);
}

void rotatingCounter(int count) {
    ResetLED();
    setLEDn(count);
}

void incrementBinaryCounter(int count) {
    int k,n;
    n = 1;
    ResetLED();
    for(k=0;k<8;k++){
        if((count & n)>0){
            setLEDn(k);
        }
        n = n << 1;
    }
}

void sequenceCounter(int count) {
    ResetLED();
    switch(count){
        case 1:
            setLEDn(3);
            setLEDn(4);
            break;
        case 2:
            setLEDn(2);
            setLEDn(5);
            break;
        case 3:
            setLEDn(1);
            setLEDn(6);
            break;
        case 4:
            setLEDn(0);
            setLEDn(7);
            break;
        case 5:
            setLEDn(1);
            setLEDn(6);
            break;
        case 6:
            setLEDn(2);
            setLEDn(5);
            break;
        case 7:
            setLEDn(3);
            setLEDn(4);
            break;
    }
}

void initTimerB0compare(){
    // setup TB0
    TB0CTL |= TBCLR;        // Clear TB0
    TB0CTL |= TBSSEL__ACLK; // select SMCLK
    TB0CTL |= MC__UP;       // UP mode

    TB0CCR0 = 16384;         // set CCR0 value (period)
    TB0CCTL0 |= CCIE;       // local IRQ enable for CCR0
    TB0CCTL0 &= ~CCIFG;     // clear CCR0 flag
}

void init(){
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    // Setup LED outputs
    P1DIR |= (BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7);
    P1OUT &= ~(BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7);

    P2DIR |= (BIT6 | BIT7);
    P2OUT &= ~(BIT6 | BIT7);

    P2DIR |= BIT0; // P2.0 LED output
    P2OUT &= ~BIT0; // P2.0 off

    bin_count = 0;
    timer_action_select= 0 ;

    initI2C_slave();
    initTimerB0compare();

    __enable_interrupt();
}

int main(void)
{
    init();

    while(1){}

    return 0;
}

void executeCommand(int command){
    if(Rx_Command == 0x17){
        P2OUT &= ~BIT0; // LED alert off
        TB0CCTL0 |= CCIFG;
        timer_action_select = 0;
        bin_count = 0;
        rotating_count = 0;
        seq_count = 0;
        ResetLED();
        TB0CCTL0 &= ~CCIFG;
    } else {
        if(Rx_Command == 0x80){
            P2OUT |= BIT0; // LED alert on
            timer_action_select = 0;
            ResetLED();
            PressA();
        } else if(Rx_Command == 0x40){
            P2OUT |= BIT0; // LED alert on
            timer_action_select = 1; // select binary counter
        } else if(Rx_Command == 0x20){
            P2OUT |= BIT0; // LED alert on
            timer_action_select = 2; // select rotating counter
        } else if(Rx_Command == 0x10){
            P2OUT |= BIT0; // LED alert on
            timer_action_select = 3; // select alternating counter
        }
    }
}

#pragma vector=EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_TX_ISR(void){
    Rx_Command = UCB0RXBUF;    // Retrieve byte from buffer
    executeCommand(Rx_Command);
    UCB0IFG &= ~UCTXIFG0;   // clear flag to allow I2C interrupt
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void){
    if(timer_action_select == 1){
        incrementBinaryCounter(bin_count);
        bin_count++;
        if(bin_count>=255){
            bin_count = 0;
        }
    } else if(timer_action_select == 2) {
        rotatingCounter(rotating_count);
        rotating_count++;
        if(rotating_count == 8){
            rotating_count = 0;
        }
    } else if(timer_action_select == 3) {
        sequenceCounter(seq_count);
        seq_count++;
        if(seq_count == 8) {
            seq_count = 0;
        }
    }

    TB0CCTL0 &= ~CCIFG;
}
