#include <msp430.h> 

int Rx_Command;
volatile int bin_count;

void initI2C_slave(){
    UCB0CTLW0 |= UCSWRST;       // SW RESET enabled
    UCB0CTLW0 |= UCMODE_3 | UCSYNC;      // Put into I2C mode
    UCB0I2COA0 = 0x0068 | UCOAEN; // Set slave address + enable

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

void delay(){
    int i,j,k;
    for(i=0;i<20000; i=i+1){
        for(j=0;j<20000; j=j+1){
            for(k=0;k<20000; k=k+1){}
        }
    }
}

void IncrementBinaryCounter(int bin_count){
    int k,n;
    n = 1;
    ResetLED();
    for(k=0;k<8;k++){
        if((bin_count & n)>0){
            setLEDn(k);
        }
        n = n << 1;
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

void disableTimerB0Compare(){
    TB0CCTL0 &= ~CCIE;       // disable IRQ enable for CCR0
}

void init(){
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    // Setup LED outputs
    P1DIR |= (BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7);
    P1OUT &= ~(BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7);

    P2DIR |= (BIT6 | BIT7);
    P2OUT &= ~(BIT6 | BIT7);

    initI2C_slave();
    initTimerB0compare();

    __enable_interrupt();
}

int main(void)
{
    init();

    int i;
    bin_count=0;
    while(1){
//        PressA();
//        delay();
//        ResetLED();
//        delay();

    }

    return 0;
}

//
#pragma vector=EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_TX_ISR(void){
    Rx_Command = UCB0RXBUF;    // Retrieve byte from buffer
    ResetLED();
    disableTimerB0Compare();

    if(Rx_Command == 0x03){
        PressA();
    } else if(Rx_Command == 0x04){
        TB0CCTL0 |= CCIE; // enable binary counting interrupt routine
    }
    UCB0IFG &= ~UCTXIFG0;   // clear flag
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void){
    IncrementBinaryCounter(bin_count++);
    if(bin_count==255){
        bin_count = 0;
    }
    TB0CCTL0 &= ~CCIFG;
}

