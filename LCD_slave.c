#include <msp430.h> 

int Rx_Command;

volatile int timer_action_select;
volatile int ms_thresh, ms_count, ms_flag;

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

void initTimerB0compare(){
    // setup TB0
    TB0CTL |= TBCLR;        // Clear TB0
    TB0CTL |= TBSSEL__SMCLK; // select SMCLK
    TB0CTL |= MC__UP;       // UP mode

    TB0CCR0 = 1045;         // set CCR0 value (period) - (1045 for 1ms)
    //TB0CCTL0 |= CCIE;       // local IRQ enable for CCR0
    TB0CCTL0 &= ~CCIFG;     // clear CCR0 flag
}

void init(){
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    P1DIR |= (BIT4 | BIT5 | BIT6 | BIT7);  // set P1.4 Output for DB4-DB7 (LCD)
    P1OUT &= ~(BIT4 | BIT5 | BIT6 | BIT7); // clear P1.4-7

    P2DIR |= BIT7;  // set P2.7 output for RS (LCD) - (RS = 0 instruction, RS = 1 data)
    P2OUT &= ~BIT7; // clear P2.7

    P2DIR |= BIT6;  // set P2.6 output for E (LCD), data starts on falling edge
    P2OUT &= ~BIT6; // clear P2.6

    P1DIR |= BIT1;  // P1.1 LED output
    P1OUT &= ~BIT1; // clear P1.1

    initTimerB0compare();
    initI2C_slave();

    __enable_interrupt();
}

void delay1000(){
    int i;
    for(i=0;i<=1000;i++){}
}

void configure(){
    P2OUT &= ~BIT7; // clear RS
    P1OUT |= (BIT4 | BIT5); // set P1.4-5
    P1OUT &= ~(BIT6 | BIT7); // P1.6-7
}

void configure2(){
    P2OUT &= ~BIT7; // clear RS
    P1OUT |= BIT5; // set P1.5
    P1OUT &= ~(BIT4 | BIT6 | BIT7);// clear P1.4, P1.6-7
}

void latch(){
    P2OUT |= BIT6; // set E bit (LCD)
    delay1000();
    P2OUT &= ~BIT6; // reset E bit (LCD)
}

void delay_ms(int ms){
    P1OUT |= BIT1; // LED on
    ms_flag = 0;
    ms_count = 0;
    ms_thresh = ms;
    TB0CCTL0 |= CCIE;       // local IRQ enable for CCR0
    while(ms_flag != 1){}
    TB0CCTL0 &= ~CCIE;       // disable CCR0
    //flag_30_ms = 0;
    P1OUT &= ~BIT1; // LED off
}


void LCDsetup() {

    configure();
    latch();
    delay_ms(10);

    configure();
    latch();
    delay_ms(10);

    configure();
    latch();
    delay_ms(10);

    configure2(); // funcset interface 4 bit
    latch();
    delay_ms(10);


    // funcset interface 4 bit
    P2OUT &= ~BIT7; // clear RS
    P1OUT &= ~(BIT7 | BIT6 | BIT4);
    P1OUT |= (BIT5);
    latch();
    delay_ms(10);

    // spec display lines and fonts
    P2OUT &= ~BIT7; // clear RS
    P1OUT |= (BIT6);
    P1OUT &= ~(BIT7 | BIT5 | BIT4);
    latch();
    delay_ms(10);

    // display on/off
    P2OUT &= ~BIT7; // clear RS
    P1OUT &= ~(BIT7 | BIT6 | BIT5 | BIT4);
    latch();
    delay_ms(10);
    P2OUT &= ~BIT7; // clear RS
    P1OUT |= (BIT7 | BIT6 );
    P1OUT &= ~(BIT5 | BIT4);

    latch();
    delay_ms(30);

    //
    P2OUT &= ~BIT7; // clear RS
    P1OUT &= ~(BIT7 | BIT6 | BIT5 | BIT4);
    latch();
    delay_ms(10);
    P2OUT &= ~BIT7; // clear RS
    P1OUT |= (BIT6 | BIT5);
    P1OUT &= ~(BIT7 | BIT4);
    latch();
    delay_ms(10);

}

void clear_display(){
    P2OUT &= ~BIT7; // set RS
    P1OUT &= ~(BIT7 | BIT6 | BIT5 | BIT4);
    latch();
    delay_ms(2);
    P1OUT &= ~(BIT7 | BIT6 | BIT5 );
    P1OUT |= BIT4;
    latch();
    delay_ms(10);
}

void setNibbleBit(int n) {
    switch(n){
        case 3:
            P1OUT |= BIT7;
            break;
        case 2:
            P1OUT |= BIT6;
            break;
        case 1:
            P1OUT |= BIT5;
            break;
        case 0:
            P1OUT |= BIT4;
            break;
    }
}

void clearNibbleBit(int n) {
    switch(n){
        case 3:
            P1OUT &= ~BIT7;
            break;
        case 2:
            P1OUT &= ~BIT6;
            break;
        case 1:
            P1OUT &= ~BIT5;
            break;
        case 0:
            P1OUT &= ~BIT4;
            break;
    }
}

void sendNibble(unsigned short byte) {
    int k,n;
    volatile int test;
    n = 1;

    for(k=0;k<=3;k++){
        test = byte & n;
        if(test>0){
            setNibbleBit(k);
        } else {
            clearNibbleBit(k);
        }
        n = n << 1;
    }
    P2OUT |= BIT7; // set RS
    latch();
    delay_ms(10);
}

int main(void)
{
    init();
    delay_ms(20);
    LCDsetup();

    clear_display();

    // Character "C"
    sendNibble(0b00000100);
    sendNibble(0b00000011);

    while(1){ }

    return 0;
}


void displayChar(short command){
    switch(command){
        case 0x00:
            sendNibble(0b00000011);
            sendNibble(0b00000000);
            break;
    }
}

#pragma vector=EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_TX_ISR(void){
    Rx_Command = UCB0RXBUF;    // Retrieve byte from buffer
    displayChar(Rx_Command);
    UCB0IFG &= ~UCTXIFG0;   // clear flag to allow I2C interrupt
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void){
    ms_count++;
    if(ms_count == ms_thresh){
        ms_flag = 1;
        ms_count = 0;
    }

    TB0CCTL0 &= ~CCIFG;
}

