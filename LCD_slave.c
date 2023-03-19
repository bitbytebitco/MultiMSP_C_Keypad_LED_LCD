#include <msp430.h> 

volatile int Rx_Command = 0;

volatile int timer_action_select;
volatile int ms_thresh, ms_count, ms_flag;
//char rx_packet = {0x00};

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

    P2DIR |= BIT0;  // P1.1 LED output
    P2OUT &= ~BIT0; // clear P1.1

    initTimerB0compare();
    initTimerB1compare();
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
    P2OUT |= BIT0; // LED on
    ms_flag = 0;
    ms_count = 0;
    ms_thresh = ms;
    TB0CCTL0 |= CCIE;       // local IRQ enable for CCR0
    while(ms_flag != 1){}
    TB0CCTL0 &= ~CCIE;       // disable CCR0
    //flag_30_ms = 0;
    P2OUT &= ~BIT0; // LED off
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
//    P2OUT &= ~BIT7; // clear RS
//    P1OUT &= ~(BIT7 | BIT6 | BIT4);
//    P1OUT |= (BIT5);
//    latch();
//    delay_ms(10);

    sendByte(0b00101100, 0); // function set interface 4-bit & spec display lines and fonts

    sendByte(0b00001000, 0); // display off

    sendByte(0b00000001, 0); // clear display

    // spec display lines and fonts
//    P2OUT &= ~BIT7; // clear RS
//    P1OUT |= (BIT6);
//    P1OUT &= ~(BIT7 | BIT5 | BIT4);
//    latch();
//    delay_ms(10);

    // display on/off
//    P2OUT &= ~BIT7; // clear RS
//    P1OUT &= ~(BIT7 | BIT6 | BIT5 | BIT4);
//    latch();
//    delay_ms(10);
//    P2OUT &= ~BIT7; // clear RS
//    P1OUT |= (BIT7 | BIT6 );
//    P1OUT &= ~(BIT5 | BIT4);
//
//    latch();
//    delay_ms(30);

    sendByte(0b00000110, 0); // entry mode set

}

void clear_display(){
    P2OUT &= ~BIT7; // set RS
    P1OUT &= ~(BIT7 | BIT6 | BIT5 | BIT4);
    latch();
    delay_ms(2);
    P1OUT &= ~(BIT7 | BIT6 | BIT5 );
    P1OUT |= BIT4;
    latch();
    delay_ms(2);
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

void sendNibble(unsigned short byte, int rs) {
    if(rs == 1){
        P2OUT |= BIT7; // set RS
    } else {
        P2OUT &= ~BIT7; // set RS
    }

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
    latch();
    delay_ms(10);
}

void sendByte(int char_code, int rs){
    sendNibble(char_code >> 4, rs);
    sendNibble(char_code, rs);
}

void incr_index_reg_right(){
    sendByte(0b00010100, 0);

}

void executeCommand(int command){
    switch(command){
        case 0x80:
            // Character "C" as a test
                sendByte(0b01000001, 1);
            break;
        case 0x40:
            // Character "C" as a test
                sendByte(0b01000010, 1);
            break;
    }
}

void initTimerB1compare(){
    // setup TB0
    TB1CTL |= TBCLR;        // Clear TB0
    TB1CTL |= TBSSEL__ACLK; // select SMCLK
    TB1CTL |= MC__UP;       // UP mode
    TB1CCTL0 |= CCIE;       // local IRQ enable for CCR0

    //TB0CCTL0 &= ~CCIFG;     // clear CCR0 flag
}


int main(void)
{
    init();
    //P1OUT ^= BIT1; // clear P1.1
    //delay_ms(20);
//    LCDsetup();
//
//    sendByte(0b00001111, 0); // display on
//
//    sendByte(0b01000001, 1);
//    delay_ms(1);
//    sendByte(0b01000001, 1);
//    delay_ms(1);
//    sendByte(0b01000001, 1);
//    delay_ms(1);
//    sendByte(0b01000001, 1);

    while(1){}

    return 0;
}


#pragma vector=EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_TX_ISR(void){
    Rx_Command = UCB0RXBUF;    // Retrieve byte from buffer

//    P2OUT ^= BIT0; // clear P1.1
//    if(Rx_Command == 0x80){
        TB1CCTL0 |= CCIE;       // local IRQ enable for CCR0
        TB1CCR0 = 2000;         // set CCR0 value (period)
        TB1CCTL0 &= ~CCIFG;     // clear CCR0 flag to begin count
//    }


//    UCB0IFG &= ~UCTXIFG0;   // clear flag to allow I2C interrupt
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

#pragma vector=TIMER1_B0_VECTOR
__interrupt void ISR_TB1_CCR0(void){
    //executeCommand(Rx_Command);

    P2OUT ^= BIT0; // clear P1.1
    TB1CCTL0 &= ~CCIE; // disable timer
    TB1CCTL0 |= CCIFG;

    UCB0IFG &= ~UCTXIFG0;   // clear flag to allow I2C interrupt
}

