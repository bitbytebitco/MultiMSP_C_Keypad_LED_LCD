#include <msp430.h> 

int Rx_Command = 0;
volatile int action_select = 0;
volatile int ms_thresh, ms_count, ms_flag;

void initI2C_slave(){
    UCB0CTLW0 |= UCSWRST;       // SW RESET enabled
    UCB0CTLW0 |= UCMODE_3 | UCSYNC;      // Put into I2C mode
    UCB0I2COA0 = 0x0068 | UCOAEN; // Set slave address + enable

    // setup ports
    P1SEL1 &= ~BIT3;            // P1.3 SCL
    P1SEL0 |= BIT3;

    P1SEL1 &= ~BIT2;            // P1.2 SDA
    P1SEL0 |= BIT2;

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

    initI2C_slave();
    initTimerB0compare();

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
    delay_ms(2);
}

void sendByte(int char_code, int rs){
    sendNibble(char_code >> 4, rs);
    sendNibble(char_code, rs);
}

void incr_index_reg_right(){
    sendByte(0b00010100, 0);
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

    sendByte(0b00101100, 0); // function set interface 4-bit & spec display lines and fonts

    sendByte(0b00001000, 0); // display off

    sendByte(0b00000001, 0); // clear display

    sendByte(0b00000110, 0); // entry mode set

}

int main(void)
{
    init();
    delay_ms(20);
    LCDsetup();

    sendByte(0b00001111, 0); // display on
    clear_display();

//    sendByte(0b01000001, 1); // "A"

    while(1){
        if(action_select == 1){
            sendByte(0b01000001, 1); // "A"
            action_select = 0;
        }
    }

    return 0;
}

void executeCommand(int command){
}

#pragma vector=EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_TX_ISR(void){
    switch(UCB0IV){
        case 0x16:  // receiving
                Rx_Command = UCB0RXBUF;    // Retrieve byte from buffer
//                executeCommand(Rx_Command);
                action_select = 1;
                P1OUT ^= BIT1;
            break;
        case 0x18:
            break;
    }

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
