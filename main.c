#include <msp430.h> 

int j = 0;
char packet[]= {0x03, 0x04};

void initI2C_master(){
     UCB1CTLW0 |= UCSWRST;       // SW RESET ON

     UCB1CTLW0 |= UCSSEL_3;      // SMCLK
     UCB1BRW = 1;               // prescalar to 10

     UCB1CTLW0 |= UCMODE_3;      // Put into I2C mode
     UCB1CTLW0 |= UCMST;         // set as MASTER
     UCB1CTLW0 |= UCTR;          // Put into Tx mode
     UCB1I2CSA = 0x0068;          // Slave address = 0x68

     UCB1CTLW1 |= UCASTP_2;      // enable automatic stop bit
     //UCB1TBCNT = sizeof(packet);  // Transfer byte count
     UCB1TBCNT = 1;  // Transfer byte count

     // setup ports
     P6DIR |= BIT6;  // set P6.6 as OUTPUT
     P6OUT &= ~BIT6; // clear P6.6

     P4SEL1 &= ~BIT7;            // P4.7 SCL
     P4SEL0 |= BIT7;

     P4SEL1 &= ~BIT6;            // P4.6 SDA
     P4SEL0 |= BIT6;

     PM5CTL0 &= ~LOCKLPM5;       // turn on I/O
     UCB1CTLW0 &= ~UCSWRST;      // SW RESET OFF

     UCB1IE |= UCTXIE0;          // enable I2C B0 Tx IRQ
}

int checkCol(int col_holding){
    if((BIT0 & col_holding)>0){
        return 0x00F1;
    } else if((BIT1 & col_holding)>0){
        return 0x00F2;
    } else if((BIT2 & col_holding)>0){
        return 0x00F4;
    } else if((BIT3 & col_holding)>0){
        return 0x00F8;
    } else {
        return 0x00;
    }
}

int checkRow(int row_holding){
    if((BIT4 & row_holding)>0){
        return 0x001F;
    } else if((BIT5 & row_holding)>0){
        return 0x002F;
    } else if((BIT6 & row_holding)>0){
        return 0x004F;
    } else if((BIT7 & row_holding)>0){
        return 0x008F;
    } else {
        return 0x00;
    }
}
void checkKeypad(){
    int i,j;
    for(i=0;i<50; i=i+1){
        for(j=0;j<1911; j=j+1){}   // small delay for button input
    }

    columnInput();

    int col_holding;
    int row_holding;
    int result;

    col_holding = checkCol(P3IN);

    rowInput();
    row_holding = checkRow(P3IN);

    // add R4 + R5
    result = row_holding + col_holding;

    // switch case
    switch(result){
        case 0x0087:
            // 1
            break;
        case 0x0083:
            // 2
            break;
        case 0x0081:
            // 3
            break;
        case 0x0080:
            //transmit("A");
            break;
        case 0x0047:
            // 4
            break;
        case 0x0043:
            // 5
            break;
        case 0x0041:
            // 6
            break;
        case 0x0040:
            // B
            break;
    }

    for(i=0;i<1500; i=i+1){}   // small delay for button input

}

//void columnInputOld(){
//    P3DIR &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // Initialize pins as input
//    P3REN |= (BIT0 | BIT1 | BIT2 | BIT3);  // Enable pull up/down resistor
//    P3OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3); // Configure resistors as pull down
//
//    P3DIR |= (BIT4 | BIT5 | BIT6 | BIT7);  // init pins as outputs
//    P3OUT |= (BIT4 | BIT5 | BIT6 | BIT7);  // set as outputs
//}

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
//        UCB1CTLW0 |= UCTXSTT;   // generate START condition
//        P6OUT |= BIT6; //  LED ON
//        for(i=0;i<20500; i=i+1){}   // delay
//        P6OUT &= ~BIT6; //  LED ON

        //checkKeypad(); // polling method - not working

    }

    return 0;
}

#pragma vector=PORT3_VECTOR
__interrupt void ISR_PORT3(void){

    /* enable timer for debouncing */
    TB0CCTL0 |= CCIE;       // local IRQ enable for CCR0
    TB0CCR0 = 2384;         // set CCR0 value (period)
    TB0CCTL0 &= ~CCIFG;     // clear CCR0 flag to begin count
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void){
    P6OUT ^= BIT6;
    UCB1CTLW0 |= UCTXSTT;   // generate START condition
    P3IFG &= ~(BIT0 | BIT1 | BIT2 | BIT3); // Clear the P3 interrupt flags
    TB0CCTL0 &= ~CCIE;
}

#pragma vector=EUSCI_B1_VECTOR
__interrupt void EUSCI_B1_TX_ISR(void){
    UCB1TXBUF = packet[0];
    if(packet[0] == 0x03){
        packet[0] = 0x04;
    } else {
        packet[0] = 0x03;
    }
}

//#pragma vector=EUSCI_B1_VECTOR
//__interrupt void EUSCI_B1_TX_ISR(void){
//    if (j == (sizeof(packet)-1)){
//        UCB1TXBUF = packet[j];
//        j = 0;
//    } else {
//        UCB1TXBUF = packet[j];
//        j++;
//    }
//}
