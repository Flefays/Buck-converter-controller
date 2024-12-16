/* 
 * File:   Mycodelab1.c
 * Author: flori
 *
 * Created on October 3, 2024, 7:09 PM
 */
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#pragma config FOSC=INTIO67, LVP=OFF
#pragma config DEBUG=OFF, WDTEN=OFF
#pragma config CCP2MX=PORTB3

unsigned int buffer = 0; // Buffer to store the result of the ADC
unsigned int v = 0;//output voltage
unsigned int D = 0;
unsigned int ADC_channel = 0;
unsigned int v_ref = 511; //corresponds to 2.5V

signed int u_PID;
signed int u_p;
signed int u_i = 0;
signed int ud;

signed int k_p = 15;// value tuned on the hil
signed int k_i = 6;// value tuned on the hil
signed int k_d = 23;// value tuned on the hil

signed int e = 0;
signed int e_prev = 0;
signed int u_i_prev = 0;

bool flag_vout = false;
bool ON = true;



void __interrupt() isr(void) {
    if (PIR1bits.TMR2IF) { // Timer2 Interrupt
            PIR1bits.TMR2IF = 0; // Clear flag
            ADCON0bits.GO = 1; // This bit starts an A/D conversion

    if (PIR1bits.ADIF) { 
        PIR1bits.ADIF = 0; // //Clear flag
        buffer = (ADRESH << 8) | ADRESL;
        
        if(ADCON0bits.CHS == 0b00001){ //input channel RA1 for the output voltage
            ADCON0bits.CHS = 0b00000; //input channel RA0 for V_g
            ADCON0bits.GO = 1;
           
            LATBbits.LATB1 = buffer;
            LATBbits.LATB2 = (buffer >> 1);
            LATC = (buffer >> 2);
            v = buffer;
            flag_vout = true;
            
            
        }
        else if(ADCON0bits.CHS == 0b00000){ // RA0 for Vin
           
           ADCON0bits.CHS = 0b00001; // Reset input channel at RA1
           ON = true;
           if(buffer < 255){ //ON-OFF MODE : V_g>8V=>V_pot>1,25=>buffer>255
               ON = false;
           }
           
        }
    }
}


void ADC_Init(void){
    
    ADCON2bits.ADCS = 0b110;//conversion clock ,Fosc/64 because these value doesn't violate the minimum required Tad time
    ADCON1bits.PVCFG = 0b00; // positive voltage ref config AVdd
    ADCON1bits.NVCFG = 0b00; // negative voltage ref config AVss
    
    ADCON2bits.ADFM = 1; // Result format, right justified
    ADCON2bits.ACQT = 0b100; // Acquisition time, 8 Tad
    ADCON0bits.ADON=1; // Turn ON ADC module
}

void PWM_Init(void){// Init of the PWM with ECCP2 Half bridge, where the argument  v = voltage read by ADC
    TRISBbits.TRISB3 = 1; // PWM output set as an input
    
    PSTR2CONbits.STR2A  = 1 ; // P2A pin is assigned to PWM
    PSTR2CONbits.STR2B  = 0 ; // P2B pin is assigned to PWM
    CCPTMRS0bits.C2TSEL = 0b00 ; // Select Timer2 as the period timer  
    
    PR2 = 255;  //4MHz to 20kHz so, divided by 200, so Prx = 200-1 = 199
    
    //Shutdown configuration
    ECCP2ASbits.CCP2AS2  = 0b000; // Auto-shutdown is disabled (OFF)       
    ECCP2ASbits.PSS2AC  =  0b00;//When shut-down: voltage to 0, LED off   
    ECCP2ASbits.PSS2BD = 0b00;//Same but for port p2B and p2D
    
    PWM2CONbits.P2RSEN =0;//Manual restart if shutdown; to automatic restart set at 1
    ECCP2ASbits.CCP2ASE=1;//In shutdown state
    
    //PWM mode and configuration
    CCP2CONbits.CCP2M=0b1100;//Enhanced PWM mode: out put is active-high meaning that a high value=ON
    CCP2CONbits.P2M=0b00;//Set to a single output
    
    // Config Timer2 resource
    T2CONbits.T2OUTPS = 0b0000; // output post-scaler set to 1:1 to conserve the f=20kHz
    T2CONbits.T2CKPS = 0b01;  // pre-scaler ? 1:4 better than 1:16 Because the larger the Prx, the better the resolution
    T2CONbits.TMR2ON = 0b1; // on
    
    //RB3
    TRISBbits.TRISB3 = 0;
    
    ECCP2ASbits.CCP2ASE=0;//PWM GO
}

unsigned int PID(){
    
    if (ON){
    e = (signed int)(v_ref - v);
    u_p = (k_p * e)>>5;
    u_d = (k_d * (e - e_prev))>>5;

    if(u_PID <= 1023 && u_PID >= 0) { // check if no saturation
	u_i = u_i_prev + ((k_i * e)>>7);
    }

    if (u_i > 32767 ) {//check if overflow
        u_i = 32767;
    }
    else if (u_i < -32767){//check if overflow
        u_i = 32767;
    }

    u_PID = up + ui + ud;
    e_prev = e;
    ui_prev = ui;


    if (u_PID > 1023) {//saturation case 1
        
        flag_vout = false;
        return 1023;
    }
    else if (u_PID < 0){// saturation case 2
        flag_vout = false;
        return 0;
    }
    else {
        flag_vout= false;
        return (unsigned int)u_PID;}

    }

    else if(!ON)
    {
        flag_vout = false;
        return 0;}
}
    
    

void main(void) {
    // Clock config
    OSCCONbits.IRCF = 0b111; // set to 16MHz
    OSCTUNEbits.PLLEN = 1; // enable the PLLEN block to multiply by 4 (16MHz to 64 MHz)
    OSCCONbits.SCS = 0b00; // Primary clock to access at the PLLEN block
    
    // Enable interrupts
    INTCONbits.PEIE = 1; //  Enable peripheral interrupt
    INTCONbits.GIE = 1; // Enable global interrupt
    PIE1bits.TMR2IE = 1; 

    // Config RB3 (pin 24 -> PWM LED)
    TRISB3 = 0; // RB3 set as output
    ANSB3 = 0; // RB3 set as digital
    
    //Setting all the pin used (at some point) as inputs(TRIS=1)/ ouputs(TRIS=0) and as analog(ANS=1)/digital(ANS=0) as needed
    TRISAbits.TRISA0 = 1; 
    ANSELAbits.ANSA0 = 1;

    TRISAbits.TRISA1 = 1;
    ANSELAbits.ANSA1 = 1;

    TRISAbits.TRISA5 = 1; 
    ANSELAbits.ANSA5 = 1; 
    
    TRISAbits.TRISA6 = 0; 
    TRISAbits.TRISA7 = 0; 
   
    TRISC = 0x00; //
    TRISB = 0b11111001; //RB1 and RB2 as output
    ANSELC = 0x00; 
    ANSELB = 0x11111001; 

    //ADC
    ADC_init();
    ADCON0bits.CHS = 0b00001;
    PIE1bits.ADIE = 1; // Enable ADC interrupt
   
    PWM_init();   
    
    while (1) {
       
        
        if(flag_vout){
            
        
        D=PID();

	// Load the eight MSBs into the CCPR1L register
        CCPR2L  = (char)(D>>2); // Shift right by 2 bits to get MSBs
            // Load the two LSBs into the DC<1:0> bits of the CCP1CON register
        CCP2CONbits.DC2B0 = D & 0x01 ; // Bit 0 of the LSB
        CCP2CONbits.DC2B1 = (D>>1) & 0x01 ; // Bit 1 of the LSB
        
        
    }
}
}

