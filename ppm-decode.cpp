// Original code from http://pastebin.com/NQtbVCFh
// Posted on the leaflabs.com forum by Dweller:
// http://forums.leaflabs.com/topic.php?id=1170
///**********************************************************************
// Various Maple tests.. including timer capture to memory via dma =)
// */
#include "main.h"
#include "wirish.h"
#include "usb.h"
#include "timer.h"
#include "dma.h"
#include "utils.h"

//number of captures to do by dma
#define TIMERS 9

// timer prescale
#define TIMER_PRESCALE 26

// TIMER_PRESCALE*(1/72 MHz) =
#define TICK_PERIOD ( TIMER_PRESCALE*0.0000138888889f )

HardwareTimer timer1 = HardwareTimer(1);
volatile int dma_data_captured=0;            //set to 1 when dma complete.
volatile uint16 data[TIMERS];   //place to put the data via dma
uint16 delta=0;
uint16 ppm_timeout=0;
int do_print=1;

//dump routine to show content of captured data.
void printData(){
	float duty;
	for(int i=0; i<TIMERS; i++){

		if(ppm_timeout==1){
			if(do_print==1)
			{
				SerialUSB.println("PPM timeout!");
				do_print=0;
			}
			return;
		}

		if(i>0) delta = data[i]-data[i-1];
		else delta = data[i] - data[TIMERS-1];

		duty=(delta)*TICK_PERIOD;

		SerialUSB.print(delta);
		SerialUSB.print(":(");
		SerialUSB.print(duty);
		SerialUSB.print(")");
		if ((i+1)%9==0)	SerialUSB.print("\r");
		else SerialUSB.print("\t");
	}
	SerialUSB.println();
}

//invoked as configured by the DMA mode flags.
void dma_isr()
{
	dma_irq_cause cause = dma_get_irq_cause(DMA1, DMA_CH2);
        //using serialusb to print messages here is nice, but
        //it takes so long, we may never exit this isr invocation
        //before the next one comes in.. (dma is fast.. m'kay)

	timer1.setCount(0);  // clear counter
	if(ppm_timeout) ppm_timeout=0;
	if(!do_print) do_print=1;
	switch(cause)
	{
		case DMA_TRANSFER_COMPLETE:
			// Transfer completed
                        //SerialUSB.println("DMA Complete");
                        dma_data_captured=1;
			break;
		case DMA_TRANSFER_HALF_COMPLETE:
			// Transfer is half complete
                        SerialUSB.println("DMA Half Complete");
			break;
		case DMA_TRANSFER_ERROR:
			// An error occurred during transfer
                        SerialUSB.println("DMA Error");
                        dma_data_captured=1;
			break;
		default:
			// Something went horribly wrong.
			// Should never happen.
                        SerialUSB.println("DMA WTF");
                        dma_data_captured=1;
			break;
	}

}

void ppm_timeout_isr()
{
	// This failure mode indicates we lost comm with
	// the transmitter

	//TODO: re-initialize the timer and wait for ppm signal
	//re-initializing should ensure that the ppm signal
	// is captured on the sync pulse.
	ppm_timeout=1;
	dma_data_captured=0;
}

void init_timer_input_capture_dma()
{

    timer_dev *t = TIMER1;

    timer1.pause();
    timer1.setPrescaleFactor(TIMER_PRESCALE);
    timer1.setOverflow(65535);
    timer1.setCount(0);

    // use channel 2 to detect when we stop receiving
    // a ppm signal from the encoder.
    timer1.setMode(TIMER_CH2, TIMER_OUTPUT_COMPARE);
    timer1.setCompare(TIMER_CH2, 65535);
    timer1.attachCompare2Interrupt(ppm_timeout_isr);


    timer1.refresh();

    timer_reg_map r = t->regs;

    //using timer1, channel1, maps to pin d27 (maple mini) //d6 (maple?)
    //according to maple master pin map.
    pinMode(PPM_PIN,INPUT_PULLUP);

    //capture compare regs TIMx_CCRx used to hold val after a transition on corresponding ICx

    //when cap occurs, flag CCXIF (TIMx_SR register) is set,
    //and interrupt, or dma req can be sent if they are enabled.

    //if cap occurs while flag is already high CCxOF (overcapture) flag is set..

    //CCIX can be cleared by writing 0, or by reading the capped data from TIMx_CCRx
    //CCxOF is cleared by writing 0 to it.

    //Capture/Compare 1 Selection
    //  set CC1S bits to 01 in the capture compare mode register.
    //  01 selects TI1 as the input to use. (page 336 stm32 reference)
    //  (assuming here that TI1 is D27, according to maple master pin map)
    //CC1S bits are bits 0,1
    bitClear(r.adv->CCMR1, 1);
    bitSet(r.adv->CCMR1, 0);


    //Input Capture 1 Filter.
    //  need to set IC1F bits according to a table saying how long
    //  we should wait for a signal to be 'stable' to validate a transition
    //  on the input.
    //  (page 336 stm32 reference)
    //IC1F bits are bits 7,6,5,4
    bitClear(r.adv->CCMR1, 7);
    bitClear(r.adv->CCMR1, 6);
    bitSet(r.adv->CCMR1, 5);
    bitSet(r.adv->CCMR1, 4);

    //sort out the input capture prescaler..
    //00 no prescaler.. capture is done at every edge detected
    bitClear(r.adv->CCMR1, 3);
    bitClear(r.adv->CCMR1, 2);

    //select the edge for the transition on TI1 channel using CC1P in CCER
    //CC1P is bit 1 of CCER (page 339)
    // 0 = rising (non-inverted. capture is done on a rising edge of IC1)
    // 1 = falling (inverted. capture is done on a falling edge of IC1)
    bitClear(r.adv->CCER,1);

    //set the CC1E bit to enable capture from the counter.
    //CCE1 is bit 0 of CCER (page 339)
    bitSet(r.adv->CCER,0);

    //enable dma for this timer..
    //sets the Capture/Compare 1 DMA request enable bit on the DMA/interrupt enable register.
    //bit 9 is CC1DE as defined on page 329.
    bitSet(r.adv->DIER,9);

    dma_init(DMA1);
    dma_setup_transfer( DMA1,    //dma device, dma1 here because that's the only one we have
                        DMA_CH2, //dma channel, channel2, because it looks after tim1_ch1 (timer1, channel1)
                        &(r.adv->CCR1), //peripheral address
                        DMA_SIZE_16BITS, //peripheral size
                        data, //memory address
                        DMA_SIZE_16BITS, //memory transfer size
                        (0
                         //| DMA_FROM_MEM  //set if going from memory, don't set if going to memory.
                         | DMA_MINC_MODE //auto inc where the data does in memory (uses size_16bits to know how much)
                         | DMA_TRNS_ERR  //tell me if it's fubar
                         //| DMA_HALF_TRNS //tell me half way (actually, don't as I spend so long there, I dont see 'complete')
                         | DMA_TRNS_CMPLT //tell me at the end
                         | DMA_CIRC_MODE // circular mode... capture "num_transfers" (below) and repeat
                         )
                        );

    dma_attach_interrupt(DMA1, DMA_CH2, dma_isr); //hook up an isr for the dma chan to tell us if things happen.
    dma_set_num_transfers(DMA1, DMA_CH2, TIMERS); //only allow it to transfer TIMERS number of times.
    dma_enable(DMA1, DMA_CH2);                    //enable it..


}

void ppm_decode_interrupt_dma()
{

	SerialUSB.println("Starting timer.. rising edge of D27 (hopefully)");

	// wait 2 seconds
	delay(2000);
	SerialUSB.println("...go!");

	//start the timer counting.
	timer1.resume();
	//the timer is now counting up, and any falling edges on D6
	//will trigger a DMA request to clone the timercapture to the array

	//If we weren't using DMA, we could busy wait on the CC1IF bit in SR
	//
	//when the timer captures, the CC1IF bit will be set on the timer SR register.
	//CC1IF bit is bit 1 (page 332)
	//while(!bitRead(r.adv->SR, 1)){
	//}
	//SerialUSB.println("Timer triggered : ");
	//SerialUSB.println(r.adv->CCR1);

	//SerialUSB.println("Waiting for dma_data_captured flag from dma...");
	//busy wait on the dma_data_captured flag
	//we could do real work here if wanted..
	while(!dma_data_captured);

	while(!SerialUSB.available() && do_print==1)
	{
	  //dump the data
	  printData();
	  delay(100);
	}

	//turn off the timer & tidy up before we leave this cmd.
	timer1.pause();

	//      dma_disable(DMA1, DMA_CH2);
	//      dma_detach_interrupt(DMA1, DMA_CH2);
	dma_data_captured=0;

}

