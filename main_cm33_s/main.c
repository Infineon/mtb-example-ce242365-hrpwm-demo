/*****************************************************************************
* File Name        : main.c
*
* Description      : This source file contains the main routine for secure
*                    application in the CM33 CPU
*
* Related Document : See README.md
*
********************************************************************************
* (c) 2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/


/******************************************************************************
 * Header Files
 *****************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
*******************************************************************************/

/* These are the addresses where the core0 and core1 images are located. */
#define CORE0_IMAGE_ADDRESS    CYMEM_CM33_0_S_m33s_ppca0_nvm_C_S_START    //  0x12030000
#define CORE1_IMAGE_ADDRESS    CYMEM_CM33_0_S_m33s_ppca1_nvm_C_S_START    //  0x12038000

#define PPCA0_IMAGE_SIZE       CYMEM_CM33_0_S_ppca0_code_SIZE
#define PPCA1_IMAGE_SIZE       CYMEM_CM33_0_S_ppca1_code_SIZE

/* Shared memory addresses for inter-core communication */
/* PPCA cores write to these locations, main core reads from them */
/* Variables located in M4 shared memory space (16KB at 0x20040000-0x20043FFF from PPCA view) */
/* Main core accesses PPCA memory through PPCA peripheral base with memory windows: */
/* M1 (CPU0 data): 0x53020000, M3 (CPU1 data): 0x53040000, M4 (shared): 0x53050000 */
#define PPCA_CPU0_M4_VAR_ADDRESS   0x53050400  /* Written by PPCA Core 0 */
#define PPCA_CPU1_M4_VAR_ADDRESS   0x53050800  /* Written by PPCA Core 1 */

/* ADC averaging buffer size for stable readings */
#define BUFFER_SIZE                (8)
/* Step size for fractional duty cycle adjustment */
#define COMPARE_VALUE_DELTA        (1)
/* PWM input clock frequency in MHz */
#define PWM_FREQ_MHZ               (200)
/* Number of bits representing the integer part of HrPWM period/compare value */
#define INTEGER_BITS               (6)
/* Mask to extract the 6-bit fractional part from HrPWM period/compare value */
#define FRACTIONAL_MASK_BITS       (0x3F)
/* ADC reference voltage in millivolts */
#define ADC_VOLTAGE_HRPWM          (3300)
/* Maximum 12-bit ADC value */
#define ADC_HIGH_VALUE             (4095)

/* UART key codes for user input */
#define DATA_LENGTH    10
#define W_KEY          0x77    /* 'w' - Increase integer part */
#define E_KEY          0x65    /* 'e' - Increase fractional part */
#define S_KEY          0x73    /* 's' - Decrease integer part */
#define D_KEY          0x64    /* 'd' - Decrease fractional part */
#define ENTER          0x0D    /* Enter - Start/stop ADC conversion */
#define MUL_HEX        0x0F

/* HrPWM compare value boundaries */
#define     LOWER_HRPPWM         64      /* Minimum CC0 value (1 integer step) */
#define     INTEGER_STEP         64      /* One integer step equals 64 fractional steps */
#define     HIGHER_HRPPWM        2496    /* Maximum CC0 value for valid duty cycle */
#define     PERCENT_NUM          100     /* Multiplier for percentage calculation */

/*******************************************************************************
* Global Variables
*******************************************************************************/

/* Debug UART variables */
static cy_stc_scb_uart_context_t    DEBUG_UART_context; /* DEBUG_UART context */
static mtb_hal_uart_t               DEBUG_UART_hal_obj; /* Debug DEBUG_UART HAL object */

/* Populate interrupt configuration structure */
cy_stc_sysint_t UART_SCB_IRQ_cfg =
{
    .intrSrc      = DEBUG_UART_IRQ,
    .intrPriority = 3u,
};

/* ADC conversion starting flag */
volatile bool start_adc_conversion = false;
volatile bool stop_adc_conversion = true;

/* UART received command. */
int rec_cmd = 0;

float32_t dc_value;                /* Calculated duty cycle percentage */
uint16_t  period;                  /* HrPWM period value (integer + fractional parts) */
int16_t   compare0_value;          /* HrPWM CC0 compare value for duty cycle control */

uint16_t intr_period;              /* Integer part of the HrPWM period */
uint16_t frac_period;              /* Fractional part of the HrPWM period */
uint8_t hrpwm_freq;                /* Calculated HrPWM output frequency in MHz */
/*******************************************************************************
* Function Prototype
*******************************************************************************/
/*Initialization of the UARTperipheral */
void uart_port_init(void);

/*Initialization of the ADC*/
void adc_init(void);

/* UART callback */
void uart_event_handler(uint32_t event);

/* UART interrupt handler */
void Isr_uart_fifo(void);

/* Function to read the ADC value */
void read_adc_channel_result(void);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM33 CPU. It performs:
*    1. Initialization of board peripherals (UART, ADC, HrPWM)
*    2. Configuration of PPCA and EPU for multi-core operation
*    3. Displays HrPWM application information and user instructions
*    4. Starts PPCA cores (CPU0 and CPU1)
*    5. Continuously reads and displays ADC values when conversion is enabled
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/

int main(void)
{
    cy_rslt_t result;
    cy_en_tcpwm_status_t status;
    
    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize UART port */
    uart_port_init();

    /* Initialize PPCA (Programmable Power Control Accelerator) configuration.
     * PPCA enables efficient peripheral control and multi-core coordination. */
    Cy_PPCA_CNFG_Init(PPCA_CNFG, &CNFG_config);
    Cy_PPCA_Enable(PPCA_CNFG);

    /* Initialize EPU (Event Processing Unit) for hardware event handling.
     * Enable exclusive access to prevent resource conflicts between cores. */
    Cy_PPCA_EPU_EnableExclusiveAccess(EPU_HW, true);
    Cy_PPCA_EPU_Enable(EPU_HW);
    
    /* Initializing the PWM */
    status = Cy_TCPWM_PWM_Init(HrPWM_HW, HrPWM_NUM, &HrPWM_config);

    /* Initialization failed */
    if(CY_TCPWM_SUCCESS != status)
    {
        CY_ASSERT(0);
    }

    /* Enable the initialized PWM */
    Cy_TCPWM_PWM_Enable(HrPWM_HW, HrPWM_NUM);
    
    /* Initialize ADC pins */
    adc_init();

    /* Transmit header to the terminal */
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    /* Printing HRPWM message */
    printf("************************************************************\r\n");
    printf("PSOC Control C3M/P8: HrPWM application\r\n");
    printf("************************************************************\r\n\n");

   printf("HRPWM (High Resolution Pulse Width Modulation) enhances the time resolution"
             "\r\nof traditional digital PWM (Pulse Width Modulation) signals.\r\n\n");
   printf("This demo uses HRPWM to generate a 5 MHz PWM output with more resolution"
             "\r\nthan what is possible with standard digital PWM with 200 MHz clock input.\r\n\n");
   printf("While standard digital PWM gives only 40 steps of resolution (0 to 39),"
             "\r\nHRPWM in PSOC Control C3M/P8 provides 64 fractional steps per digital step.\r\n\n");
   printf("----------          -------------          -----------\r\n");
   printf("| HRPWM  |--------->| RC Filter |--------->| SAR ADC |\r\n");
   printf("----------          -------------          -----------\r\n\n");
   printf("The HRPWM output is connected to the SAR ADC through an RC filter."
             "\r\nThe SAR ADC sampler gain is set to 6. This gives approx. 5 count"
             "\r\nvariation for each fractional step change.\r\n\n");

   printf("CPU frequency          : %u MHz\r\n", cy_delayFreqMhz);
   printf("HRPWM Clock frequency  : %u MHz\r\n", (uint8_t)PWM_FREQ_MHZ);

   period = Cy_TCPWM_PWM_GetPeriod0(HrPWM_HW, HrPWM_NUM);
   compare0_value = Cy_TCPWM_PWM_GetCompare0Val(HrPWM_HW, HrPWM_NUM);

   intr_period = period >> INTEGER_BITS;
   frac_period = period & FRACTIONAL_MASK_BITS;
   hrpwm_freq = (uint8_t)PWM_FREQ_MHZ/intr_period;

   printf("HRPWM output frequency : %3u MHz\r\n\n", hrpwm_freq);

   /* Then start the PWM B */
   Cy_TCPWM_TriggerStart_Single(HrPWM_HW, HrPWM_NUM);

   printf("==========================================================================\r\n");
   printf("Instructions:\r\n");
   printf("==========================================================================\r\n");
   printf("Press 'w'     : To Increasing the Integer Part by 1 (64 Steps)\r\n");
   printf("Press 'e'     : To Increasing the Fractional Part by 1 (1 Step)\r\n");
   printf("Press 's'     : To Decreasing the Integer Part by 1 (64 Steps)\r\n");
   printf("Press 'd'     : To Decreasing the Fractional Part by 1 (1 Step)\r\n");
   printf("Press 'Enter' : To start/stop the HRPWM - SAR ADC conversion\r\n");
   printf("==========================================================================\r\n");
   printf("Period: %u [Integer Part : %02u, Fractional Part : %02u]\r\n", period, intr_period, frac_period);
   printf("==========================================================================\r\n\n");

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize and start PPCA Core 0 (CM33_0) with its firmware image */
    Cy_System_Init_CPU0((void*)CORE0_IMAGE_ADDRESS, PPCA0_IMAGE_SIZE);
    /* Initialize and start PPCA Core 1 (CM33_1) with its firmware image */
    Cy_System_Init_CPU1((void*)CORE1_IMAGE_ADDRESS, PPCA1_IMAGE_SIZE);

    for (;;)
    {
        /* Read ADC channel result and print out the result */
        while((start_adc_conversion == true) && (stop_adc_conversion == false))
        {
            read_adc_channel_result();
            Cy_SysLib_Delay(1000);
        }
    }
}

/*******************************************************************************
 * Function Name: uart_port_init
 ********************************************************************************
 * Summary:
 * Initialize the uart port to print in the Serial monitor and get the user input.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void uart_port_init(void)
{
    cy_rslt_t result;
    /* Initialize retarget-io to use the debug UART port */
    result = (cy_rslt_t)Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Registers a callback function that notifies that
     *  uart_callback_events occurred in the Cy_SCB_UART_Interrupt.*/
    Cy_SCB_UART_RegisterCallback(DEBUG_UART_HW, (cy_cb_scb_uart_handle_events_t)uart_event_handler, &DEBUG_UART_context);

    /* Configuring priority and enabling NVIC IRQ
     * for the defined Service Request line number */
    Cy_SysInt_Init(&UART_SCB_IRQ_cfg, Isr_uart_fifo);
    NVIC_EnableIRQ(UART_SCB_IRQ_cfg.intrSrc);

    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);

    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);

}

/*******************************************************************************
 * Function Name: adc_init
 ********************************************************************************
 * Summary:
 * This function is used to initialize the ADC channel.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void adc_init()
{
    /* Initializing ATOP Analog reference */
    Cy_PPCA_AREF_Init(AREF_HW, &AREF_config);

    /* Enabling AREF */
    Cy_PPCA_AREF_Enable(AREF_HW);

    /* Initializing ATOP ADC 0*/
    Cy_PPCA_ADC_Init(ADC2_HW, &ADC2_config);

    /* Enabling ADC 2*/
    Cy_PPCA_ADC_Enable(ADC2_HW);
}
/********************************************************************************
 * Function Name: uart_event_handler
 ********************************************************************************
 * Summary:
 * Uart interrupt event handler callback function
 *
 * Parameters:
 *  handler_arg: user defined argument
 *  event: uart interrupt event source
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void uart_event_handler(uint32_t event)
{
    if (event == CY_SCB_UART_TRANSMIT_ERR_EVENT)
    {
        CY_ASSERT(0);
        /* An error occurred in Tx */
        /* Insert application code to handle Tx error */
    }
    else if (event == CY_SCB_UART_TRANSMIT_DONE_EVENT)
    {
        Cy_SCB_UART_ClearRingBuffer(DEBUG_UART_HW, &DEBUG_UART_context);
        /* All Tx data has been transmitted */
        /* Insert application code to handle Tx done */
    }
    else if (event == CY_SCB_UART_RECEIVE_DONE_EVENT)
    {
        CY_ASSERT(0);
        /* All Rx data has been received */
        /* Insert application code to handle Rx done */
    }
    else if (event == CY_SCB_UART_RECEIVE_NOT_EMTPY)
    {
        /* Get input command */
        uint32_t read_value = Cy_SCB_UART_Get(DEBUG_UART_HW);
        rec_cmd = (uint8_t)read_value;

        /* Distinguish command */
        switch(rec_cmd)
        {
        case ENTER:
            if(start_adc_conversion == true)
            {
                start_adc_conversion = false;
                stop_adc_conversion = !start_adc_conversion;
                printf("\r\nstop the ADC conversion \r\n\n");
            }
            else
            {
                start_adc_conversion = true;
                stop_adc_conversion = !start_adc_conversion;
                printf("start the ADC conversion \r\n\n");
            }

            break;
        case W_KEY:
            /* Increment the CC0 value in multiples of 64 steps */
            compare0_value += INTEGER_STEP;
            /* If CC0 is higher than 422, make CC0 to 422 */
            if(compare0_value > HIGHER_HRPPWM)
                compare0_value = HIGHER_HRPPWM;

            dc_value = (compare0_value * PERCENT_NUM);
            break;
        case E_KEY:
            /* Increment the CC0 value in terms of 1 step */
            compare0_value += COMPARE_VALUE_DELTA;

            /* If CC0 is higher than 422, make CC0 to 422 */
            if(compare0_value > HIGHER_HRPPWM)
                compare0_value = HIGHER_HRPPWM;

            dc_value = (compare0_value * PERCENT_NUM);
            break;
        case S_KEY:
            /* Decrement the CC0 value in multiples of 64 steps */
            compare0_value -= INTEGER_STEP;

            /* If CC0 value is less than 64 and greater than 0, make the duty cycle 0 */
            if(compare0_value < LOWER_HRPPWM && compare0_value >= 0)
            {
                dc_value = (compare0_value * PERCENT_NUM);
            }
            /* If CC0 value is less than 0, equate the CC0 to 0 */
            else if(compare0_value <= 0)
            {
                compare0_value = 0;
            }
            break;
        case D_KEY:
            /* Decrement the CC0 value in terms of 1 step */
            compare0_value -= COMPARE_VALUE_DELTA;

            /* If CC0 value is less than 64 and greater than 0, make the duty cycle 0 */
            if(compare0_value < LOWER_HRPPWM && compare0_value >= 0)
            {
                dc_value = (compare0_value * PERCENT_NUM);
            }
            /* If CC0 value is less than 0, equate the CC0 to 0 */
            else if(compare0_value <= 0)
            {
                compare0_value = 0;
            }
            break;
        }
        /* Set new values for CC0/1 compare */
        Cy_TCPWM_PWM_SetCompare0(HrPWM_HW, HrPWM_NUM, compare0_value);
        Cy_SysLib_Delay(50);
    }
}

/*******************************************************************************
* Function Name: Isr_uart_fifo
********************************************************************************
* Summary:
* This function is registered to be called when UART1 interrupt occurs.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void Isr_uart_fifo(void)
{
    Cy_SCB_UART_Interrupt(DEBUG_UART_HW, &DEBUG_UART_context);
}

/*******************************************************************************
* Function Name: read_adc_channel_result
********************************************************************************
* Summary:
* This function reads the SAR ADC channel result by polling and displays
* the averaged ADC count, voltage, and HrPWM duty cycle information.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void read_adc_channel_result(void)
{
    uint16_t channel_result = 0;
    float32_t volts = 0;
    float32_t buffer_volt[BUFFER_SIZE];
    float32_t mean_value_volt;
    float32_t buffer_channel[BUFFER_SIZE];
    float32_t mean_value_channel;
    int32_t buffer_index = 0;

    mean_value_volt = 0;
    mean_value_channel = 0;

    /* Averaging the read data */
    for (int32_t average = 0; average < BUFFER_SIZE; average++)
    {
          /* Trigger SAR ADC group 0 conversion */
          Cy_PPCA_ADC_Manual_Trigger(ADC2_HW, 3);

          /* Wait for channel conversion done */
          do
          {

          }while(true == Cy_PPCA_ADC_Is_ADC_Busy(ADC2_HW));

          /* Get channel data */
          channel_result = Cy_PPCA_ADC_Read_ADC_Data(ADC2_HW, 3);

          /* Saving 8 consecutive ADC values */
          buffer_channel[buffer_index] = channel_result;
          mean_value_channel += buffer_channel[average];

          /* mapping ADC values to 3.3 Volts */
          volts = (channel_result*3.3)/4096;

          /* Saving 8 consecutive voltages */
          buffer_volt[buffer_index] = volts;
          mean_value_volt += buffer_volt[average];

          buffer_index = (buffer_index + 1) % BUFFER_SIZE;
          Cy_SysLib_Delay(10);
    }

    /* Averaging 16 consecutive ADC values to have a stable ADC value */
    mean_value_channel = mean_value_channel/BUFFER_SIZE;

    /* Averaging 16 consecutive voltages to have a stable voltage */
    mean_value_volt = mean_value_volt/BUFFER_SIZE;
    dc_value = (compare0_value * PERCENT_NUM);
    Cy_SysLib_Delay(10);

    /* If the CC0 value is less than 64, make the duty cycle 0 */
    dc_value = dc_value/period;
    if(compare0_value <= (LOWER_HRPPWM-1))
    {
         dc_value = 0;
    }

    int16_t integer_part1 = (int16_t)mean_value_channel;
    uint16_t integer_part2 = compare0_value >> INTEGER_BITS;
    uint16_t fractional_part = compare0_value & FRACTIONAL_MASK_BITS;

    /* print the HRPWM values in a readable format */
    printf("HRPWM - Period: %u, CC0: %04d [Integer Part : %02u, Fractional Part : "
           "%02u], Duty: %06.2f %%, ADC count = %04d", period, compare0_value,
           integer_part2, fractional_part, dc_value, integer_part1);

    /* If the ADC value goes above 4095, print the following message */
    if(integer_part1 == ADC_HIGH_VALUE)
    {
         printf(". ADC input at max value. Reduce CC0 Integer/Fractional Part\r\n");
    }

    /* If the ADC value goes below 64, print the following message */
    else if(compare0_value <= (LOWER_HRPPWM-1))
    {
         printf(". CC0 Value below 64 results in Duty = 0 %%\r\n");
    }
    else
    {
         printf("\r\n");
    }
}
