/*!******************************************************************
 * \file main_VIBRATION.c
 * \brief Sens'it Discovery mode Vibration demonstration code
 * \author Sens'it Team
 * \copyright Copyright (c) 2018 Sigfox, All Rights Reserved.
 *
 * For more information on this firmware, see vibration.md.
 *******************************************************************/
/******* INCLUDES **************************************************/
#include "sensit_types.h"
#include "sensit_api.h"
#include "error.h"
#include "button.h"
#include "battery.h"
#include "radio_api.h"
#include "fxos8700.h"
#include "discovery.h"


/******** DEFINES **************************************************/
#define VIBRATION_THRESHOLD                0x10 /* With 2g range, 3,9 mg threshold */
#define VIBRATION_COUNT                    2

/******* GLOBAL VARIABLES ******************************************/
u8 firmware_version[] = "VIBR_v2.0.0";

/**
 * Init sigfox module
 */
static void init()
{
    SENSIT_API_configure_button(INTERRUPT_BOTH_EGDE); /* Configure button */

    error_t err;

    err = RADIO_API_init(); /* Initialize Sens'it radio */
    ERROR_parser(err);

    err = FXOS8700_init(); /* Initialize accelerometer */
    ERROR_parser(err);

    /* Put accelerometer in transient mode */
    FXOS8700_set_transient_mode(FXOS8700_RANGE_2G, VIBRATION_THRESHOLD, VIBRATION_COUNT);

    /* Clear pending interrupt */
    pending_interrupt = 0;
}

typedef struct
{
    struct {
        s8 eventId:4;
        s8 reserved:4;
    };
    struct
    {
        s16 ax;
        s16 ay;
        s16 az;
    };
} __attribute__((packed)) data_t;

/**
 * Send data by sigfox
 * @param data
 * @return
 */
static void sendData(data_t *data)
{
    /* Send the message */
    error_t err = RADIO_API_send_message(RGB_BLUE, (u8 *) data, sizeof(data_t), FALSE, NULL);
    ERROR_parser(err);/* Parse the error code */
}

#define CHECK_EVENT(evt) (pending_interrupt & evt) == evt
#define CLEAR_EVENT(evt) pending_interrupt &= ~evt

/**
 * Convert value from little endian to Big endian
 * @param num s16 value in little endian
 * @return s16 value big endian
 */
s16 convertLittleToBig(s16 num) {
    return (num>>8) | (num<<8);
}

/**
 * Get acceleration information from sensor
 * @param data
 */
static void getAccInfo(data_t *data)
{
    fxos8700_data_s info;
    error_t err = FXOS8700_read_acceleration(&info);

    ERROR_parser(err);

    data->eventId = 1;

    data->ax = convertLittleToBig(info.x);
    data->ay = convertLittleToBig(info.y);
    data->az = convertLittleToBig(info.z);
}

int main()
{
    init();
    bool send = FALSE;

    while (TRUE) {
        data_t data = {0};

        /* RTC alarm interrupt handler */
        if (CHECK_EVENT(INTERRUPT_MASK_RTC)) {
            CLEAR_EVENT(INTERRUPT_MASK_RTC);
        }

        /* Button interrupt handler */
        if (CHECK_EVENT(INTERRUPT_MASK_BUTTON)) {
            SENSIT_API_set_rgb_led(RGB_BLUE); /* RGB Led ON during count of button presses */
            button_e btn = BUTTON_handler(); /* Count number of presses */
            SENSIT_API_set_rgb_led(RGB_OFF); /* RGB Led OFF */

            if (btn == BUTTON_TWO_PRESSES) {
                getAccInfo(&data);
                send = TRUE;
            } else if (btn == BUTTON_FOUR_PRESSES) {
                SENSIT_API_reset(); /* Reset the device */
            }

            CLEAR_EVENT(INTERRUPT_MASK_BUTTON);
        }

        /* Reed switch interrupt handler */
        if (CHECK_EVENT(INTERRUPT_MASK_REED_SWITCH)) {
            CLEAR_EVENT(INTERRUPT_MASK_REED_SWITCH);
        }

        /* Accelerometer interrupt handler */
        if (CHECK_EVENT(INTERRUPT_MASK_FXOS8700)) {
            getAccInfo(&data);
            send = TRUE;
            CLEAR_EVENT(INTERRUPT_MASK_FXOS8700);
        }

        /* Check if we need to send a message */
        if (send == TRUE) {
            sendData(&data);
            send = FALSE;
        }

        /* Check if all interrupt have been clear */
        if (pending_interrupt == 0) {
            SENSIT_API_sleep(FALSE); /* Wait for Interrupt */
        }
    } /* End of while */
}
