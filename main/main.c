#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_system.h"
#include "esp_intr_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "driver/sdmmc_types.h"
#include "driver/spi_common.h"
#include "esp_lvgl_port.h"

// <<-- UART DATA TRANSFER DEFINITIONS -->>
#define UART_PORT UART_NUM_0
#define UART_BUFF_SZ 1024*2
#define UART_QUEUE_SZ 10
#define UART_TX_PIN GPIO_NUM_1
#define UART_RX_PIN GPIO_NUM_3

// <<-- GPIO PIN DEFINITIONS -->>
#define GPIO_LED GPIO_NUM_27
#define GPIO_FLOW GPIO_NUM_25
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_OUTPUT_PIN_SEL (1ULL<<GPIO_LED)
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_FLOW)

// <<-- LCD DISPLAY DEFINITIONS -->>
#define DISPLAY_MOSI GPIO_NUM_23
#define DISPLAY_SCLK GPIO_NUM_18
#define DISPLAY_CS GPIO_NUM_5
#define DISPLAY_DC GPIO_NUM_22
#define DISPLAY_RST GPIO_NUM_21
#define DISPLAY_BL GPIO_NUM_26

#define DISPLAY_HEIGHT 320
#define DISPLAY_WIDTH 240

// <<-- PULSE CONTROL DEFINTIONS -->>
#define PWM_NUM_MODES 3
#define MAX_FLOW 100
#define PWM_FREQ 150
#define PWM_GPIO GPIO_NUM_32
#define PWM_SPD_MODE LEDC_LOW_SPEED_MODE
#define PWM_TIMER LEDC_TIMER_0
#define PWM_CH LEDC_CHANNEL_0
#define PWM_DUTY_RES LEDC_TIMER_10_BIT
#define PWM_PERIOD_MS 200

// <<-- FLOW CONTROL DEFINITIONS -->>
#define FLOW_BUFF_SIZE 20
#define FLOW_OUTPUT_BUFF_SIZE 20
#define FLOW_OUTPUT_PERIOD_MS 200
#define FLOW_DEFAULT_BUFFER_VAL pdMS_TO_TICKS(1200) //UINT32_MAX
#define FLOW_DEBOUNCE 5
#define FLOW_K_VALUE 0.26
#define FLOW_PULSE_TIMEOUT 1000

// <<-- DATA SAVING DEFINITIONS -->>
#define DATA_GPIO_CMD GPIO_NUM_15
#define DATA_GPIO_CLK GPIO_NUM_14
#define DATA_GPIO_D0 GPIO_NUM_2
#define DATA_GPIO_D1 GPIO_NUM_4
#define DATA_GPIO_D2 GPIO_NUM_12
#define DATA_GPIO_D3 GPIO_NUM_13
#define DATA_NUM_MODES 2

// <<-- SERIAL STUDIO OUTPUT DEFINITIONS -->>
#define SERIAL_OUTPUT 1

// <<-- CUSTOM TYPEDEFS -->>

typedef struct  {
    volatile float* pntrRate;
    volatile float* pntrFreq;
    int buffIdx;
    float buffRate[FLOW_OUTPUT_BUFF_SIZE];
    float buffFreq[FLOW_OUTPUT_BUFF_SIZE];
    volatile TickType_t buffTimestamp[FLOW_OUTPUT_BUFF_SIZE];
    volatile TickType_t flowTicks[FLOW_BUFF_SIZE];
} flow_data_t;

typedef struct {
    volatile TickType_t period;
    volatile TickType_t timestamp;
} isr_time_data_t;

// GLOBAL VARIABLES - GENERAL
static const char *TAG = "PumpCtrl";

// GLOBAL VARIABLES - UART
static QueueHandle_t UART_event_q = NULL;

// GLOBAL VARIABLES - FLOW
static volatile DRAM_ATTR TickType_t flow_last_tick = 0;
//static volatile TickType_t flow_buff[FLOW_BUFF_SIZE];
static volatile uint8_t flow_tar = 0;
static volatile float flowRate = 0;
static volatile float flowFreq = 0;
portMUX_TYPE flow_spinlock = portMUX_INITIALIZER_UNLOCKED;
QueueHandle_t flow_event_q = NULL;

flow_data_t flow_meter = {
    .pntrRate = &flowRate,
    .pntrFreq = &flowFreq,
    .buffIdx = 0
};

// GLOBAL VARIABLE - DATA
bool mountState = false;
int dataWrite = 0;
FILE *f;
static const char data_label[DATA_NUM_MODES][20] = {"Off","On"};

// GLOBAL VARIABLES - LCD
esp_lcd_panel_handle_t disp_handle;

lv_display_t *DISP_LCD;
lv_obj_t *init_Label;

// GLOBAL VARIABLES PUSLE CTRL
static uint8_t target_spd = 0;
static uint8_t curr_spd = 0;
static int mode = 0;
static const char mode_label[PWM_NUM_MODES][20] = {"Off","Auto","Manual"};
// static float pwm_duty = 0;
esp_timer_handle_t auto_pwm_timer = NULL;

void IRAM_ATTR flow_meter_isr(void * args){
    
TickType_t local_now_tick = xTaskGetTickCountFromISR();
    TickType_t isr_new_period = local_now_tick-flow_last_tick;
    if(pdTICKS_TO_MS(isr_new_period) > FLOW_DEBOUNCE){
        flow_last_tick = local_now_tick;
        isr_time_data_t isr_new_time = {
            .period = isr_new_period,
            .timestamp = local_now_tick
        };
        xQueueSendFromISR(flow_event_q,&isr_new_time,NULL);
    }
}

void init_UART(){
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT,UART_BUFF_SZ,UART_BUFF_SZ,UART_QUEUE_SZ,&UART_event_q,0));

    uart_config_t UART_CONFIG = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &UART_CONFIG));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT,UART_TX_PIN,UART_RX_PIN,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
}

void init_GPIO(){
    //zero-initialize the config structure
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO35/36
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of falling edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    flow_event_q = xQueueCreate(10,sizeof(isr_time_data_t));

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_FLOW, flow_meter_isr, NULL);
}

void init_Display(){

    gpio_set_direction(DISPLAY_DC, GPIO_MODE_OUTPUT);

    spi_bus_config_t bus_config = {
        .mosi_io_num = DISPLAY_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = DISPLAY_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 320*240*sizeof(uint16_t)
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));


    esp_lcd_panel_io_spi_config_t io_config = {0};
        io_config.dc_gpio_num = DISPLAY_DC;
        io_config.cs_gpio_num = DISPLAY_CS;
        io_config.pclk_hz = 20 * 1000 * 1000;
        io_config.spi_mode = 0;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        io_config.trans_queue_depth = 10;
        io_config.on_color_trans_done = NULL;
        io_config.user_ctx = NULL;

    esp_lcd_panel_io_handle_t lcd_io;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &lcd_io));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = DISPLAY_RST,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &disp_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(disp_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(disp_handle));

    gpio_set_direction(DISPLAY_DC, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(disp_handle, true));

    const lvgl_port_cfg_t lvgl_config = {
        .task_priority = 5,
        .task_stack = 4096,
        .task_affinity = -1,
        .task_max_sleep_ms = 500,
        .timer_period_ms = 5
    };
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_config));

    const lvgl_port_display_cfg_t disp_config = {
        .io_handle = lcd_io,
        .panel_handle = disp_handle,
        .buffer_size = DISPLAY_HEIGHT*DISPLAY_WIDTH*2/10,
        .double_buffer = 0,
        .hres = DISPLAY_HEIGHT,
        .vres = DISPLAY_WIDTH,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = true,
            .mirror_x = false,
            .mirror_y = true
        },
        .flags = {
            .buff_dma = false,
            .buff_spiram = false,
            .direct_mode = false,
            .swap_bytes = false
        }
    };

    DISP_LCD = lvgl_port_add_disp(&disp_config);

    init_Label = lv_label_create(lv_screen_active());
    lv_label_set_text_fmt(init_Label,"Mode: %s\nTarget Flow Rate: %d%%\nCommanded Flow Rate: %d%%\n",mode_label[mode],target_spd,curr_spd);
    lv_obj_set_style_text_font(init_Label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_pos(init_Label, 0, 0);
}

void init_PWM(){
    ledc_timer_config_t PWM_timer = {
        .speed_mode = PWM_SPD_MODE,
        .duty_resolution = PWM_DUTY_RES,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&PWM_timer));

    ledc_channel_config_t PWM_channel = {
        .speed_mode = PWM_SPD_MODE,
        .channel = PWM_CH,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWM_GPIO,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&PWM_channel));
}

void init_DATA(){
    esp_err_t return_ERR;
    sdmmc_card_t *SD_card;
    const char mount_point[] = "/sdcard";

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;

    sdmmc_slot_config_t slot_config = {
        .clk = DATA_GPIO_CLK,
        .cmd = DATA_GPIO_CMD,
        .d0 = DATA_GPIO_D0,
        .width = 1,
        .flags = 0,
        .cd = SDMMC_SLOT_NO_CD,
        .wp = SDMMC_SLOT_NO_WP
    };

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    return_ERR = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &SD_card);

    if(return_ERR != ESP_OK){
        ESP_LOGE(TAG,"Failed to mount filesystem: %s",esp_err_to_name(return_ERR));
        return;
    }

    mountState = true;
    ESP_LOGI(TAG,"Successfully mounted filesystem");
}

void init_TIMER(){

    // const esp_timer_create_args_t periodic_pwm_timer_args = {
    //     .callback = pwm_cb,
    //     .arg = &curr_spd,
    //     .name = "Local_Timestamp"
    // };
    // ESP_ERROR_CHECK(esp_timer_create(&periodic_pwm_timer_args, &auto_pwm_timer));
    // // ESP_LOGI(TAG, "Start motor speed loop");
    // ESP_ERROR_CHECK(esp_timer_start_periodic(auto_pwm_timer, PWM_PERIOD_MS * 1000));

    //TODO Later use this function for an actually timer and move creating the queue to the flow_TASK
}

void Speed2Duty(uint8_t x){
    uint32_t d = 0;
    float s = 0;
    const uint32_t pwm_res = (1U << PWM_DUTY_RES); // 2^resolution

    if(x > 0 && x <= 100){      // TODO - ADD BETTER EDGE CASES
        s = ((((float)x/100)*0.72)+0.13);
        d = (uint32_t) ((((float)x/100*0.72)+0.13)*pwm_res);
    }
    ESP_LOGI(TAG,"Writing Duty: %d, Calculated: %f, Input: %d",d,s,x);
    if(curr_spd == 0 && x != 0){
        ESP_ERROR_CHECK(ledc_set_duty(PWM_SPD_MODE,PWM_CH,pwm_res-1));
        ESP_ERROR_CHECK(ledc_update_duty(PWM_SPD_MODE,PWM_CH));
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    curr_spd = x;
    ESP_ERROR_CHECK(ledc_set_duty(PWM_SPD_MODE,PWM_CH,d));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_SPD_MODE,PWM_CH));
}

void parse_cmd(char * cmd){

    ESP_LOGI(TAG,"Echo: %s",cmd);

    char * tok = strtok(cmd," ");
    if(tok != NULL){
        if(strcmp(tok,"mode") == 0){ // command "MODE"
            tok = strtok(NULL," ");
            int modeN;
            if(tok != NULL){
                modeN = atoi(tok);
            } else {
                modeN = 0;
            }
            if (modeN < PWM_NUM_MODES && modeN >= 0){
                mode = modeN;
                ESP_LOGI(TAG,"New Mode: %s",mode_label[mode]);
                if(modeN == 0){
                    target_spd = 0;
                    Speed2Duty(target_spd);
                }
            } else {
                ESP_LOGI(TAG,"Invalid Mode");
            }

        } else if (strcmp(tok,"flow") == 0){ // command "FLOW"
            tok = strtok(NULL," ");
            int speedN;
            if(tok != NULL){
                speedN = atoi(tok);
            } else {
                speedN = 0;
            }
            if (speedN <= MAX_FLOW && speedN >= 0){
                target_spd = speedN;
                ESP_LOGI(TAG,"New Flow: %d",target_spd);
                if(mode == 2){
                    Speed2Duty(target_spd);
                }
            } else {
                ESP_LOGI(TAG,"Invalid Flow Rate");
            }

        }else if(strcmp(tok,"data") == 0){ // command "DATA"
            if(mountState){
                tok = strtok(NULL," ");
                int dataN;
                if(tok != NULL){
                    dataN = atoi(tok);
                } else {
                    dataN = 0;
                }
                if(dataN < DATA_NUM_MODES && dataN >=0){
                    if(dataN != dataWrite){
                        switch (dataN)
                        {
                            case 0:
                                fclose(f);
                                break;

                            case 1:
                                f = fopen("/sdcard/test.txt","w");
                                if(f == NULL){
                                    ESP_LOGE(TAG,"Failed to open data file");
                                    dataN = 0;
                                }
                                break;
                            
                            default:
                                break;
                        }
                        dataWrite = dataN;
                    }
                    ESP_LOGI(TAG,"Data Write: %s",data_label[dataWrite]);
                }
            } else {
                ESP_LOGE(TAG,"No mounted file system for data saving");
            }

        } else {        // OTHERWISE
            ESP_LOGI(TAG,"Unrecognized Command: \"%s\"",tok);
        }
    } else {
        ESP_LOGI(TAG,"Unrecognized Command: \"%s\"",tok);
    }
}

void read_input_TASK(void * pntr){

    uart_event_t input_event;
    uint8_t input_data[UART_BUFF_SZ];
    static char input_buff[256];
    static int buff_index = 0;

    while(true){
        if(xQueueReceive(UART_event_q, (void *) &input_event, portMAX_DELAY)){
            switch(input_event.type){
                case UART_DATA:
                    int len = uart_read_bytes(UART_PORT, input_data, input_event.size, portMAX_DELAY);
                    for(int i = 0; i < len; i++){
                        char c = input_data[i];
                        if ((buff_index == sizeof(input_buff) - 1 || c == '\n' || c == '\r') && buff_index > 0){
                            input_buff[buff_index] = '\0';
                            parse_cmd(input_buff);
                            buff_index = 0;
                            break;
                        }
                        else if (buff_index < sizeof(input_buff) - 1){
                            input_buff[buff_index++] = c;
                        }
                    }
                    break;
                default:
                    break;
            }
        }
    }
}

void update_disp_TASK(void * pntr){
    while(true){
        uint8_t local_tSpd = target_spd;
        uint8_t local_cSpd = curr_spd;
        
        portENTER_CRITICAL(&flow_spinlock);
        float local_flow = flowRate;
        float local_freq = flowFreq;
        portEXIT_CRITICAL(&flow_spinlock);
        lvgl_port_lock(0);
        lv_label_set_text_fmt(init_Label,
            "Mode: %s\nData Write: %s\nTarget Motor Speed: %d%%\nCommanded Motor Speed: %d%%\n\nCurrent Flow Rate: %.2f\nCurrent Freq: %.2f",
            mode_label[mode],data_label[dataWrite],local_tSpd,local_cSpd,local_flow,local_freq);
        lvgl_port_unlock();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // lv_obj_set_style_text_font(init_Label, &lv_font_montserrat_20, LV_PART_MAIN);
    // lv_obj_set_pos(init_Label, 0, 0);
}

/* Function flow_TASK()
    fr_pntr - void pointer the the typedef holding the buffers and control data for the flow meter*/
void flow_TASK(void * fr_pntr){

    flow_data_t* local_meter = (flow_data_t*) fr_pntr;          //dereference the void pointer to access the values within the typedef
    volatile float* pRate = local_meter->pntrRate;              //create a local pointer to the rate value for display
    volatile float* pFreq = local_meter->pntrFreq;              //create a local pointer to the freqency value for display
    volatile TickType_t* flow_buff = local_meter->flowTicks;    //create a local pointer to the buffer that holds the tick values returned by the ISR
    for(int i = 0; i < FLOW_BUFF_SIZE;i++){
        flow_buff[i] = FLOW_DEFAULT_BUFFER_VAL;                 //Iterate thru the buffer and initiallize all the values to ignore irregular initial values
    }

    isr_time_data_t isr_time = {
        .period = 0,
        .timestamp = 0,
    };                                                          //define a typedef to hold the recieved period and timestamp from the ISR
    TickType_t local_tNow = 0;                                  //define a local value for current tick count
    TickType_t local_tLast = xTaskGetTickCount();               //define a second local value for tick count to compare the last one to
    int* local_Idx = &(local_meter->buffIdx);                   //define a local pointer for the number of new measurements within the buffer
    float new_rate;                                             //define a local value to hold calculated rate
    float new_freq;                                             //define a local value to hold calculated frequency

    while(true){        // << -- ENTER FREE-RUNNING LOOP -- >>
        if(xQueueReceive(flow_event_q,&isr_time,pdMS_TO_TICKS(FLOW_PULSE_TIMEOUT))){    //Tell the loop to defer execution until the flow_event_q receives a value (from the flow ISR) or it hits its timeout value
            flow_buff[flow_tar++] = isr_time.period;        //add the period sent from ISR into the buffer of values
            flow_tar %= FLOW_BUFF_SIZE;                     //move to the next pointiton in the buffer, at the end loop back over the preivous entries
            float flow_time = 0;                            //float to hold the mean period of all values in the buffer flow_buff
            for(int i = 0; i < FLOW_BUFF_SIZE;i++){
                flow_time += pdTICKS_TO_MS(flow_buff[i]);   //interate thru every value in the buffer and add them
            }
            flow_time /= ((float)FLOW_BUFF_SIZE*1000.0);    //divide the sum by the total number of values

            //ESP_LOGI(TAG,"Running Flow: %f",flow_time);
            if (flow_time > 0){
                new_rate = 1.0/(flow_time*FLOW_K_VALUE);
                new_freq = 1.0/flow_time;
            } else {
                new_rate = 0;
                new_freq = 0;
            }
        } else {    //ON TIMEOUT
            new_rate = 0;
            new_freq = 0;
            isr_time.timestamp = xTaskGetTickCount();
        }
        local_meter->buffRate[*local_Idx] = new_rate;
        local_meter->buffFreq[*local_Idx] = new_freq;
        local_meter->buffTimestamp[*local_Idx] = isr_time.timestamp;
        *local_Idx = (*local_Idx)+1;
        if(*local_Idx > FLOW_OUTPUT_BUFF_SIZE){
            ESP_LOGW(TAG,"FLOW OUTPUT OVERFLOW!");
            *local_Idx = *local_Idx % FLOW_OUTPUT_BUFF_SIZE;
        }
        local_tNow = xTaskGetTickCount();
        if((local_tNow-local_tLast) > pdMS_TO_TICKS(FLOW_OUTPUT_PERIOD_MS)){
            ESP_LOGI(TAG,"TS %lu",local_tNow);
            #if SERIAL_OUTPUT
                char OUT_STR[26*FLOW_OUTPUT_BUFF_SIZE+1] = "\0";
                char LOOP_STR[26] = "\0";
                for(int i = 0; i < *local_Idx; i++){
                    sprintf(LOOP_STR,"/*%3.2f, %3.2f, %3lu*/\n",local_meter->buffRate[i],local_meter->buffFreq[i],local_meter->buffTimestamp[i]);
                    strcat(OUT_STR,LOOP_STR);
                }
                ESP_LOGI(TAG,"%s",OUT_STR);
            #endif
            if(dataWrite == 1){
                fprintf(f,"%s",OUT_STR);
            }
            *local_Idx = 0;
            portENTER_CRITICAL(&flow_spinlock);
            *pRate = new_rate;
            *pFreq = new_freq;
            portEXIT_CRITICAL(&flow_spinlock);
            local_tLast = xTaskGetTickCount();
        }
    }
    

}

/*Function - heartbeat_TASK()
    pntr - unused
a function to alternate the state of a LED to show that the Controller is operational*/
void heartbeat_TASK(void * pntr){

    bool LEDstate = 0;  //store the state of the LED
    while (true){           // Loop forever
        LEDstate = !LEDstate;               //invert the state
        gpio_set_level(GPIO_LED,LEDstate);  //apply the new state
        vTaskDelay(pdMS_TO_TICKS(500));     //wait half a second
    }
}

void app_main(void){

    esp_log_level_set("*", ESP_LOG_DEBUG);        // Everything
    esp_log_level_set("spi_master", ESP_LOG_DEBUG);
    esp_log_level_set("lcd_panel.io.spi", ESP_LOG_DEBUG);

    init_UART();
    init_GPIO();
    init_Display();
    init_PWM();
    init_DATA();

    xTaskCreate(read_input_TASK,"Read Input",4096,NULL,6,NULL);
    xTaskCreate(heartbeat_TASK,"LED Pulse",4096,NULL,3,NULL);
    xTaskCreate(update_disp_TASK,"LCD Display",4096,NULL,3,NULL);
    xTaskCreate(flow_TASK,"Calculate Flow",4096,(void*)&flow_meter,4,NULL);

    ESP_LOGI(TAG,"Starting Echo Chamber . . .\n");

}