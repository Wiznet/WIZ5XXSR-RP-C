#include "port_common.h"

#include "common.h"
#include "WIZ5XXSR-RP_board.h"
#include "ConfigData.h"
#include "gpioHandler.h"
#include "wizchip_conf.h"

#ifdef __USE_USERS_GPIO__
  const uint16_t USER_IO_PIN[USER_IOn] =     {USER_IO_A_PIN, USER_IO_B_PIN};
  uint8_t     USER_IO_ADC_CH[USER_IOn] =     {USER_IO_A_ADC_CH, USER_IO_B_ADC_CH};
  uint8_t        USER_IO_SEL[USER_IOn] =     {USER_IO_A, USER_IO_B};
  const char*    USER_IO_STR[USER_IOn] =     {"a", "b", "c", "d"};
  const char*    USER_IO_PIN_STR[USER_IOn] = {"GPIO26\0", "GPIO27\0"};
  const char*    USER_IO_TYPE_STR[] =        {"Digital", "Analog"};
  const char*    USER_IO_DIR_STR[] =         {"Input", "Output"};
#endif


/**
  * @brief  RP2040 GPIO Initialize Function
  */
 
void GPIO_Configuration(uint16_t GPIO_Pin, USER_IO_Direction GPIO_Mode, USER_IO_PULL GPIO_Pull)
{
	  /*Configure GPIO pin */
    gpio_init(GPIO_Pin);

    if (GPIO_Mode == IO_INPUT)
        gpio_set_dir(GPIO_Pin, GPIO_IN);
    else
        gpio_set_dir(GPIO_Pin, GPIO_OUT);

    if (GPIO_Pull == IO_PULLUP)
        gpio_pull_up(GPIO_Pin);
    else if(GPIO_Pull == IO_PULLDOWN)
        gpio_pull_down(GPIO_Pin);
}


void GPIO_Output_Set(uint16_t GPIO_Pin)
{
    gpio_put(GPIO_Pin, IO_HIGH);
}


void GPIO_Output_Reset(uint16_t GPIO_Pin)
{
    gpio_put(GPIO_Pin, IO_LOW);
}

uint8_t GPIO_Output_Toggle(uint16_t GPIO_Pin)
{
    uint8_t pin_state;
    pin_state = gpio_get(GPIO_Pin);
    gpio_put(GPIO_Pin, !pin_state);
}

uint8_t GPIO_Input_Read(uint16_t GPIO_Pin)
{
    return gpio_get(GPIO_Pin);
}

/**
  * @brief  Device I/O Initialize Function
  */
void Device_IO_Init(void)
{
    struct __serial_option *serial_option = (struct __serial_option *)&(get_DevConfig_pointer()->serial_option);
    // Set the DTR pin to high when the DTR signal enabled (== PHY link status disabled)

    if(serial_option->dtr_en == 1)
    {
        init_flowcontrol_dtr_pin();
        set_flowcontrol_dtr_pin(ON);
    }

    if(serial_option->dsr_en == 1)
        init_flowcontrol_dsr_pin();
}

// This function is intended only for output connection status pins; PHYlink, TCPconnection
void set_connection_status_io(uint16_t pin, uint8_t set)
{
    struct __serial_option *serial_option = (struct __serial_option *)&(get_DevConfig_pointer()->serial_option);

    if(pin == STATUS_PHYLINK_PIN)
    {
        //PRT_INFO("pin = PHY, set = %d\r\n", set);
        if(set == ON)
            GPIO_Output_Set(STATUS_PHYLINK_PIN);
        else // OFF
            GPIO_Output_Reset(STATUS_PHYLINK_PIN);
    }
    else if(pin == STATUS_TCPCONNECT_PIN)
    {
        if(set == ON)
            GPIO_Output_Set(STATUS_TCPCONNECT_PIN);
        else // OFF            
            GPIO_Output_Reset(STATUS_TCPCONNECT_PIN);
    }
    
}

uint8_t get_connection_status_io(uint16_t pin)
{
    uint8_t status = IO_LOW;

    if(pin == STATUS_PHYLINK_PIN)
    {
        if(get_phylink() == PHY_LINK_ON)
            status = IO_HIGH;
    }
    else if(pin == STATUS_TCPCONNECT_PIN)
    {
        if(get_device_status() == ST_CONNECT)
            status = IO_HIGH;
    }    
    return status;
}

// PHY link status pin
void init_phylink_status_pin(void)
{
    GPIO_Configuration(STATUS_PHYLINK_PIN, IO_OUTPUT, IO_NOPULL);
    // Pin initial state; Low
    GPIO_Output_Reset(STATUS_PHYLINK_PIN);
}

// DTR pin
// output
void init_flowcontrol_dtr_pin(void)
{
    GPIO_Configuration(DTR_PIN, IO_OUTPUT, IO_NOPULL);
    GPIO_Output_Reset(DTR_PIN);
}

void set_flowcontrol_dtr_pin(uint8_t set)
{
    if(set == ON)
        GPIO_Output_Set(DTR_PIN);
    else
        GPIO_Output_Reset(DTR_PIN);
}

// DSR pin
// input, active high
void init_flowcontrol_dsr_pin(void)
{
    GPIO_Configuration(DSR_PIN, IO_INPUT, IO_NOPULL);
}

uint8_t get_flowcontrol_dsr_pin(void)
{
    return GPIO_Input_Read(DSR_PIN);
}

void init_connection_status_io(void)
{
    struct __serial_option *serial_option = (struct __serial_option *)&(get_DevConfig_pointer()->serial_option);

    init_phylink_status_pin();
    init_tcpconnection_status_pin();
}

// Check the PHY link status
uint8_t check_phylink_status(void)
{
    static uint8_t prev_link_status;
    uint8_t link_status;

    link_status = get_phylink();

    //PRT_INFO("link_status = %d\r\n", link_status);
    
    if(prev_link_status != link_status)
    {
        if(link_status == PHY_LINK_ON)
            set_connection_status_io(STATUS_PHYLINK_PIN, ON); 	// PHY Link up
        else
            set_connection_status_io(STATUS_PHYLINK_PIN, OFF); 	// PHY Link down
        prev_link_status = link_status;
    }
    return link_status;
}

// This function have to call every 1 millisecond by Timer IRQ handler routine.
void gpio_handler_timer_msec(void)
{
    // PHY link check
    if(++phylink_check_time_msec >= PHYLINK_CHECK_CYCLE_MSEC)
    {
        phylink_check_time_msec = 0;
        flag_check_phylink = 1;
    }
}

uint8_t factory_flag;
void factory_reset_pin_callback(uint32_t gpio, uint32_t events)
{
    PRT_INFO("interrupt occured\r\n");
    factory_flag = 1;
}

#ifdef __USE_USERS_GPIO__

void init_user_io(uint8_t io_sel)
{
  struct __user_io_info *user_io_info = (struct __user_io_info *)&(get_DevConfig_pointer()->user_io_info);
  uint8_t idx = 0;
  
  uint8_t io_status = 0;
  
  if((user_io_info->user_io_enable & io_sel) == io_sel)
  {
    idx = get_user_io_bitorder(io_sel);
    
    if((user_io_info->user_io_type & io_sel) == io_sel) // IO_ANALOG_IN == 1 
    {
      if(USER_IO_ADC_CH[idx] != USER_IO_NO_ADC)
      {
        // Analog Input
        adc_init();
        adc_gpio_init(USER_IO_PIN[idx]);
      }
      //else // IO init falied
    }
    else
    {
      // Digital Input / Output
      if((user_io_info->user_io_direction & io_sel) == io_sel) // IO_OUTPUT == 1
      {
        GPIO_Configuration(USER_IO_PIN[idx], IO_OUTPUT, IO_NOPULL);
        io_status = ((user_io_info->user_io_status & (uint16_t)io_sel) == (uint16_t)io_sel)?IO_HIGH:IO_LOW;
        
        if(io_status == IO_HIGH)
          GPIO_Output_Set(USER_IO_PIN[idx]);
        else
          GPIO_Output_Reset(USER_IO_PIN[idx]);
      }
      else
        GPIO_Configuration(USER_IO_PIN[idx], IO_INPUT, IO_NOPULL);
    }
  }
}


uint8_t set_user_io_enable(uint8_t io_sel, uint8_t enable)
{
  struct __user_io_info *user_io_info = (struct __user_io_info *)&(get_DevConfig_pointer()->user_io_info);
  uint8_t ret = 1;
  
  
  if(enable == IO_ENABLE)
    user_io_info->user_io_enable |= io_sel;
  else if(enable == IO_DISABLE)
    user_io_info->user_io_enable &= ~(io_sel);
  else
    ret = 0;
  
  return ret;
}


uint8_t set_user_io_type(uint8_t io_sel, uint8_t type)
{
  struct __user_io_info *user_io_info = (struct __user_io_info *)&(get_DevConfig_pointer()->user_io_info);
  uint8_t ret = 1;
  uint8_t idx;
  
  if(type == IO_ANALOG_IN)
  {
    idx = get_user_io_bitorder(io_sel);
    
    if(USER_IO_ADC_CH[idx] != USER_IO_NO_ADC)
    {
      set_user_io_direction(io_sel, IO_INPUT); // Analog: input only
      user_io_info->user_io_type |= io_sel;
      init_user_io(io_sel); // IO reinitialize
    }
    else
      ret = 0;
  }
  else if(type == IO_DIGITAL)
  {
    user_io_info->user_io_type &= ~(io_sel);
    init_user_io(io_sel); // IO reinitialize
  }
  else
    ret = 0;
  
  init_user_io(io_sel);
  
  return ret;
}


uint8_t set_user_io_direction(uint8_t io_sel, uint8_t dir)
{
  struct __user_io_info *user_io_info = (struct __user_io_info *)&(get_DevConfig_pointer()->user_io_info);
  uint8_t ret = 1;
  
  
  if(dir == IO_OUTPUT)
  {
    user_io_info->user_io_direction |= io_sel;
    init_user_io(io_sel); // IO reinitialize
  }
  else if(dir == IO_INPUT)
  {
    user_io_info->user_io_direction &= ~(io_sel);
    init_user_io(io_sel); // IO reinitialize
  }
  else
    ret = 0;
  
  init_user_io(io_sel);
  
  return ret;
}


uint8_t get_user_io_enabled(uint8_t io_sel)
{
  struct __user_io_info *user_io_info = (struct __user_io_info *)&(get_DevConfig_pointer()->user_io_info);
  uint8_t ret;
  
  if((user_io_info->user_io_enable & io_sel) == io_sel)
    ret = IO_ENABLE;
  else
    ret = IO_DISABLE;
  
  return ret;
}


uint8_t get_user_io_type(uint8_t io_sel)
{
  struct __user_io_info *user_io_info = (struct __user_io_info *)&(get_DevConfig_pointer()->user_io_info);
  uint8_t ret;
  
  if((user_io_info->user_io_type & io_sel) == io_sel)
    ret = IO_ANALOG_IN;
  else
    ret = IO_DIGITAL;
  
  return ret;
}


uint8_t get_user_io_direction(uint8_t io_sel)
{
  struct __user_io_info *user_io_info = (struct __user_io_info *)&(get_DevConfig_pointer()->user_io_info);
  uint8_t ret;
  
  if((user_io_info->user_io_direction & io_sel) == io_sel)
    ret = IO_OUTPUT;
  else
    ret = IO_INPUT;
  
  return ret;
}

// Analog input:  Returns ADC value (12-bit resolution)
// Digital in/out:  Returns I/O status to match the bit ordering
uint8_t get_user_io_val(uint16_t io_sel, uint16_t * val)
{
  struct __user_io_info *user_io_info = (struct __user_io_info *)&(get_DevConfig_pointer()->user_io_info);
  
  uint8_t ret = 0; // I/O Read failed
  uint8_t idx = 0;
  uint8_t status = 0;
  
  *val = 0;
  
  if((user_io_info->user_io_enable & io_sel) == io_sel) // Enable
  {
    idx = get_user_io_bitorder(io_sel);
    
    if((user_io_info->user_io_type & io_sel) == io_sel) // IO_ANALOG == 1
    {
      if(USER_IO_ADC_CH[idx] != USER_IO_NO_ADC)       // Analog Input: value
        *val = read_ADC(USER_IO_ADC_CH[idx]);
      else        // IO value get failed
        *val = 0;
    }
    else // IO_DIGITAL == 0
    {
      status = GPIO_Input_Read(USER_IO_PIN[idx]);
      *val = status;
    }
    ret = 1; // I/O Read success
  }
  
  return ret;
}


uint8_t set_user_io_val(uint16_t io_sel, uint16_t * val)
{
  struct __user_io_info *user_io_info = (struct __user_io_info *)&(get_DevConfig_pointer()->user_io_info);
  uint8_t ret = 0; // I/O Read failed
  uint8_t idx = 0;
  
  if((user_io_info->user_io_enable & io_sel) == io_sel) // Enable
  {
    // Digital output only (type == 0 && direction == 1)
    if(((user_io_info->user_io_type & io_sel) == IO_DIGITAL) && ((user_io_info->user_io_direction & io_sel) == io_sel))
    {
      idx = get_user_io_bitorder(io_sel);
      
      if(*val == 0)
      {
        user_io_info->user_io_status &= ~(io_sel);
        GPIO_Output_Reset(USER_IO_PIN[idx]);
      }
      else if(*val == 1)
      {
        user_io_info->user_io_status |= io_sel;        
        GPIO_Output_Set(USER_IO_PIN[idx]);
      }
      ret = 1;
    }
  }
  
  return ret;
}


uint8_t get_user_io_bitorder(uint16_t io_sel)
{
  uint8_t i;
  uint8_t ret = 0;
  
  for(i = 0; i < USER_IOn; i++)
  {
    if((io_sel >> i) == 1)
    {
      ret = i;
      break;
    }
  }
    
  return ret;
}

// Read ADC val: 12-bit ADC resolution
uint16_t read_ADC(uint8_t ch)
{
  uint16_t adc_result;
  adc_select_input(ch);        ///< Select ADC channel to CH
  adc_result = adc_read();

  return adc_result;  ///< read ADC Data
}

#endif


