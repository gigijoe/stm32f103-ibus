/**
  ******************************************************************************
  * @file    GPIO/IOToggle/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
//#include "stm32_eval.h"

#include <string.h>

#include "usart.h"
#include "delay.h"

static char *hextoa(uint8_t hex)
{
  static char a[3];
  uint8_t v = (hex & 0xf0) >> 4;
  switch(v) {
    case 10: a[0] = 'a';
      break;
    case 11: a[0] = 'b';
      break; 
    case 12: a[0] = 'c';
      break; 
    case 13: a[0] = 'd';
      break; 
    case 14: a[0] = 'e';
      break; 
    case 15: a[0] = 'f';
      break; 
    default: a[0] = (v + 0x30);
      break;
  }
  v = hex & 0xf;
  switch(v) {
    case 10: a[1] = 'a';
      break;
    case 11: a[1] = 'b';
      break; 
    case 12: a[1] = 'c';
      break; 
    case 13: a[1] = 'd';
      break; 
    case 14: a[1] = 'e';
      break; 
    case 15: a[1] = 'f';
      break; 
    default: a[1] = (v + 0x30);
      break;
  }
  a[2] = '\0';
  return a;
}

static uint8_t atohex8(char *s)
{
  uint8_t value = 0;
  if(!s)
    return 0;

  if(*s >= '0' && *s <= '9')
    value = (*s - '0') << 4;
  else if(*s >= 'A' && *s <= 'F')
    value = ((*s - 'A') + 10) << 4;
  else if(*s >= 'a' && *s <= 'f')
    value = ((*s - 'a') + 10) << 4;

  s++;

  if(*s >= '0' && *s <= '9')
    value |= (*s - '0');
  else if(*s >= 'A' && *s <= 'F')
    value |= ((*s - 'A') + 10);
  else if(*s >= 'a' && *s <= 'f')
    value |= ((*s - 'a') + 10);

  return value;
}

static char *ibus_device_name(uint8_t id) 
{
  switch(id) {
    case 0xff:
    case 0x00: return "Broadcast";
    case 0x08: return "SHD Sunroof Control";
    case 0x18: return "CDC CD-Player";
    case 0x24: return "HKM Tailgate lift";
    case 0x28: return "FUH Radio controlled clock";
    case 0x30: return "CCM Check control module";
    case 0x3b: return "NAV Navigation/Videomodule";
    case 0x3f: return "DIA Diagnostic";
    case 0x40: return "FBZV Remote control central locking";
    case 0x43: return "MenuScreen";
    case 0x44: return "EWS Electronic immobiliser";
    case 0x46: return "CID Central information display";
    case 0x50: return "MFL Multi Functional Steering Wheel Buttons";
    case 0x51: return "MM0 Mirror memory";
    case 0x5b: return "IHK Integrated heating and air conditioning";
    case 0x60: return "PDC Park Distance Control";
    case 0x68: return "RAD Radio";
    case 0x6A: return "DSP Digital Sound Processor";
    case 0x70: return "RDC Tyre pressure control";
    case 0x72: return "SM0 Seat memory";
    case 0x73: return "SDRS Sirius Radio";
    case 0x76: return "CDCD CD changer, DIN size";
    case 0x7f: return "Navigation (Europe)";
    case 0x80: return "IKE Instrument Control Electronics";
    case 0x9b: return "MM1 Mirror memory";
    case 0x9c: return "MM2 Mirror memory";
    case 0xa0: return "FMID Rear multi-info-display";
    case 0xa4: return "ABM Air bag module";
    case 0xac: return "EHC Electronic height control";
    case 0xb0: return "SES Speed recognition system";
    case 0xbb: return "Navigation (Japan)";
    case 0xbf: return "GLO Global, broadcast address";
    case 0xc0: return "MID Multi-Information Display Buttons";
    case 0xc8: return "TEL Telephone";
    case 0xd0: return "LCM Light control module";
    case 0xe0: return "IRIS Integrated radio information system";
    case 0xe7: return "ANZV OBC TextBar";
    case 0xed: return "TV Television";
    case 0xf0: return "BMB Board Monitor Buttons";
    default: return "Unknown";
  }
}

static uint8_t XOR_Checksum(uint8_t *buf, uint16_t len)
{
  uint8_t checksum = 0;
  uint16_t i;
  for(i=0;i<len;i++)
    checksum = checksum ^ buf[i];
  return checksum;
}

#define MAX_CMD_LEN 128

typedef struct {
  char data[MAX_CMD_LEN + 1];
  int len;
} Shell;

void Shell_Input(Shell *s, char c)
{
  if(s->len < (MAX_CMD_LEN - 1)) {
    if(c == 0x8) { /* backspace */
      s->data[--s->len] = '\0';
      return;
    } else if(c < 0x20)
      return;

    s->data[s->len++] = c;
    s->data[s->len] = '\0';
  }
}

char *Shell_InputString(Shell *s)
{
  return s->len > 0 ? s->data : 0;
}

void Shell_InputReset(Shell *s)
{
  memset(s->data, 0, MAX_CMD_LEN + 1);
  s->len = 0;
}

#define MAX_ID_COUNT 32
static uint8_t srcIdCount = 0;
static uint8_t srcId[MAX_ID_COUNT];
static uint8_t destIdCount = 0;
static uint8_t destId[MAX_ID_COUNT];
typedef enum {ibusStop, ibusRecv} IBusState;
static IBusState state = ibusStop;

int IBus_Help(void)
{
  Usart2_Puts("\r\nIBus Inspector\r\n");

  Usart2_Puts("\r\n src [dev id 0] [dev id 1] ... [dev id n]");
  Usart2_Puts("\r\n dest [dev id 0] [dev id 1] ... [dev id n]");
  Usart2_Puts("\r\n recv");
  Usart2_Puts("\r\n stop");
  Usart2_Puts("\r\n send <src id> <dest id> <XX XX XX ...>");
  Usart2_Puts("\r\n send raw <XX XX XX ...>");

  return 0;
}

int IBus_SetupSource(int argc, char *argv[])
{
  int i;
  srcIdCount = 0;
  memset(srcId, 0, MAX_ID_COUNT);
  for(i=1;i<argc;i++) {
    srcId[i-1] = atohex8(argv[i]);
    if(++srcIdCount >= MAX_ID_COUNT)
      break;
  }
  return 0;
}

int IBus_SetupDestination(int argc, char *argv[])
{
  int i;
  destIdCount = 0;
  memset(destId, 0, MAX_ID_COUNT);
  for(i=1;i<argc;i++) {
    destId[i-1] = atohex8(argv[i]);
    if(++destIdCount >= MAX_ID_COUNT)
      break;
  }
  return 0;
}

int IBus_StartReceive(int argc, char *argv[])
{
  state = ibusRecv;
  return 0;
}

int IBus_StopReceive(int argc, char *argv[])
{
  state = ibusStop;
  return 0;
}

int IBus_Send(int argc, char *argv[])
{
  if(argc <= 4)
    return -1; /* Not enough data */
  uint8_t code[MAX_TX_LEN];
  code[0] = atohex8(argv[1]); /* src */
  code[2] = atohex8(argv[2]); /* dest */
  int i, len = 1;
  for(i=3;i<argc;i++) {
    code[i] = atohex8(argv[i]); /* data */
    len++;
  }
  code[1] = len + 1;
  code[i] = XOR_Checksum((uint8_t *)&code[0], len + 2);

  Usart2_Puts("\r\n");
  Usart2_Printf("Source : %s\r\n", ibus_device_name(code[0]));
  Usart2_Printf("Length : %d\r\n", code[1]);
  Usart2_Printf("Destination : %s\r\n", ibus_device_name(code[2]));
  Usart2_Printf("CRC : %s\r\n", hextoa(code[i])); /* All exclude CRC itself */

  Usart3_Write(&code[0], len + 2);

  return 0;
}

int IBus_SendRaw(int argc, char *argv[])
{
  if(argc <= 6)
    return -1; /* Not enough data */
  uint8_t code[MAX_TX_LEN];
  int i, len = 0;
  for(i=2;i<argc;i++)
    code[len++] = atohex8(argv[i]); /* data */

  Usart2_Puts("\r\n");
  Usart2_Printf("Source : %s\r\n", ibus_device_name(code[0]));
  Usart2_Printf("Length : %d\r\n", code[1]);
  Usart2_Printf("Destination : %s\r\n", ibus_device_name(code[2]));
  Usart2_Printf("CRC : %s\r\n", hextoa(code[len-1])); /* All exclude CRC itself */

  Usart3_Write(&code[0], len);

  return 0;
}

int IBus_Send2(uint8_t src, uint8_t dest, uint8_t *data, uint8_t dataLen) 
{
  uint8_t code[MAX_TX_LEN];

  code[0] = src; /* src */
  /* code[1] : The length of the packet whithout Source ID and length it-self. */
  code[2] = dest; /* dest */
  int i, len = 1; /* dest length is 1 */
  for(i=0;i<dataLen;i++) {
    code[i+3] = data[i]; /* data */
    len++;
  }
  len++; /* check sum length is 1 */
  code[1] = len; 
  code[i+3] = XOR_Checksum((uint8_t *)&code[0], len + 1);
#if 0
  Usart2_Puts("\r\n");
  Usart2_Printf("Source : %s\r\n", ibus_device_name(code[0]));
  Usart2_Printf("Length : 0x%x\r\n", code[1]);
  Usart2_Printf("Destination : %s\r\n", ibus_device_name(code[2]));
  Usart2_Printf("CRC : %s\r\n", hextoa(code[i+3])); /* All exclude CRC itself */
#endif
  Usart3_Write(&code[0], len + 2);

  return 0;
}

IBusState IBus_State() { return state; }

uint8_t IBus_ValidSource(uint8_t id)
{
  int i;
  if(srcIdCount == 0)
    return id;
  for(i=0;i<srcIdCount;i++)
    if(srcId[i] == id)
      return id;
  return 0;
}

uint8_t IBus_ValidDestination(uint8_t id)
{
  int i;
  if(destIdCount == 0)
    return id;
  for(i=0;i<destIdCount;i++)
    if(destId[i] == id)
      return id;
  return 0;
}

int Shell_Run(Shell *s)
{
  if(s->len == 0)
    return -1;

  int ret = -1;

  char *p = 0;
#define MAX_ARGC 32 
  char *argv[MAX_ARGC];
  uint16_t argc = 0;

  int i;
  for(i=(s->len-1);i>=0;i--) {
    if(s->data[i] <= 0x20 || s->data[i] > 0x7e) {
      s->data[i] = '\0'; /* Strip back */
      s->len--;
    } else
      break;
  }

  for(i=0;i<s->len;i++) {
    if(s->data[i] > 0x20 && s->data[i] <= 0x7e)
      break; 
    s->data[i] = '\0'; /* Strip front */
    s->len--;
  }

  p = &s->data[i];
  i = 0;
  while(i < s->len) {
    if(p[i] == 0x20) {
      p[i++] = '\0';
      continue;
    }
    argv[argc++] = &p[i++];
    while(p[i] > 0x20 && p[i] <= 0x7e)
      i++;
    p[i++] = '\0'; /* end string */
    if(argc >= MAX_ARGC)
      break;
  }
#if 0
  for(i=0;i<argc;i++) {
    Usart2_Printf("\r\nargv[%d] = %s", i, argv[i]); 
  }
#endif
  if(strcmp("help", argv[0]) == 0)
    ret = IBus_Help();
  else if(strcmp("src", argv[0]) == 0)
    ret = IBus_SetupSource(argc, argv);
  else if(strcmp("dest", argv[0]) == 0)
    ret = IBus_SetupDestination(argc, argv);
  else if(strcmp("recv", argv[0]) == 0)
    ret = IBus_StartReceive(argc, argv);
  else if(strcmp("stop", argv[0]) == 0)
    ret = IBus_StopReceive(argc, argv);
  else if(strcmp("send", argv[0]) == 0) {
    if(argc > 2 && strcmp("raw", argv[1]) == 0)
      ret = IBus_SendRaw(argc, argv);
    else
      ret = IBus_Send(argc, argv);
  }

  Shell_InputReset(s);

  return ret;
}

static Shell shell;

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*
*
*/

static uint32_t tim4Tick = 0;

void Tim4_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* NVIC_PriorityGroup */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  //基础设置，时基和比较输出设置，由于这里只需定时，所以不用OC比较输出
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
  
  TIM_DeInit(TIM4);

  TIM_TimeBaseStructure.TIM_Period = 1000;//装载值
  //prescaler is 72, that is 72000000/72/1000 = 1000Hz;
  TIM_TimeBaseStructure.TIM_Prescaler = 71;//分频系数
  //set clock division 
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
  //count up
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  //clear the TIM4 overflow interrupt flag
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  //TIM4 overflow interrupt enable
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  //enable TIM4
  TIM_Cmd(TIM4, DISABLE);
}

void Tim4_Enable(void)
{
  TIM_Cmd(TIM4, ENABLE);
}

static uint32_t tim4Tick_1ms = 0;

void Tim4_1ms(void)
{
}

static uint32_t tim4Tick_10ms = 0;

void Tim4_10ms(void)
{
}

static uint32_t tim4Tick_50ms = 0;

void Tim4_50ms(void)
{  
/*  
  uint8_t d2[] = { 0x01 };
  IBus_Send2(0xc0, 0x68, d2, 1);
*/
}

static uint32_t tim4Tick_100ms = 0;

void Tim4_100ms(void)
{
}

static uint32_t tim4Tick_200ms = 0;

void Tim4_200ms(void)
{
/*
*/
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  DelayInit();
#if 0
  Usart3_Init(115200);
  Usart3_Puts("\r\nIBus Inspector v0.0.1");
  for(;;) {
    int len = Usart3_Poll();
      if(len > 0) 
        Usart3_Write(Usart3_Gets(), len);
  }
#endif
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOC clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOC, GPIO_Pin_0); /* pull low */
  GPIO_ResetBits(GPIOC, GPIO_Pin_1); /* pull low */
  GPIO_ResetBits(GPIOC, GPIO_Pin_2); /* pull low */
  GPIO_ResetBits(GPIOC, GPIO_Pin_3); /* pull low */

  Tim4_Init();
  Tim4_Enable();

  Usart2_Init(115200);
  Usart2_Puts("\r\nIBus Inspector v0.0.1");
  Usart2_Puts("\r\nAuthor : Steve Chang");
  Usart2_Puts("\r\n26th October 2016");

  Usart2_Puts("\r\nIBus\\> ");

  Usart3_Init(9600);

  /* GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; /* NSLP */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOB, GPIO_Pin_12); /* pull low */
  DelayMs(5); 
  GPIO_SetBits(GPIOB, GPIO_Pin_12); /* pull high to leave sleep mode */

  uint32_t tick_1ms = tim4Tick_1ms;
  uint32_t tick_10ms = tim4Tick_10ms;
  uint32_t tick_50ms = tim4Tick_50ms;
  uint32_t tick_100ms = tim4Tick_100ms;
  uint32_t tick_200ms = tim4Tick_200ms;

  for(;;) {
    if(tick_1ms != tim4Tick_1ms) {
      tick_1ms = tim4Tick_1ms;
      Tim4_1ms();
    }

    if(tick_10ms != tim4Tick_10ms) {
      tick_10ms = tim4Tick_10ms;
      Tim4_10ms();
    }

    if(tick_50ms != tim4Tick_50ms) {
      tick_50ms = tim4Tick_50ms;
      Tim4_50ms();
    }

    if(tick_100ms != tim4Tick_100ms) {
      tick_100ms = tim4Tick_100ms;
      Tim4_100ms();
    }

    if(tick_200ms != tim4Tick_200ms) {
      tick_200ms = tim4Tick_200ms;
      Tim4_200ms();
    }

    int len = Usart2_Poll();
    if(len > 0) {
      char *p = Usart2_Gets();
      int i;
      for(i=0;i<len;i++) {
        if(p[i] == 0xd) { /* CR */
          char *sc = Shell_InputString(&shell);
          if(sc)
            if(Shell_Run(&shell) == -1)
              Usart2_Puts("\r\nIllegal command\r\nIBus\\> ");
            else
              Usart2_Puts("\r\nIBus\\> ");
          else
            Usart2_Puts("\r\nIBus\\> ");
        } else if((p[i] >= 0x20 && p[i] <= 0x7e) || p[i] == 0x8) {
          if(p[i] == 0x8) /*backspace */
            Usart2_Puts("\b \b"); /* backspace + space + backspace */
          else 
            Usart2_Write((uint8_t *)&p[i], 1);
          Shell_Input(&shell, p[i]);
        }
      }
    }

    len = Usart3_Poll();
  /*    
      if(len > 0) 
        Usart2_Write(Usart3_Gets(), len);
  */
    if(IBus_State() == ibusStop) {
      Usart3_Gets(); /* Flush data*/
      continue;
    }

    if(len > 0) {
      char *p = Usart3_Gets();

      if(p[0] != IBus_ValidSource(p[0]) ||
        p[2] != IBus_ValidDestination(p[2]))
        continue;

      Usart2_Puts("\r\n");
#if 0
      Usart2_Printf("Source : %s\r\n", ibus_device_name(p[0]));
      Usart2_Printf("Length : %d\r\n", p[1]);
      Usart2_Printf("Destination : %s\r\n", ibus_device_name(p[2]));
      Usart2_Printf("CRC : %s\r\n", hextoa(XOR_Checksum((uint8_t *)&p[0], len - 1))); /* All exclude CRC itself */

      int i;
      for(i=0;i<len;i++) {
        if(p[i] < 0x20 || p[i] > 0x7e)
          Usart2_Write((uint8_t *)".", 1);
        else
          Usart2_Write((uint8_t *)&p[i], 1);
        if(i != 0 && i % 20 == 0)
          Usart2_Puts("\r\n");
      }
      Usart2_Puts("\r\n");
      for(i=0;i<len;i++) {
        Usart2_Write((uint8_t *)hextoa(p[i]), 2);
        Usart2_Write((uint8_t *)" ", 1);
        if(i != 0 && i % 20 == 0)
          Usart2_Puts("\r\n");
      }
      Usart2_Puts("\r\n");
#else
      Usart2_Printf("%s : %s\r\n", hextoa(p[0]), ibus_device_name(p[0]));
      Usart2_Printf("%s : %s\r\n", hextoa(p[2]), ibus_device_name(p[2]));
      int i;
      for(i=0;i<len;i++) {
        if(p[i] < 0x20 || p[i] > 0x7e)
          Usart2_Write((uint8_t *)".", 1);
        else
          Usart2_Write((uint8_t *)&p[i], 1);
#if 0        
        if(i != 0 && i % 20 == 0)
          Usart2_Puts("\r\n");
#endif          
      }
      Usart2_Puts("\r\n");
      for(i=0;i<len;i++) {
        Usart2_Write((uint8_t *)hextoa(p[i]), 2);
        Usart2_Write((uint8_t *)" ", 1);
#if 0
        if(i != 0 && i % 20 == 0)
          Usart2_Puts("\r\n");
#endif
        if(i == 2 || i == (len - 2))
          Usart2_Write((uint8_t *)"| ", 2);
      }
      Usart2_Puts("\r\n");
#endif      
    }
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

void TIM4_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
    tim4Tick++;
    tim4Tick_1ms++;
    if(tim4Tick % 10 == 0)
      tim4Tick_10ms++;
    if(tim4Tick % 50 == 0)
      tim4Tick_50ms++;
    if(tim4Tick % 100 == 0)
      tim4Tick_100ms++;
    if(tim4Tick % 200 == 0)
      tim4Tick_200ms++;
    //
    // 清除 TIM2
    TIM_ClearITPendingBit(TIM4, /*TIM_IT_Update*/ TIM_FLAG_Update);
  }
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
