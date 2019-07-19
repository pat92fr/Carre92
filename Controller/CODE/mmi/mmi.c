/*
 * mmi.c
 *
 *  Created on: 22 avr. 2018
 *      Author: Patrick
 */

#ifndef MMI_C_
#define MMI_C_

#include "mmi.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

bool isNumeric(const char *str)
{
    while(*str != '\0')
    {
        if(*str < '0' || *str > '9')
            return false;
        str++;
    }
    return true;
}

HAL_MMI_Entry const * _entry;

HAL_Serial_Handler * _serial;

void HAL_MMI_Init(HAL_Serial_Handler * serial)
{
	_serial = serial;
	HAL_Serial_Print(_serial,"cmd>");
}

void HAL_MMI_Configure(HAL_MMI_Entry const * entry)
{
	_entry = entry;
}

#define STR_BUFFER_SIZE 24

enum
{
        MMI_CODE =0,
        MMI_VALUE
};

void HAL_MMI_Process()
{
	   static unsigned int state = 0;
	    static char code_str[STR_BUFFER_SIZE];
	    static unsigned int code_str_position = 0;
	    static char value_str[STR_BUFFER_SIZE];
	    static unsigned int value_str_position = 0;
	    static int code_index = -1;
	    if(HAL_Serial_Available(_serial))
	    {
			char c = HAL_Serial_GetChar(_serial);
			HAL_Serial_Print(_serial,"%c",c);
			switch(state)
			{
				case MMI_CODE :
					{
						//! wait for a valid code name
						if(c=='\r' || c=='\n')
						{
							//! null terminate string
							code_str[code_str_position]=0;
							//! check code name
							if(code_str_position==0)
							{
								//! code name empty
								HAL_Serial_Print(_serial,"cmd>");
							}
							else
							{
								//! find code name
								code_index = -1;
								unsigned int index = 0;
								while(_entry[index].code!=0)
								{
									if(strcmp(_entry[index].code,code_str)==0)
									{
										//! code name found
										code_index = index;
										break;
									}
									 ++index;
								}
								//! process code name
								if(code_index==-1)
								{
									//! code name unknwonw
									HAL_Serial_Print(_serial,"cmd>");
									code_str_position = 0;
								}
								else
								{
									//! kind of code
									switch(_entry[code_index].type)
									{
									case MMI_ENTRY_DATA:
										{
											//! found DATA code name
											HAL_Serial_Print(_serial,"\t%s(x%d)=%d\n",_entry[code_index].code,_entry[code_index].weight,(int32_t)(*_entry[code_index].callback.dptr*(float)_entry[code_index].weight));
											HAL_Serial_Print(_serial,"\t%s(x%d)=",_entry[code_index].code,_entry[code_index].weight);

											//! go to value input
											state = MMI_VALUE;
											value_str_position = 0;
										}
										break;
									case MMI_ENTRY_FUNCTION:
										{
											//! found FUNCTION code name
											HAL_Serial_Print(_serial,"\tcalling %s()...",_entry[code_index].code);
											(*_entry[code_index].callback.fptr)();
											//! go to code name input
											HAL_Serial_Print(_serial,"\ncmd>");
											state = MMI_CODE;
											code_str_position = 0;

										}
										break;
									case MMI_ENTRY_FUNCTION_1ARG:
										{
											//! found FUNCTION code name
											HAL_Serial_Print(_serial,"\targ=");
											//! go to arg input
											state = MMI_VALUE;
											value_str_position = 0;
										}
										break;
									}
								}
							}
						}
						else
						{
							code_str[code_str_position++]=c;
							if(code_str_position>=STR_BUFFER_SIZE)
								code_str_position = STR_BUFFER_SIZE-1;
						}
					}
					break;
				case MMI_VALUE :
					{
						//! wait for a valid value
						if(c=='\r' || c=='\n')
						{
							//! null terminate string
							 value_str[value_str_position]=0;
							//! check value
							if(value_str_position==0)
							{
								 switch(_entry[code_index].type)
								{
								case MMI_ENTRY_DATA:
									{
										//! value empty
										state = MMI_CODE;
										code_str_position = 0;
										HAL_Serial_Print(_serial,"\tdata change aborted.\ncmd>");
									}
									break;
								case MMI_ENTRY_FUNCTION:
									{
										//! return to safe state
										state = MMI_CODE;
										code_str_position = 0;
										printf("cmd>");
									}
									break;
								case MMI_ENTRY_FUNCTION_1ARG:
									{
										//! value empty
										state = MMI_CODE;
										code_str_position = 0;
										HAL_Serial_Print(_serial,"\tfunction call aborted\ncmd>");
									}
									break;
								}
							}
							else if(isNumeric(value_str))
							{
								switch(_entry[code_index].type)
								{
								case MMI_ENTRY_DATA:
									{
										//! value is numeric
										int num = atoi(value_str);
										*_entry[code_index].callback.dptr = (float)num / (float)_entry[code_index].weight;
										HAL_Serial_Print(_serial,"\t\tchange %s(x%d) to %d\n",_entry[code_index].code,_entry[code_index].weight,(int32_t)(*_entry[code_index].callback.dptr*(float)_entry[code_index].weight));
										HAL_Serial_Print(_serial,"\t%s(x%d)=",_entry[code_index].code,_entry[code_index].weight);

										//! repeat value input
										state = MMI_VALUE;
										value_str_position = 0;
									}
									break;
								case MMI_ENTRY_FUNCTION:
									{
										//! return to safe state
										state = MMI_CODE;
										code_str_position = 0;
										printf("cmd>");
									}
									break;
								case MMI_ENTRY_FUNCTION_1ARG:
									{
										//! value is numeric
										int num = atoi(value_str);
										//! call FUNCTION with arg
										HAL_Serial_Print(_serial,"\tcalling %s(%d)...",_entry[code_index].code,num);
										(*_entry[code_index].callback.fptr1arg)(num);
										//! go to code name input
										printf("\ncmd>");
										state = MMI_CODE;
										code_str_position = 0;
									}
									break;
								}



							}
							else
							{
								switch(_entry[code_index].type)
								{
								case MMI_ENTRY_DATA:
									{
										//! invalid value
										HAL_Serial_Print(_serial,"\t%s(x%d)=%d\n",_entry[code_index].code,_entry[code_index].weight,(int32_t)(*_entry[code_index].callback.dptr*(float)_entry[code_index].weight));
										HAL_Serial_Print(_serial,"\t%s(x%d)=",_entry[code_index].code,_entry[code_index].weight);

										//! repeat value input
										state = MMI_VALUE;
										value_str_position = 0;
									}
									break;
								case MMI_ENTRY_FUNCTION:
									{
										//! return to safe state
										state = MMI_CODE;
										code_str_position = 0;
										printf("cmd>");
									}
									break;
								case MMI_ENTRY_FUNCTION_1ARG:
									{
										//! invalid value
										printf("\targ=");
										//! repeat value input
										state = MMI_VALUE;
										value_str_position = 0;
									}
									break;
								}
							}
						}
						else
						{
							value_str[value_str_position++]=c;
							if(value_str_position>=STR_BUFFER_SIZE)
								value_str_position = STR_BUFFER_SIZE-1;
						}
					}
					break;
			}
	    }
}


#endif /* MMI_C_ */
