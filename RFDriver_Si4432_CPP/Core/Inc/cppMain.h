/*
 * cppMain.h
 *
 *  Created on: Aug 2, 2024
 *      Author: Zi
 */

#ifndef INC_CPPMAIN_H_
#define INC_CPPMAIN_H_


#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"


/* USER MAIN CODE BEGIN */
void cppMain(void);
/* USER MAIN CODE END */


/* USER CODE BEGIN Private defines */
extern UART_HandleTypeDef	huart2;
//extern I2C_HandleTypeDef 	hi2c1;
extern SPI_HandleTypeDef	hspi1;
/* USER CODE END Private defines */


#ifdef __cplusplus
}
#endif


#endif /* INC_CPPMAIN_H_ */
