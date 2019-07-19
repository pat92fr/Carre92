/*
 * app_fw.h
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#ifndef APPLICATION_USER_HAL_APP_FW_H_
#define APPLICATION_USER_HAL_APP_FW_H_

#ifdef __cplusplus
extern "C" {
#endif

    /**
      * @brief  Setup callback.
      * @param  None
      * @retval None
      */
    void APP_SetupCallback(void);

    /**
      * @brief  Loop callback.
      *         Called at each iteration of the main loop (background task).
      * @param  None
      * @retval None
      */
    void APP_LoopCallback(void);

    /**
      * @brief  Systick callback.
      *         Called every ms (highest priority task).
      * @param  None
      * @retval None
      */
    void APP_SystickCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_USER_HAL_APP_FW_H_ */
