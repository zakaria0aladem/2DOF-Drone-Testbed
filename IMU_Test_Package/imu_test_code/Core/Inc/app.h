/*
 * app.h - Application layer for the 2DOF drone rig
 *
 * main.c only calls these two functions, so all project logic lives in
 * app.c and CubeMX can regenerate main.c without touching it.
 */
#ifndef APP_H
#define APP_H

#include "main.h"

/* Called once, after CubeMX has initialised the peripherals (USER CODE 2). */
void App_Init(void);

/* Called repeatedly from the while(1) loop in main.c (USER CODE 3). */
void App_Loop(void);

#endif /* APP_H */
