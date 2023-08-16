#ifndef __KEYBOARD_H__
#define __KEYBOARD_H__

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
static struct termios initial_settings, new_settings;
static int peek_character = -1;
void init_keyboard(void);
void close_keyboard(void);
int kbhit(void);
int readch(void); 
int scanKeyboard(void);

#endif