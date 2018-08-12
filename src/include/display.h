#ifndef DISPLAY_H
#define DISPLAY_H
/* Matrix settings */

#define MATRIX_ROWS 14
#define MATRIX_COLUMNS 14

void vDisplayTask( void *pvParameters );
void displayInit();
void vDisplayInterruptHandler( void );
#endif
