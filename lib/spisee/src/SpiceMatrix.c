
#include "SpiceMatrix.h"

uint8_t ucBrightnessToBin( int iPercent )
{
    if ( iPercent < 0 )
    {
        return 0;
    }
    
    if ( iPercent > 100 )
    {
        return 0;
    }

    return iPercent * 255 / 100;
}

int iBinToBrightness( uint8_t ucBinary )
{
    return ( ( int ) ucBinary ) * 100 / 255;
}


void vFlashNextFrame( uint8_t **pucAddressMatrix, const uint8_t UCRows, const uint8_t UCCols  )
{
    int iCtrRows, iCtrCols;

    uint8_t *pucMatrix = *pucAddressMatrix;

    for ( iCtrRows = 0; iCtrRows < UCRows; iCtrRows++ )
    {
        for ( iCtrCols = 0; iCtrCols < UCCols; iCtrCols++ )
        {
            if ( iBinToBrightness( pucMatrix[ iCtrRows * UCCols + iCtrCols ] ) )
            {
                pucMatrix[ iCtrRows * UCCols + iCtrCols ] = ucBrightnessToBin( 0 ); 
            }
            else
            {
                pucMatrix[ iCtrRows * UCCols + iCtrCols ] = ucBrightnessToBin( 100 ); 
            }

        }
    }
}



void vCountingAnimation( void )
{
    /*
    volatile int i;
    for ( i = 0; i < MATRIX_ROWS; i++ )
    {

        if ( _pucValues[ i ] == 1 || _pucValues[ i ] == 16383 )
        {
            _pbCountBackward[ i ] = ! _pbCountBackward[ i ];
            printf( "Row number %d now counting %s\n", i, _pbCountBackward[ i ] ? "backward" : "forward" );
        }

        _pucValues[ i ] = ( _pbCountBackward[ i ] ) ? _pucValues[ i ] >> 1 : ( _pucValues[ i ] << 1 ) + 1; 
        printf( "Row number %d now counting %s to %u\n", i, _pbCountBackward[ i ] ? "down" : "up", _pucValues[ i ] );
    }
    */


}
