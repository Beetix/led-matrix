
#include "unity.h"
#include "SpiceMatrix.h"

#define DIMENSION_OF(a) (sizeof(a)/sizeof(a[0]))

void test_ucBrightnessToBin_0_percent_should_return_0( void )
{
   TEST_ASSERT_EQUAL_HEX8( 0, ucBrightnessToBin( 0 ) );
}

void test_ucBrightnessToBin_100_percent_should_return_255( void )
{
    TEST_ASSERT_EQUAL_HEX8( 255, ucBrightnessToBin( 100 ) );
}

void test_ucBrightnessToBin_50_percent_should_return_127( void )
{
    TEST_ASSERT_EQUAL_HEX8( 127, ucBrightnessToBin( 50 ) );
}

void test_ucBrightnessToBin_several_in_bound_percentages( void )
{
    int piInput[] = { 7, 11, 16, 54, 88 };
    uint8_t pucExpected[] = { 17, 28, 40, 137, 224 };
    int i;

    for ( i = 0; i < DIMENSION_OF( piInput ); i++ )
    {
        TEST_ASSERT_EQUAL_HEX8( pucExpected[ i ], ucBrightnessToBin( piInput[ i ] ) );
    }
}

void test_ucBrightnessToBin_negative_percentage_should_return_0( void )
{
    TEST_ASSERT_EQUAL_HEX8( 0, ucBrightnessToBin( -1 ) );
}

void test_ucBrightnessToBin_percentage_over_100_should_return_0( void )
{
    TEST_ASSERT_EQUAL_HEX8( 0, ucBrightnessToBin( 101 ) );
    TEST_ASSERT_EQUAL_HEX8( 0, ucBrightnessToBin( 23553 ) );
}

void test_iBinToBrightness_0_should_return_0_percent( void )
{
    TEST_ASSERT_EQUAL_INT( 0, iBinToBrightness( 0 ) );
}

void test_iBinToBrightness_255_should_return_100_percent( void )
{
    TEST_ASSERT_EQUAL_INT( 100, iBinToBrightness( 255 ) );
}

void test_iBinToBrightness_several_bytes( void )
{
    uint8_t pucInput[] = { 54, 103, 247, 253 };
    int piExpected[] = { 21, 40, 96, 99 };
    int i;

    for ( i = 0; i < DIMENSION_OF( pucInput ); i++ )
    {
        TEST_ASSERT_EQUAL_INT( piExpected[ i ], iBinToBrightness( pucInput[ i ] ) );
    }
    
}

void test_vFlashNextFrame_matrix_1x1_0_percent_should_change_to_100_percent( void )
{
    uint8_t pucMatrix[ 1 ][ 1 ] = { 0 };
    const uint8_t pUCExpected[ 1 ][ 1 ] = { 1 };
    const uint8_t ucRows = 1, ucCols = 1;
    printf ( "\n%x\n", &pucMatrix );
    vFlashNextFrame( &pucMatrix, ucRows, ucCols );

    TEST_ASSERT_EQUAL_UINT8_ARRAY( pUCExpected, pucMatrix, 1 ); 
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST( test_ucBrightnessToBin_0_percent_should_return_0 );
    RUN_TEST( test_ucBrightnessToBin_100_percent_should_return_255 );
    RUN_TEST( test_ucBrightnessToBin_50_percent_should_return_127 );
    RUN_TEST( test_ucBrightnessToBin_several_in_bound_percentages );
    RUN_TEST( test_ucBrightnessToBin_negative_percentage_should_return_0 );
    RUN_TEST( test_ucBrightnessToBin_percentage_over_100_should_return_0 );
    RUN_TEST( test_iBinToBrightness_0_should_return_0_percent );
    RUN_TEST( test_iBinToBrightness_255_should_return_100_percent );
    RUN_TEST( test_iBinToBrightness_several_bytes);
    RUN_TEST( test_vFlashNextFrame_matrix_1x1_0_percent_should_change_to_100_percent );
    return UNITY_END();
}
