/*
 * azamipng.h
 *
 *  Created on: 6 Nov 2019
 *      Author: drark
 */

#ifndef AZAMIPNG_H_
#define AZAMIPNG_H_

#include <string.h> /*for size_t*/
/**Options**/
#define USE_LZ77
//#define USE_LZ77_IP //not implemented
#define TEST_LZ77_OUTPUT
#define EXAMPLE_RANDOM //VGA sized random image generation
//#define USE_DEBUG_PRINTF
/**Setting**/
#define MEM_MAXADDR 0x1fffffff
#define MEM_MINADDR 0x00500000
#define MEM_RAWSIZE 0x01ffffff
#define MEM_SPACE	0x10
//Deflat5e
#define DEFLATE_MAX_STACK_BYTESIZE 0x130000 //min vlz777 buffer too big for small images (1.2 MiB)
//LZ77
#define WINDOW_SIZE 2048 	//standard from lodepng
#define MINMATCH 3			//standard from lodepng
#define NICEMATCH 128		//standard from lodepng
#define MAX_DFLATE_LZ77_LENGTH 258
#define LZ77MINBUFFERSIZE  350000
#define LZ77MAXBUFFERSIZE 1398110
#ifdef USE_LZ77
#ifndef USE_HWLZ77
#define USE_SWLZ77
#endif
#endif
#ifndef USE_LZ77
#define NOLZ77
#endif
//Huffman Codding
#define NUM_CODE_LENGTH_CODES 19
//General
#ifndef NULL
#define NULL 0
#endif

#define MEM_MINREQU 0x140000// in case of very small images min mem allocation fails to alloc enough
/*Base Structs*/

/*Memory*/
void* Mem_MaxAddr=(void*)MEM_MAXADDR;
void* Mem_MinAddr=(void*)MEM_MINADDR;
/**Error **/
#define NoErr 0
#define ErrOk 0
//Mem errs
#define ErrMallocFailure -1
#define ErrMemmoryOverflow -2
#define ErrMemmoryOutOfBounds -3
//write vec errs
#define ErrDataWriteOverflow -10
#define ErrCharWriteOverflow -11
#define ErrUnsignedWriteOverflow -12
//Lz77 errs
#define ErrLZ77BadLogic -20
#define ErrLZ77BigWindow -21
#define ErrLZ77WindowNotPowerOf2 -22
#define ErrLZ77WrongEncoded -23
//Huffman erss
#define ErrDeflateAllocFailsN0  -24//frequencies count
#define ErrDeflateAllocFailsN1  -25//frequencies count#define ErrDeflateAllocFailsN0  -24//frequencies count
#define ErrDeflateAllocFailsN2  -26//frequencies count
#define ErrHuffMakeAllocFails  -27//MakeTree ll
#define ErrHuffLEavesAllocFaills -28
#define ErrHuffListAllocFaills -29
#define ErrHuffMAkeAllocFaills2 -30
#define ErrHuffMAkeAllocFaills3 -31
#define ErrHuffTree2d -32
#define ErrDeflateAllocFailsN3  -27//frequencies count
#define ErrDeflateAllocFailsN4  -28
#define ErrDeflateAllocFailsN5  -29
#define ErrHuff256SymbolInvVal -30
//Deflate
#define ErrDeflateCompressFail -31
//PNG creation
#define ErrPngCreation -35
#define ErrIHDR -32
#define ErrIDAT -33
#define ErrIEND -34
//miscillaneous
#define ERRFailToOpenFile -40;
/*******************************************************************/
/** Functions                                                     **/
/*******************************************************************/
int AzamiPNG_encode();

#endif /* AZAMIPNG_H_ */
