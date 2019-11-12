/*
 * azamipng.c
 *
 *  Created on: 6 Nov 2019
 *      Author: Drarkin
 *      email: drarkin@oulook.com
 *      website: https://github.com/Drarkin
 *
 *      Base RGB(A) images PNG ENcoder for Embed Systems
 *
 *      This source code is based on lodePNG (  )
 *
 */


#include "azamipng.h"

#include <stdio.h>
#include <stdlib.h>

/*******************************************************************/
/** Base Structures                                               **/
/*******************************************************************/

struct _imginfo{
	unsigned width;
	unsigned heigth;
	unsigned bpp;//RGB:24bit or RGBA:32bit
	void	*data;

	unsigned bitdepth;
	unsigned colortype;
	unsigned interlace_method;
} imginfo;

struct _LZ77_settings{
	unsigned minmatch;
	unsigned nicematch;
	unsigned window;
	void* valid;//validates that the setting have been set
} LZ77settings;


typedef struct _DataVectorChar{
	char* data;
	size_t size;
	size_t maxsize;
} DV_char;

typedef struct _DataVectorUnsigned{
	unsigned* data;
	size_t size;
	size_t maxsize;
} DV_unsigned;


/*******************************************************************/
/** Base Functions to configure encoder                           **/
/*******************************************************************/
void set_inimg(void*data,unsigned width, unsigned heigth,unsigned bitsperpixel){
	imginfo.bpp=bitsperpixel;
	imginfo.data=data;
	imginfo.heigth=heigth;
	imginfo.width=width;

	imginfo.bitdepth=8;
	imginfo.colortype=2;
	imginfo.interlace_method=0;
	return;
}
void set_baseencoder(){
	LZ77settings.minmatch=MINMATCH;
	LZ77settings.nicematch=NICEMATCH;
	LZ77settings.window=WINDOW_SIZE;
	LZ77settings.valid=&LZ77settings;
}
int set_mem(size_t memsize){
	Mem_MinAddr = malloc(memsize);
	if (Mem_MinAddr==NULL) return ErrMallocFailure;
	Mem_MaxAddr = Mem_MinAddr+memsize;
	return ErrOk;
}
int set_memforimg(unsigned heigth, unsigned width){

	size_t maxneedsize= (6+heigth)*(width*4+1)*2+DEFLATE_MAX_STACK_BYTESIZE;
	maxneedsize= (maxneedsize < MEM_MINREQU)? MEM_MINREQU:maxneedsize;
	return set_mem(maxneedsize);
}
/*******************************************************************/
/** Base Information                                              **/
/*******************************************************************/
void* Mem_FreeAddr;
unsigned error;



/*******************************************************************/
/** Basic Important Data                                          **/
/*******************************************************************/
DV_char FilteredData;//contains the filtered data
DV_char DeflateData;
DV_char ZlibData;
DV_unsigned LZ77edData;
DV_char PNGchar;
/*******************************************************************/
/** Base Data Operations                                          **/
/*******************************************************************/
int dvc_ini(DV_char *dv, size_t size,void* data_addr ){
	if (data_addr<Mem_MinAddr) return ErrMemmoryOutOfBounds;
	dv->maxsize=size;
	dv->size=0;
	dv->data=data_addr;
	return ErrOk;
}
int dvc_write(DV_char *dv,char c){
	if (dv->maxsize<dv->size+1){
#ifdef USE_DEBUG_PRINTF
		printf("ErrCharWriteOverflow\r\n");
#endif
		return ErrCharWriteOverflow;
	}
	dv->data[dv->size++]=c;
	return ErrOk;
}

int dvc_writeS(DV_char *dv,char *str,int size){
	if (dv->maxsize<dv->size+size){
#ifdef USE_DEBUG_PRINTF
		printf("ErrCharWriteOverflow\r\n");
#endif
		return ErrCharWriteOverflow;
	}
	for (int i=0;i<size;i++)
		dv->data[dv->size+i]=str[i];
	dv->size+=size;
	return ErrOk;
}
static int dvc_add32bitInt(DV_char* dv, unsigned value)
{
	if (dv->maxsize<dv->size+4){
	#ifdef USE_DEBUG_PRINTF
			printf("ErrCharWriteOverflow\r\n");
	#endif
			return ErrCharWriteOverflow;
		}
	dv->data[dv->size]   = (unsigned char)((value >> 24) & 0xff);
	dv->data[dv->size+1] = (unsigned char)((value >> 16) & 0xff);
	dv->data[dv->size+2] = (unsigned char)((value >>  8) & 0xff);
	dv->data[dv->size+3] = (unsigned char)((value      ) & 0xff);
	dv->size+=4;
	return ErrOk;
}

int dvu_ini(DV_unsigned *dv, size_t size,void* data_addr ){
	if (data_addr<Mem_MinAddr) return ErrMemmoryOutOfBounds;
	dv->maxsize=size;
	dv->size=0;
	dv->data=data_addr;
	return ErrOk;
}
int dvu_write(DV_unsigned *dv,unsigned u){
	if (dv->maxsize<dv->size+1){
#ifdef USE_DEBUG_PRINTF
		printf("ErrUnsignedWriteOverflow: %p MaxSize:%zd Size:%zd\r\n",dv->data,dv->maxsize,dv->size);
#endif
		return ErrUnsignedWriteOverflow;
	}
	dv->data[dv->size++]=u;
	return ErrOk;
}
void dvu_reset(DV_unsigned *dv){
	for(int i=0;i<dv->maxsize;i++)
		dv->data[i]=0;
}

static int write32bit(char* d, unsigned value)
{
	d[0] = (unsigned char)((value >> 24) & 0xff);
	d[1] = (unsigned char)((value >> 16) & 0xff);
	d[2] = (unsigned char)((value >>  8) & 0xff);
	d[3] = (unsigned char)((value      ) & 0xff);
	return ErrOk;
}
/*******************************************************************/
/** Micillaneous                                                  **/
/*******************************************************************/
int writefilefc(DV_char *dv,char* name){
	   FILE *fileAddress;
	   fileAddress = fopen(name, "w");
#ifdef USE_DEBUG_PRINTF
	  printf("Create file %s in %p\r\nPNG addr:%p size:%zd\r\n",name,fileAddress,dv->data,dv->size);
#endif
	   if (fileAddress != NULL) {
		   size_t i;
		for (i = 0; i < dv->size; i++) {
		   // Let us use our fputc
		   fputc (dv->data[i], fileAddress);
		}
		fclose(fileAddress);
	   }
	   else {
	  	  return ERRFailToOpenFile;
	   }
	   return ErrOk;
}

#ifdef USE_DEBUG_PRINTF
	void dvc_print(DV_char *dv){
		for(size_t i = 0; i< dv->size; i+=2){
			  printf("%x",dv->data[i]&0xff);
			  if ((i&0x1)==1)
				  printf(" ");
		}
		printf("\r\n");
		return;
	}
	void FilterImg_print(){
		size_t l,c,i;
		i=0;
		char* pixel=FilteredData.data;
		for (l=0;l<imginfo.heigth;l++){
			printf("%x:\t",pixel[i++]&0xff);
			for(c=0;c<imginfo.width;c++){
				switch(imginfo.bpp){
				case 24:
					printf("[%2x %2x %2x]",pixel[i]&0xff,pixel[i+1]&0xff,pixel[i+2]&0xff);
					i+=3;
					break;
				default:
					printf("unknown bpp:%d!\r\n",imginfo.bpp);
					return;
				}
			}
			printf("\r\n");
		}
		printf("\r\n\n");
	}
	void ImgRGB_print(){
		size_t l,c,i;
		i=0;
		char* pixel=imginfo.data;
		for (l=0;l<imginfo.heigth;l++){
			printf("  \t");
			for(c=0;c<imginfo.width;c++){
				switch(imginfo.bpp){
				case 24:
					printf("[%2x %2x %2x]",pixel[i]&0xff,pixel[i+1]&0xff,pixel[i+2]&0xff);
					i+=3;
					break;
				default:
					printf("unknown bpp:%d!\r\n",imginfo.bpp);
					return;
				}
			}
			printf("\r\n");
		}
	}
#endif
/*******************************************************************/
/** Filtering image data                                          **/
/*******************************************************************/
/*Paeth predicter, used by PNG filter type 4
The parameters are of type short, but should come from unsigned chars, the shorts
are only needed to make the paeth calculation correct.
*/
static unsigned char paethPredictor(short a, short b, short c)
{
  short pa = abs(b - c);
  short pb = abs(a - c);
  short pc = abs(a + b - c - c);

  if(pc < pa && pc < pb) return (unsigned char)c;
  else if(pb < pa) return (unsigned char)b;
  else return (unsigned char)a;
}

static void filterScanline(unsigned char* out, unsigned char* scanline, unsigned char* prevline,
                           size_t length, size_t bytewidth, unsigned char filterType)
{
  size_t i;
  switch(filterType)
  {
    case 0: /*None*/
      for(i = 0; i != length; ++i) out[i] = scanline[i];
      break;
    case 1: /*Sub*/
      for(i = 0; i != bytewidth; ++i) out[i] = scanline[i];
      for(i = bytewidth; i < length; ++i) out[i] = scanline[i] - scanline[i - bytewidth];
      break;
    case 2: /*Up*/
      if(prevline)
      {
        for(i = 0; i != length; ++i) out[i] = scanline[i] - prevline[i];
      }
      else
      {
        for(i = 0; i != length; ++i) out[i] = scanline[i];
      }
      break;
    case 3: /*Average*/
      if(prevline)
      {
        for(i = 0; i != bytewidth; ++i) out[i] = scanline[i] - (prevline[i] >> 1);
        for(i = bytewidth; i < length; ++i) out[i] = scanline[i] - ((scanline[i - bytewidth] + prevline[i]) >> 1);
      }
      else
      {
        for(i = 0; i != bytewidth; ++i) out[i] = scanline[i];
        for(i = bytewidth; i < length; ++i) out[i] = scanline[i] - (scanline[i - bytewidth] >> 1);
      }
      break;
    case 4: /*Paeth*/
      if(prevline)
      {
        /*paethPredictor(0, prevline[i], 0) is always prevline[i]*/
        for(i = 0; i != bytewidth; ++i) out[i] = (scanline[i] - prevline[i]);
        for(i = bytewidth; i < length; ++i)
        {
          out[i] = (scanline[i] - paethPredictor(scanline[i - bytewidth], prevline[i], prevline[i - bytewidth]));
        }
      }
      else
      {
        for(i = 0; i != bytewidth; ++i) out[i] = scanline[i];
        /*paethPredictor(scanline[i - bytewidth], 0, 0) is always scanline[i - bytewidth]*/
        for(i = bytewidth; i < length; ++i) out[i] = (scanline[i] - scanline[i - bytewidth]);
      }
      break;
    default: return; /*unexisting filter type given*/
  }
}


static unsigned filter()
{
	if(error){
		return error;
	}
	/*Setup for to receiv3e filtered data from the filtering process*/
	unsigned bpp = imginfo.bpp;
	size_t linebytes = (imginfo.width * bpp + 7) / 8;//the width of a scanline in bytes, not including the filter type
	size_t scanlinesize= linebytes+1; //scanline size with the filter type byte
	FilteredData.maxsize = imginfo.heigth * scanlinesize; //full filtered image size
	FilteredData.size=FilteredData.maxsize;
	//filtered data starts 6 scanlines bellow  raw image. filtered image will overwrite raw image to save space
	FilteredData.data =(unsigned char*) (((size_t)imginfo.data-(7*scanlinesize))&~0xf);
	if ((void*)FilteredData.data<Mem_MinAddr)return ErrMemmoryOutOfBounds;
	unsigned w=imginfo.width;
	unsigned h=imginfo.heigth;
	unsigned char* in=imginfo.data;
	/*
  For PNG filter method 0
  out must be a buffer with as size: h + (w * h * bpp + 7) / 8, because there are
  the scanlines with 1 extra byte per scanline
  */



  /*bytewidth is used for filtering, is 1 when bpp < 8, number of bytes per pixel otherwise*/
  size_t bytewidth = (bpp + 7) / 8;
  unsigned char* prevline = 0;
  unsigned x, y;

    /*adaptive filtering*/
    size_t sum[5];
    unsigned char* attempt[5]; /*five filtering attempts, one for each filter type*/
    size_t smallest = 0;
    unsigned char type, bestType = 0;
    for(y = 0; y != h; ++y)
      {
		for(type = 0; type != 5; ++type)
		{
		  attempt[type] = (unsigned char*) FilteredData.data +(y+1+type)*(scanlinesize);
		  if(attempt[type]>(unsigned char*)Mem_MaxAddr) return ErrMemmoryOverflow; /*max addr reached*/
		}
        /*try the 5 filter types*/
        for(type = 0; type != 5; ++type)
        {
          filterScanline(attempt[type], &in[y * linebytes], prevline, linebytes, bytewidth, type);

          /*calculate the sum of the result*/
          sum[type] = 0;
          if(type == 0)
          {
            for(x = 0; x != linebytes; ++x) sum[type] += (unsigned char)(attempt[type][x]);
          }
          else
          {
            for(x = 0; x != linebytes; ++x)
            {
              /*For differences, each byte should be treated as signed, values above 127 are negative
              (converted to signed char). Filtertype 0 isn't a difference though, so use unsigned there.
              This means filtertype 0 is almost never chosen, but that is justified.*/
              unsigned char s = attempt[type][x];
              sum[type] += s < 128 ? s : (255U - s);
            }
          }

          /*check if this is smallest sum (or if type == 0 it's the first case so always store the values)*/
          if(type == 0 || sum[type] < smallest)
          {
            bestType = type;
            smallest = sum[type];
          }
         // printf("SL:%5d;Type:%d Sum:%ld\r\n",y,type,sum[type]);//check filter scan lines evaluation
        }

        prevline = &in[y * linebytes];

        /*now fill the out values*/
        FilteredData.data[y * (linebytes + 1)] = bestType; /*the first byte of a scanline will be the filter type*/
        for(x = 0; x != linebytes; ++x) FilteredData.data[y * scanlinesize  + x+1] = attempt[bestType][x];
      }
#ifdef USE_DEBUG_PRINTF
    printf("Filtering ended - Err:%d\r\n",error);
#endif
  return ErrOk;
}



/*******************************************************************/
/** LZ77 Algorithms                                               **/
/*******************************************************************/
#ifdef TEST_LZ77_OUTPUT
size_t lz77fimg;//REMOVE!

int LZ77Test(unsigned char* idata,int* ldata){
	if(error){
			return error;
		}
	lz77fimg=0;//REMOVE!
	size_t lpos=0;
	size_t ipos=0;
	int equal=1;
	int DISTANCEBASE[30]
	  = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513,
	     769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
	int LENGTHBASE[29]
	  = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59,
	     67, 83, 99, 115, 131, 163, 195, 227, 258};
	while(ldata[lpos]!=256 && equal==1){
		int code=ldata[lpos];

		if (code>256){
			//obtain pointer codes
			int lencode = ldata[lpos]-257;
			int lenextr = ldata[lpos+1];
			int discode = ldata[lpos+2];
			int disextr = ldata[lpos+3];
			lpos += 4;
			//obtain len&dist
			int length =  LENGTHBASE[lencode]+lenextr;
			int distance = DISTANCEBASE[discode]+disextr;
			for (int i=0;i<length;i++){
				equal &= idata[ipos]==idata[ipos-distance];
				ipos++;
#ifdef USE_DEBUG_PRINTF
				if(!equal)printf("LZ77 Failed in Ptr(%d|%d)[%d]\r\n",length,distance,i);
#endif
			}
		}else{
				int letter=(int)idata[ipos];
				equal &= letter == code;
				ipos++;
				lpos++;
#ifdef USE_DEBUG_PRINTF
				if(!equal)printf("LZ77 Failed in pos:%d char:{%d}\r\n",(int)ipos,letter);
#endif
		}
	}
	if (equal){
#ifdef USE_DEBUG_PRINTF
		printf("LZ77 Succeed\r\n");
#endif
		return ErrOk;
	}else{
		return ErrLZ77WrongEncoded;
	}
}

#endif
#ifdef USE_LZ77_IP

#else
int DISTANCEBASE[30]
  = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513,
     769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
int LENGTHBASE[29]
  = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59,
     67, 83, 99, 115, 131, 163, 195, 227, 258};
/*the extra bits of backwards distances (added to base)*/
static const unsigned DISTANCEEXTRA[30]
  = {0, 0, 0, 0, 1, 1, 2,  2,  3,  3,  4,  4,  5,  5,   6,   6,   7,   7,   8,
       8,    9,    9,   10,   10,   11,   11,   12,    12,    13,    13};
/*the extra bits used by codes 257-285 (added to base length)*/
static const unsigned LENGTHEXTRA[29]
  = {0, 0, 0, 0, 0, 0, 0,  0,  1,  1,  1,  1,  2,  2,  2,  2,  3,  3,  3,  3,
      4,  4,  4,   4,   5,   5,   5,   5,   0};
/*search the index in the array, that has the largest value smaller than or equal to the given value,
given array must be sorted (if no value is smaller, it returns the size of the given array)*/
static size_t searchCodeIndex(const unsigned* array, size_t array_size, size_t value)
{
  /*binary search (only small gain over linear). TODO: use CPU log2 instruction for getting symbols instead*/
  size_t left = 1;
  size_t right = array_size - 1;

  while(left <= right) {
    size_t mid = (left + right) >> 1;
    if (array[mid] >= value) right = mid - 1;
    else left = mid + 1;
  }
  if(left >= array_size || array[left] > value) left--;
  return left;
}
int addLengthDistance(DV_unsigned *dv, unsigned length, unsigned distance){
	/*values in encoded vector are those used by deflate:
	  0-255: literal bytes
	  256: end
	  257-285: length/distance pair (length code, followed by extra length bits, distance code, extra distance bits)
	  286-287: invalid*/

	  unsigned length_code = (unsigned)searchCodeIndex(LENGTHBASE, 29, length);
	  unsigned extra_length = (unsigned)(length - LENGTHBASE[length_code]);
	  unsigned dist_code = (unsigned)searchCodeIndex(DISTANCEBASE, 30, distance);
	  unsigned extra_distance = (unsigned)(distance - DISTANCEBASE[dist_code]);

	  dvu_write(dv, length_code + 257);
	  dvu_write(dv, extra_length);
	  dvu_write(dv, dist_code);
	  dvu_write(dv, extra_distance);
	return ErrOk;
}
//Hash system
struct HashLZ77{
		struct HashLZ77* NextAddr;
		struct HashLZ77* PrevAddr;
		unsigned char* ListPtr;//pointer to symbol
	} HashList[32769];
struct HashLZ77* HashTable[256];

void HashInit(){
	//init hash - clean memory zone

		unsigned pos;
		for (pos=0;pos<256;pos++){
			HashTable[pos]=NULL;
		}
		for (pos=0;pos<LZ77settings.window;pos++){
			HashList[pos].NextAddr=NULL;
			HashList[pos].PrevAddr=NULL;
			HashList[pos].ListPtr=NULL;
		}
}
//LZ77 algorithm with hash system
static unsigned encodeLZ77(size_t inpos, size_t insize)
{
	if(error){
			return error;
		}
//lz77 settings
	unsigned windowsize = LZ77settings.window;
	unsigned minmatch = LZ77settings.minmatch;
	unsigned nicematch= LZ77settings.nicematch;

	unsigned char* in=FilteredData.data;//data

  size_t pos;
  unsigned i;
  /*for large window lengths, assume the user wants no compression loss. Otherwise, max hash chain length speedup.*/
  unsigned offset; /*the offset represents the distance in LZ77 terminology*/
  unsigned length;
  unsigned current_offset, current_length;
  unsigned prev_offset;
  unsigned char *lastptr, *foreptr, *backptr;

	struct HashLZ77 *HashListSearch,*HashListSearchOld;
	int HashListPos = -1;
	int HashListSize=windowsize;




  if(windowsize == 0 || windowsize > 32768) return ErrLZ77BigWindow; /*error: windowsize smaller/larger than allowed*/
  if((windowsize & (windowsize - 1)) != 0) return ErrLZ77WindowNotPowerOf2; /*error: must be power of two*/

  if(nicematch > MAX_DFLATE_LZ77_LENGTH) nicematch = MAX_DFLATE_LZ77_LENGTH;

  for(pos = inpos; pos < insize && error==NoErr; ++pos)
  {
	//update hash to current position
	HashListPos++;
	//check if buffer overflow, change from invalid postion back to start
	if (HashListPos==HashListSize){
		//Outside, get back to start
		HashListPos=0;
	}

	//remove current hash from chain
	if (HashList[HashListPos].PrevAddr!=NULL){
		//remove data from old entry from table
		(*HashList[HashListPos].PrevAddr).NextAddr=NULL;
		HashList[HashListPos].PrevAddr=NULL;
	}
	//HashList[HashListPos].PrevAddr=NULL;//is implicit
	//remove entry from table
	if (HashList[HashListPos].ListPtr!=NULL)
		if (HashTable[*(HashList[HashListPos].ListPtr)]==&HashList[HashListPos])
			HashTable[*(HashList[HashListPos].ListPtr)]=NULL;

	//update hash table

	if (HashTable[in[pos]]!=NULL){
		HashList[HashListPos].NextAddr=HashTable[in[pos]];
		(*HashTable[in[pos]]).PrevAddr=&HashList[HashListPos];
	}
	HashTable[in[pos]]=&HashList[HashListPos];

	//update current hash information
	HashList[HashListPos].ListPtr=&in[pos];


    /*the length and offset found for the current position*/
    length = 0;
    offset = 0;

    lastptr = &in[insize < pos +MAX_DFLATE_LZ77_LENGTH ? insize : pos + MAX_DFLATE_LZ77_LENGTH]; //stop pointer

    /*search for the longest string*/
	//current_offset=pos < windowsize? pos:windowsize;
	//initiate new search
	HashListSearch=NULL;//initiate new hash list search
    for(;;)

    {
		HashListSearchOld=HashListSearch;
		//get next hash list value
		if (HashListSearch==NULL){
			HashListSearch=(*HashTable[in[pos]]).NextAddr;
		}else{
			HashListSearch=(*HashListSearch).NextAddr;
		}
		//break condition
		if (HashListSearch==NULL || HashListSearch==&HashList[HashListPos]){
			//end search when arriving to list end
			//end search if start list is current position
			break;
		}
		//set offset
		current_offset=(unsigned)(&in[pos]-(*HashListSearch).ListPtr);
		if( current_offset > windowsize) break;
        /*test the next characters*/
        foreptr = &in[pos];
        backptr = (*HashListSearch).ListPtr;

        while(foreptr != lastptr && *backptr == *foreptr) /*maximum supported length by deflate is max length*/
        {
          ++backptr;
          ++foreptr;
        }

        current_length = (unsigned)(foreptr - &in[pos]);
        if(current_length > length)
        {
          length = current_length; /*the longest length*/
          offset = current_offset; /*the offset that is related to this longest length*/
          /*jump out once a length of max length is found (speed gain). This also jumps
          out if length is MAX_SUPPORTED_DEFLATE_LENGTH*/
          if(current_length >= nicematch) break;
        }

    }

    if(length >= 3 && offset > windowsize){
		return ErrLZ77BadLogic;
	}
    /*encode it as length/distance pair or literal value*/
    if(length <= 3) /*only lengths of 3 or higher are supported as length/distance pair*/
    {
      error=dvu_write(&LZ77edData, in[pos]);
    }
    else if(length < minmatch || (length == 3 && offset > 4096))
    {
      /*compensate for the fact that longer offsets have more extra bits, a
      //length of only 3 may be not worth it then*/
      error=dvu_write(&LZ77edData, in[pos]);
    }
    else
    {
      error=addLengthDistance(&LZ77edData, length, offset);
	  for(i = 1; i < length; ++i){
		  //update pos
		  pos++;

			//update hash to current position
			HashListPos++;
			//check if buffer overflow, change from invalid postion back to start
			if (HashListPos==HashListSize){
				//Outside, get back to start
				HashListPos=0;
			}

			//remove current hash from chain
			if (HashList[HashListPos].PrevAddr!=NULL){
				//remove data from old entry from table
				(*HashList[HashListPos].PrevAddr).NextAddr=NULL;
				HashList[HashListPos].PrevAddr=NULL;
			}
			//HashList[HashListPos].PrevAddr=NULL;//is implicit
			//remove entry from table
			if (HashList[HashListPos].ListPtr!=NULL)
				if (HashTable[*(HashList[HashListPos].ListPtr)]==&HashList[HashListPos])
					HashTable[*(HashList[HashListPos].ListPtr)]=NULL;

			//update hash table

			if (HashTable[in[pos]]!=NULL){
				HashList[HashListPos].NextAddr=HashTable[in[pos]];
				(*HashTable[in[pos]]).PrevAddr=&HashList[HashListPos];
			}
			HashTable[in[pos]]=&HashList[HashListPos];

			//update current hash information
			HashList[HashListPos].ListPtr=&in[pos];
	  }
    }
  } /*end of the loop through each character of input*/
  error=dvu_write(&LZ77edData,256);//and termination code
#ifdef USE_DEBUG_PRINTF
    printf("LZ77 ended - Err:%d\r\n",error);
#endif
  return error;
}
#endif
/*******************************************************************/
/** Huffman Coding Dynamic                                        **/
/*******************************************************************/
typedef struct _HuffmanTree
{
  unsigned* tree2d;
  unsigned* tree1d;
  unsigned* lengths; /*the lengths of the codes of the 1d-tree*/
  unsigned maxbitlen; /*maximum number of bits a single code can get*/
  unsigned numcodes; /*number of symbols in the alphabet = number of codes*/
} HuffmanTree;
#define FIRST_LENGTH_CODE_INDEX 257
#define LAST_LENGTH_CODE_INDEX 285
/*256 literals, the end code, some length codes, and 2 unused codes*/
#define NUM_DEFLATE_CODE_SYMBOLS 288
/*the distance codes have their own symbols, 30 used, 2 unused*/
#define NUM_DISTANCE_SYMBOLS 32
/*the code length codes. 0-15: code lengths, 16: copy previous 3-6 times, 17: 3-10 zeros, 18: 11-138 zeros*/
#define NUM_CODE_LENGTH_CODES 19
/*the order in which "code length alphabet code lengths" are stored, out of this
the huffman tree of the dynamic huffman tree lengths is generated*/
static const unsigned CLCL_ORDER[NUM_CODE_LENGTH_CODES]
  = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
/*BPM: Boundary Package Merge, see "A Fast and Space-Economical Algorithm for Length-Limited Coding",
Jyrki Katajainen, Alistair Moffat, Andrew Turpin, 1995.*/

/*chain node for boundary package merge*/
typedef struct BPMNode
{
  int weight; /*the sum of all weights in this chain*/
  unsigned index; /*index of this leaf node (called "count" in the paper)*/
  struct BPMNode* tail; /*the next nodes in this chain (null if last)*/
  int in_use;
} BPMNode;

/*lists of chains*/
typedef struct BPMLists
{
  /*memory pool*/
  unsigned memsize;
  BPMNode* memory;
  unsigned numfree;
  unsigned nextfree;
  BPMNode** freelist;
  /*two heads of lookahead chains per list*/
  unsigned listsize;
  BPMNode** chains0;
  BPMNode** chains1;
} BPMLists;
/*creates a new chain node with the given parameters, from the memory in the lists */
static BPMNode* bpmnode_create(BPMLists* lists, int weight, unsigned index, BPMNode* tail)
{
  unsigned i;
  BPMNode* result;

  /*memory full, so garbage collect*/
  if(lists->nextfree >= lists->numfree)
  {
    /*mark only those that are in use*/
    for(i = 0; i != lists->memsize; ++i) lists->memory[i].in_use = 0;
    for(i = 0; i != lists->listsize; ++i)
    {
      BPMNode* node;
      for(node = lists->chains0[i]; node != 0; node = node->tail) node->in_use = 1;
      for(node = lists->chains1[i]; node != 0; node = node->tail) node->in_use = 1;
    }
    /*collect those that are free*/
    lists->numfree = 0;
    for(i = 0; i != lists->memsize; ++i)
    {
      if(!lists->memory[i].in_use) lists->freelist[lists->numfree++] = &lists->memory[i];
    }
    lists->nextfree = 0;
  }

  result = lists->freelist[lists->nextfree++];
  result->weight = weight;
  result->index = index;
  result->tail = tail;
  return result;
}

/*sort the leaves with stable mergesort*/
static void bpmnode_sort(BPMNode* leaves, size_t num)
{
  BPMNode* mem = (BPMNode*)leaves-(sizeof(*leaves) * num);
  size_t width, counter = 0;
  for(width = 1; width < num; width *= 2)
  {
    BPMNode* a = (counter & 1) ? mem : leaves;
    BPMNode* b = (counter & 1) ? leaves : mem;
    size_t p;
    for(p = 0; p < num; p += 2 * width)
    {
      size_t q = (p + width > num) ? num : (p + width);
      size_t r = (p + 2 * width > num) ? num : (p + 2 * width);
      size_t i = p, j = q, k;
      for(k = p; k < r; k++)
      {
        if(i < q && (j >= r || a[i].weight <= a[j].weight)) b[k] = a[i++];
        else b[k] = a[j++];
      }
    }
    counter++;
  }
  if(counter & 1) memcpy(leaves, mem, sizeof(*leaves) * num);

}

static void boundaryPM(BPMLists* lists, BPMNode* leaves, size_t numpresent, int c, int num)
{
  unsigned lastindex = lists->chains1[c]->index;

  if(c == 0)
  {
    if(lastindex >= numpresent) return;
    lists->chains0[c] = lists->chains1[c];
    lists->chains1[c] = bpmnode_create(lists, leaves[lastindex].weight, lastindex + 1, 0);
  }
  else
  {
    /*sum of the weights of the head nodes of the previous lookahead chains.*/
    int sum = lists->chains0[c - 1]->weight + lists->chains1[c - 1]->weight;
    lists->chains0[c] = lists->chains1[c];
    if(lastindex < numpresent && sum > leaves[lastindex].weight)
    {
      lists->chains1[c] = bpmnode_create(lists, leaves[lastindex].weight, lastindex + 1, lists->chains1[c]->tail);
      return;
    }
    lists->chains1[c] = bpmnode_create(lists, sum, lastindex, lists->chains1[c - 1]);
    /*in the end we are only interested in the chain of the last list, so no
    need to recurse if we're at the last one (this gives measurable speedup)*/
    if(num + 1 < (int)(2 * numpresent - 2))
    {
      boundaryPM(lists, leaves, numpresent, c - 1, num);
      boundaryPM(lists, leaves, numpresent, c - 1, num);
    }
  }
}


unsigned lodepng_huffman_code_lengths(unsigned* lengths, const unsigned* frequencies,
                                      size_t numcodes, unsigned maxbitlen)
{

  unsigned i;
  size_t numpresent = 0; /*number of symbols with non-zero frequency*/
  BPMNode* leaves; /*the symbols, only those with > 0 frequency*/

  if(numcodes == 0) return 80; /*error: a tree of 0 symbols is not supposed to be made*/
  if((1u << maxbitlen) < (unsigned)numcodes) return 80; /*error: represent all symbols*/

  leaves = (BPMNode*)Mem_FreeAddr-(numcodes * sizeof(*leaves));
  if((void*)leaves<Mem_MinAddr) return ErrHuffLEavesAllocFaills; /*alloc fail*/

  for(i = 0; i != numcodes; ++i)
  {
    if(frequencies[i] > 0)
    {
      leaves[numpresent].weight = (int)frequencies[i];
      leaves[numpresent].index = i;
      ++numpresent;
    }
  }

  for(i = 0; i != numcodes; ++i) lengths[i] = 0;

  /*ensure at least two present symbols. There should be at least one symbol
  according to RFC 1951 section 3.2.7. Some decoders incorrectly require two. To
  make these work as well ensure there are at least two symbols. The
  Package-Merge code below also doesn't work correctly if there's only one
  symbol, it'd give it the theoritical 0 bits but in practice zlib wants 1 bit*/
  if(numpresent == 0)
  {
    lengths[0] = lengths[1] = 1; /*note that for RFC 1951 section 3.2.7, only lengths[0] = 1 is needed*/
  }
  else if(numpresent == 1)
  {
    lengths[leaves[0].index] = 1;
    lengths[leaves[0].index == 0 ? 1 : 0] = 1;
  }
  else
  {
    BPMLists lists;
    BPMNode* node;

    bpmnode_sort(leaves, numpresent);

    lists.listsize = maxbitlen;
    lists.memsize = 2 * maxbitlen * (maxbitlen + 1);
    lists.nextfree = 0;
    lists.numfree = lists.memsize;
    lists.memory = (BPMNode*)leaves-(lists.memsize * sizeof(*lists.memory));
    lists.freelist = (BPMNode**)lists.memory-(lists.memsize * sizeof(BPMNode*));
    lists.chains0 = (BPMNode**)lists.freelist-(lists.listsize * sizeof(BPMNode*));
    lists.chains1 = (BPMNode**)lists.chains0-(lists.listsize * sizeof(BPMNode*));
    if((void*)lists.chains1<Mem_MinAddr) return ErrHuffListAllocFaills; /*alloc fail*/
    Mem_FreeAddr =(void*)lists.chains1;

    if(!error)
    {
      for(i = 0; i != lists.memsize; ++i) lists.freelist[i] = &lists.memory[i];

      bpmnode_create(&lists, leaves[0].weight, 1, 0);
      bpmnode_create(&lists, leaves[1].weight, 2, 0);

      for(i = 0; i != lists.listsize; ++i)
      {
        lists.chains0[i] = &lists.memory[0];
        lists.chains1[i] = &lists.memory[1];
      }

      /*each boundaryPM call adds one chain to the last list, and we need 2 * numpresent - 2 chains.*/
      for(i = 2; i != 2 * numpresent - 2; ++i) boundaryPM(&lists, leaves, numpresent, (int)maxbitlen - 1, (int)i);

      for(node = lists.chains1[maxbitlen - 1]; node; node = node->tail)
      {
        for(i = 0; i != node->index; ++i) ++lengths[leaves[i].index];
      }
    }
  }

  return error;
}

static unsigned HuffmanTree_make2DTree(HuffmanTree* tree)
{
  unsigned nodefilled = 0; /*up to which node it is filled*/
  unsigned treepos = 0; /*position in the tree (1 of the numcodes columns)*/
  unsigned n, i;

  tree->tree2d = (unsigned*)Mem_FreeAddr-(tree->numcodes * 2 * sizeof(unsigned));
  if((void*)tree->tree2d<Mem_MinAddr) return ErrHuffTree2d; /*alloc fail*/
  Mem_FreeAddr=tree->tree2d;
  /*
  convert tree1d[] to tree2d[][]. In the 2D array, a value of 32767 means
  uninited, a value >= numcodes is an address to another bit, a value < numcodes
  is a code. The 2 rows are the 2 possible bit values (0 or 1), there are as
  many columns as codes - 1.
  A good huffman tree has N * 2 - 1 nodes, of which N - 1 are internal nodes.
  Here, the internal nodes are stored (what their 0 and 1 option point to).
  There is only memory for such good tree currently, if there are more nodes
  (due to too long length codes), error 55 will happen
  */
  for(n = 0; n < tree->numcodes * 2; ++n)
  {
    tree->tree2d[n] = 32767; /*32767 here means the tree2d isn't filled there yet*/
  }

  for(n = 0; n < tree->numcodes; ++n) /*the codes*/
  {
    for(i = 0; i != tree->lengths[n]; ++i) /*the bits for this code*/
    {
      unsigned char bit = (unsigned char)((tree->tree1d[n] >> (tree->lengths[n] - i - 1)) & 1);
      /*oversubscribed, see comment in lodepng_error_text*/
      if(treepos > 2147483647 || treepos + 2 > tree->numcodes) return 55;
      if(tree->tree2d[2 * treepos + bit] == 32767) /*not yet filled in*/
      {
        if(i + 1 == tree->lengths[n]) /*last bit*/
        {
          tree->tree2d[2 * treepos + bit] = n; /*put the current code in it*/
          treepos = 0;
        }
        else
        {
          /*put address of the next step in here, first that address has to be found of course
          (it's just nodefilled + 1)...*/
          ++nodefilled;
          /*addresses encoded with numcodes added to it*/
          tree->tree2d[2 * treepos + bit] = nodefilled + tree->numcodes;
          treepos = nodefilled;
        }
      }
      else treepos = tree->tree2d[2 * treepos + bit] - tree->numcodes;
    }
  }

  for(n = 0; n < tree->numcodes * 2; ++n)
  {
    if(tree->tree2d[n] == 32767) tree->tree2d[n] = 0; /*remove possible remaining 32767's*/
  }

  return 0;
}
/*
Second step for the ...makeFromLengths and ...makeFromFrequencies functions.
numcodes, lengths and maxbitlen must already be filled in correctly. return
value is error.
*/
static unsigned HuffmanTree_makeFromLengths2(HuffmanTree* tree)
{
  DV_unsigned blcount;
  DV_unsigned nextcode;
  unsigned bits, n;

   tree->tree1d = (unsigned*)Mem_FreeAddr-(tree->numcodes * sizeof(unsigned));
  if((void*)tree->tree1d<Mem_MinAddr) return ErrHuffMAkeAllocFaills2; /*alloc fail*/
  Mem_FreeAddr=tree->tree1d;
 dvu_ini(&blcount,tree->maxbitlen + 1,tree->tree1d-(tree->maxbitlen + 1)*sizeof(unsigned));
 dvu_ini(&nextcode,tree->maxbitlen + 1,blcount.data-(tree->maxbitlen + 1)*sizeof(unsigned));
  if((void*)nextcode.data<Mem_MinAddr)
    return ErrHuffMAkeAllocFaills3; /*alloc fail*/
  //clean allocated memory (required for blcount and nextcode)
  blcount.size=blcount.maxsize;dvu_reset(&blcount);
  nextcode.size=nextcode.maxsize;dvu_reset(&blcount);

  if(!error)
  {
    /*step 1: count number of instances of each code length*/
    for(bits = 0; bits != tree->numcodes; ++bits) ++blcount.data[tree->lengths[bits]];
    /*step 2: generate the nextcode values*/
    for(bits = 1; bits <= tree->maxbitlen; ++bits)
    {
      nextcode.data[bits] = (nextcode.data[bits - 1] + blcount.data[bits - 1]) << 1;
    }
    /*step 3: generate all the codes*/
    for(n = 0; n != tree->numcodes; ++n)
    {
      if(tree->lengths[n] != 0) tree->tree1d[n] = nextcode.data[tree->lengths[n]]++;
    }
  }


  if(!error) return HuffmanTree_make2DTree(tree);
  else return error;
}
/*Create the Huffman tree given the symbol frequencies*/
static unsigned HuffmanTree_makeFromFrequencies(HuffmanTree* tree, const unsigned* frequencies,
                                                size_t mincodes, size_t numcodes, unsigned maxbitlen)
{
  //init trees
  tree->tree2d = 0;
  tree->tree1d = 0;
  tree->lengths = 0;
  //make tress
  while(!frequencies[numcodes - 1] && numcodes > mincodes) --numcodes; /*trim zeroes*/
  tree->maxbitlen = maxbitlen;
  tree->numcodes = (unsigned)numcodes; /*number of symbols*/
  tree->lengths = (unsigned*)Mem_FreeAddr- numcodes * sizeof(unsigned);
  if((void*)tree->lengths<Mem_MinAddr) return ErrHuffMakeAllocFails;
  Mem_FreeAddr=(void*)tree->lengths;
  /*initialize all lengths to 0*/
  memset(tree->lengths, 0, numcodes * sizeof(unsigned));

  error = lodepng_huffman_code_lengths(tree->lengths, frequencies, numcodes, maxbitlen);


  if(!error) error = HuffmanTree_makeFromLengths2(tree);
  return error;
}
static unsigned HuffmanTree_getLength(const HuffmanTree* tree, unsigned index)
{
  return tree->lengths[index];
}
static unsigned HuffmanTree_getCode(const HuffmanTree* tree, unsigned index)
{
  return tree->tree1d[index];
}

/** Bit operations**/
#define addBitToStream(/*size_t**/ bitpointer, /*ucvector**/ bitstream, /*unsigned char*/ bit)\
{\
  /*add a new byte at the end*/\
  if(((*bitpointer) & 7) == 0) dvc_write(bitstream, (unsigned char)0);\
  /*earlier bit of huffman code is in a lesser significant bit of an earlier byte*/\
  (bitstream->data[bitstream->size - 1]) |= (bit << ((*bitpointer) & 0x7));\
  ++(*bitpointer);\
  }

static void addBitsToStream(size_t* bitpointer, DV_char* bitstream, unsigned value, size_t nbits)
{
  size_t i;
  for(i = 0; i != nbits; ++i) addBitToStream(bitpointer, bitstream, (unsigned char)((value >> i) & 1));
}

static void addBitsToStreamReversed(size_t* bitpointer, DV_char* bitstream, unsigned value, size_t nbits)
{
  size_t i;
  for(i = 0; i != nbits; ++i) addBitToStream(bitpointer, bitstream, (unsigned char)((value >> (nbits - 1 - i)) & 1));
}
/*bitlen is the size in bits of the code*/
static void addHuffmanSymbol(size_t* bp, DV_char* compressed, unsigned code, unsigned bitlen)
{
  addBitsToStreamReversed(bp, compressed, code, bitlen);
}

/*
write the lz77-encoded data, which has lit, len and dist codes, to compressed stream using huffman trees.
tree_ll: the tree for lit and len codes.
tree_d: the tree for distance codes.
*/
static void writeLZ77data(size_t* bp, DV_char* out, const DV_unsigned* lz77_encoded,
                          const HuffmanTree* tree_ll, const HuffmanTree* tree_d)
{
#ifdef USE_DEBUG_PRINTF
	  size_t regi=0;
#endif
  size_t i = 0;
  for(i = 0; i != lz77_encoded->size; ++i)
  {
    unsigned val = lz77_encoded->data[i];
    addHuffmanSymbol(bp, out, HuffmanTree_getCode(tree_ll, val), HuffmanTree_getLength(tree_ll, val));
    if(val > 256) /*for a length code, 3 more things have to be added*/
    {
      unsigned length_index = val - FIRST_LENGTH_CODE_INDEX;
      unsigned n_length_extra_bits = LENGTHEXTRA[length_index];
      unsigned length_extra_bits = lz77_encoded->data[++i];

      unsigned distance_code = lz77_encoded->data[++i];

      unsigned distance_index = distance_code;
      unsigned n_distance_extra_bits = DISTANCEEXTRA[distance_index];
      unsigned distance_extra_bits = lz77_encoded->data[++i];

      addBitsToStream(bp, out, length_extra_bits, n_length_extra_bits);
      addHuffmanSymbol(bp, out, HuffmanTree_getCode(tree_d, distance_code),
                       HuffmanTree_getLength(tree_d, distance_code));
      addBitsToStream(bp, out, distance_extra_bits, n_distance_extra_bits);
    }
#ifdef USE_DEBUG_PRINTF
	  if (regi!=out->size){
		  regi=out->size;
		  printf("[Deflate:HuffmanCoding](%ld:%d=[code:%u|len:%u])()%2x%2x\r\n",i,val,HuffmanTree_getCode(tree_ll, val), HuffmanTree_getLength(tree_ll, val),out->data[out->size]&0xff,out->data[out->size-1]&0xff);
	  }
#endif
  }
}
#ifdef USE_DEBUG_PRINTF
void PrintHuffmanCodes(HuffmanTree *tree){
	printf("Huffman codes of tree %p\r\n",tree);
			if(!tree)return;
	for (unsigned i=0;i<tree->numcodes;i++){
		printf("\t%u %u %u\r\n",i,tree->tree1d[i],tree->lengths[i]);
	}
}
#endif
/*******************************************************************/
/** Deflate Dynamic                                               **/
static unsigned adler32(unsigned adler, const unsigned char* data, unsigned len)
{
  unsigned s1 = adler & 0xffff;
  unsigned s2 = (adler >> 16) & 0xffff;

  while(len > 0)
  {
    /*at least 5552 sums can be done before the sums overflow, saving a lot of module divisions*/
    unsigned amount = len > 5552 ? 5552 : len;
    len -= amount;
    while(amount > 0)
    {
      s1 += (*data++);
      s2 += s1;
      --amount;
    }
    s1 %= 65521;
    s2 %= 65521;
  }

  return (s2 << 16) | s1;
}

/*******************************************************************/
int deflate(){
	if(error){
		return error;
	}

	/*on PNGs, deflate blocks of 65-262k seem to give most dense encoding*/
	//calcula deflate block average size
	size_t blocksize = FilteredData.size / 8 + 8;
	if(blocksize < 65536) blocksize = 65536;
	if(blocksize > 262144) blocksize = 262144;
	blocksize&=~0xf;
	//Zlib Data Allocation
	ZlibData.maxsize=FilteredData.maxsize; //deflate data should never be bigger than filtered data. compressed data smaller than uncompressed data
	ZlibData.data= (char*)(((size_t)FilteredData.data - (ZlibData.maxsize))&~0xf);
	ZlibData.size=0;
	if ((void*)ZlibData.data<Mem_MinAddr) return ErrMemmoryOutOfBounds;
		//zlib header
	 /*initially, *out must be NULL and outsize 0, if you just give some random *out
	  that's pointing to a non allocated buffer, this'll crash*/
		  /*zlib data: 1 byte CMF (CM+CINFO), 1 byte FLG, deflate data, 4 byte ADLER32 checksum of the Decompressed data*/
		  unsigned CMF = 120; /*0b01111000: CM 8, CINFO 7. With CINFO 7, any window size up to 32768 can be used.*/
		  unsigned FLEVEL = 0;
		  unsigned FDICT = 0;
		  unsigned CMFFLG = 256 * CMF + FDICT * 32 + FLEVEL * 64;
		  unsigned FCHECK = 31 - CMFFLG % 31;
		  CMFFLG += FCHECK;

		  /*ucvector-controlled version of the output buffer, for dynamic array*/

		  dvc_write(&ZlibData, (unsigned char)(CMFFLG >> 8));
		  dvc_write(&ZlibData, (unsigned char)(CMFFLG & 255));
		  DeflateData.data=ZlibData.data+ZlibData.size;
		  DeflateData.maxsize=ZlibData.maxsize-ZlibData.size;
		  DeflateData.size=0;

	//LZ77 data space allocation
	LZ77edData.maxsize=(blocksize*4/3+4)+1;//size in unsigned units
	LZ77edData.data= (unsigned*)(((size_t)ZlibData.data - (LZ77edData.maxsize*sizeof(unsigned)))&~0xf);

	if ((void*)LZ77edData.data<Mem_MinAddr) return ErrMemmoryOutOfBounds;
	//Huffman space allocations
	HuffmanTree tree_ll; /*tree for lit,len values*/
	HuffmanTree tree_d; /*tree for distance codes*/
	HuffmanTree tree_cl; /*tree for encoding the code lengths representing tree_ll and tree_d*/
	DV_unsigned freq_ll,freq_d,freq_cl;
	size_t numcodes_ll, numcodes_d;
	DV_unsigned bitlen_lld; /*lit,len,dist code lenghts (int bits), literally (without repeat codes).*/
	DV_unsigned bitlen_lld_e; /*bitlen_lld encoded with repeat codes (this is a rudemtary run length compression)*/
	/*bitlen_cl is the code length code lengths ("clcl"). The bit lengths of codes to represent tree_cl
	(these are written as is in the file, it would be crazy to compress these using yet another huffman
	tree that needs to be represented by yet another set of code lengths)*/
	DV_unsigned bitlen_cl;
		//mem alloc
	if(dvu_ini(&freq_ll, 286, LZ77edData.data-286*sizeof(unsigned))!=NoErr) return ErrDeflateAllocFailsN0;
	freq_ll.size=286;
	if(dvu_ini(&freq_d, 30,freq_ll.data-30*sizeof(unsigned))!=NoErr) return ErrDeflateAllocFailsN1;
	freq_d.size=30;
	if(dvu_ini(&freq_cl, NUM_CODE_LENGTH_CODES,freq_d.data-NUM_CODE_LENGTH_CODES*sizeof(unsigned))!=NoErr) return ErrDeflateAllocFailsN2;
	freq_cl.size=NUM_CODE_LENGTH_CODES;
	//Deflate definitions and settings
	  unsigned BFINAL;
	  unsigned HLIT, HDIST, HCLEN;
	  size_t bitpointer=0;
	  size_t *bp;bp=&bitpointer;
	  DV_char *deflateOut=&DeflateData;
	//calculate number of deflate blocks
	int numdeflateblocks = (FilteredData.size + blocksize - 1) / blocksize;
	if(numdeflateblocks == 0) numdeflateblocks = 1;
	//init hash
	HashInit();
	//Deflate for each block
	for(int di = 0; di != numdeflateblocks && error==NoErr; ++di){
		BFINAL = (di == numdeflateblocks - 1);//marks block as final if it is the last block
		//select filtered data to be encoded
		size_t start = di * blocksize;
		size_t end = start + blocksize;
		if(end > FilteredData.size) end = FilteredData.size;
		int tsize=(int) end - start;
		char *sdata=FilteredData.data+start;
		//use lz77
		LZ77edData.size=0;//reset lz77 buffer
#ifdef USE_SWLZ77
		error=encodeLZ77(start, end);
#endif
#ifdef NOLZ77
		size_t li;
		for(li = start ;li < end; ++li) LZ77edData.data[li - start] = (unsigned char)FilteredData.data[li];
		LZ77edData.data[li-start] =256;
#endif
#ifdef TEST_LZ77_OUTPUT
		error=LZ77Test(FilteredData.data+start,LZ77edData.data);
#endif
		//USe huffman
			//Calculate frequencies
		dvu_reset(&freq_ll);
		dvu_reset(&freq_d);
		dvu_reset(&freq_cl);
		size_t i=0;;
		do
		{
		  unsigned symbol = LZ77edData.data[i];
		  ++freq_ll.data[symbol];
		  if(symbol > 256)
		  {
			unsigned dist = LZ77edData.data[i + 2];
			++freq_d.data[dist];
			i += 3;
		  }
		}while(LZ77edData.data[++i] != 256);
#ifdef USE_DEBUG_PRINTF
		printf("LZ77edData Size %zd %zd\r\n",LZ77edData.size,i);
#endif
		LZ77edData.size=i;//doesn't count end flag as part of the data
		freq_ll.data[256] = 1; /*there will be exactly 1 end code, at the end of the block*/
			//make trees ll e d
		Mem_FreeAddr=freq_cl.data;//set freeaddr to be used in this cycle wihout consuming extra memory
		/*Make both huffman trees, one for the lit and len codes, one for the dist codes*/
		error = HuffmanTree_makeFromFrequencies(&tree_ll, freq_ll.data, 257, freq_ll.size, 15);
		if(error) return error;
		/*2, not 1, is chosen for mincodes: some buggy PNG decoders require at least 2 symbols in the dist tree*/
		error = HuffmanTree_makeFromFrequencies(&tree_d, freq_d.data, 2, freq_d.size, 15);
		if(error) return error;
		numcodes_ll = tree_ll.numcodes; if(numcodes_ll > 286) numcodes_ll = 286;
		numcodes_d = tree_d.numcodes; if(numcodes_d > 30) numcodes_d = 30;
		 /*store the code lengths of both generated trees in bitlen_lld*/
		dvu_ini(&bitlen_lld,(numcodes_ll+numcodes_d+2),Mem_FreeAddr-((numcodes_ll+numcodes_d+2)*sizeof(unsigned)));
		dvu_ini(&bitlen_lld_e,bitlen_lld.maxsize,bitlen_lld.data-(bitlen_lld.maxsize*sizeof(unsigned)));
		if((void*)bitlen_lld_e.data<Mem_MinAddr) return ErrDeflateAllocFailsN3;
		Mem_FreeAddr=(void*)bitlen_lld_e.data;
		for(i = 0; i != numcodes_ll; ++i) dvu_write(&bitlen_lld, HuffmanTree_getLength(&tree_ll, (unsigned)i));
		for(i = 0; i != numcodes_d; ++i) dvu_write(&bitlen_lld, HuffmanTree_getLength(&tree_d, (unsigned)i));
			//RLE Trees
		for(i = 0; i != (unsigned)bitlen_lld.size; ++i)	{
		  unsigned j = 0; /*amount of repititions*/
		  while(i + j + 1 < (unsigned)bitlen_lld.size && bitlen_lld.data[i + j + 1] == bitlen_lld.data[i]) ++j;

		  if(bitlen_lld.data[i] == 0 && j >= 2) /*repeat code for zeroes*/
		  {
			++j; /*include the first zero*/
			if(j <= 10) /*repeat code 17 supports max 10 zeroes*/
			{
			  dvu_write(&bitlen_lld_e, 17);
			  dvu_write(&bitlen_lld_e, j - 3);
			}
			else /*repeat code 18 supports max 138 zeroes*/
			{
			  if(j > 138) j = 138;
			  dvu_write(&bitlen_lld_e, 18);
			  dvu_write(&bitlen_lld_e, j - 11);
			}
			i += (j - 1);
		  }
		  else if(j >= 3) /*repeat code for value other than zero*/
		  {
			size_t k;
			unsigned num = j / 6, rest = j % 6;
			dvu_write(&bitlen_lld_e, bitlen_lld.data[i]);
			for(k = 0; k < num; ++k)
			{
			  dvu_write(&bitlen_lld_e, 16);
			  dvu_write(&bitlen_lld_e, 6 - 3);
			}
			if(rest >= 3)
			{
			  dvu_write(&bitlen_lld_e, 16);
			  dvu_write(&bitlen_lld_e, rest - 3);
			}
			else j -= rest;
			i += j;
		  }
		  else /*too short to benefit from repeat code*/
		  {
			dvu_write(&bitlen_lld_e, bitlen_lld.data[i]);
		  }
		}
			//make tree clcl
		 /*generate tree_cl, the huffmantree of huffmantrees*/

		for(i = 0; i != bitlen_lld_e.size; ++i)
		{
		  ++freq_cl.data[bitlen_lld_e.data[i]];
		  /*after a repeat code come the bits that specify the number of repetitions,
		  those don't need to be in the frequencies_cl calculation*/
		  if(bitlen_lld_e.data[i] >= 16) ++i;
		}

		error = HuffmanTree_makeFromFrequencies(&tree_cl, freq_cl.data,
												freq_cl.size, freq_cl.size, 7);
		if(error) return error;

		if(dvu_ini(&bitlen_cl, tree_cl.numcodes,Mem_FreeAddr-tree_cl.numcodes*sizeof(unsigned))) return ErrDeflateAllocFailsN4;
		dvu_reset(&bitlen_cl);bitlen_cl.size=bitlen_cl.maxsize;//clean data values
		for(i = 0; i != tree_cl.numcodes; ++i)
		{
		  /*lenghts of code length tree is in the order as specified by deflate*/
		  bitlen_cl.data[i] = HuffmanTree_getLength(&tree_cl, CLCL_ORDER[i]);
		}
	    while(bitlen_cl.data[bitlen_cl.size - 1] == 0 && bitlen_cl.size > 4)
		{
		  /*remove zeros at the end, but minimum size must be 4*/
		  bitlen_cl.size-- ;
		  bitlen_cl.maxsize-- ;
		}
		if(error) return ErrDeflateCompressFail;
#ifdef USE_DEBUG_PRINTF
		PrintHuffmanCodes(&tree_ll);
		PrintHuffmanCodes(&tree_d);
		PrintHuffmanCodes(&tree_cl);
#endif
			//write data
		 /*
		    Write everything into the output

		    After the BFINAL and BTYPE, the dynamic block consists out of the following:
		    - 5 bits HLIT, 5 bits HDIST, 4 bits HCLEN
		    - (HCLEN+4)*3 bits code lengths of code length alphabet
		    - HLIT + 257 code lenghts of lit/length alphabet (encoded using the code length
		      alphabet, + possible repetition codes 16, 17, 18)
		    - HDIST + 1 code lengths of distance alphabet (encoded using the code length
		      alphabet, + possible repetition codes 16, 17, 18)
		    - compressed data
		    - 256 (end code)
		    */
		    /*Write block type*/
		    addBitToStream(bp, deflateOut, BFINAL);
		    addBitToStream(bp, deflateOut, 0); /*first bit of BTYPE "dynamic"*/
		    addBitToStream(bp, deflateOut, 1); /*second bit of BTYPE "dynamic"*/
		    /*write the HLIT, HDIST and HCLEN values*/
		    HLIT = (unsigned)(numcodes_ll - 257);
		    HDIST = (unsigned)(numcodes_d - 1);
		    HCLEN = (unsigned)bitlen_cl.size - 4;
		    /*trim zeroes for HCLEN. HLIT and HDIST were already trimmed at tree creation*/
		    while(!bitlen_cl.data[HCLEN + 4 - 1] && HCLEN > 0) --HCLEN;
		    addBitsToStream(bp, deflateOut, HLIT, 5);
		    addBitsToStream(bp, deflateOut, HDIST, 5);
		    addBitsToStream(bp, deflateOut, HCLEN, 4);
		    /*write the code lenghts of the code length alphabet*/
		    for(i = 0; i != HCLEN + 4; ++i) addBitsToStream(bp, deflateOut, bitlen_cl.data[i], 3);

		    /*write the lenghts of the lit/len AND the dist alphabet*/
		    for(i = 0; i != bitlen_lld_e.size; ++i)
		    {
		      addHuffmanSymbol(bp, deflateOut, HuffmanTree_getCode(&tree_cl, bitlen_lld_e.data[i]),
		                       HuffmanTree_getLength(&tree_cl, bitlen_lld_e.data[i]));
		      /*extra bits of repeat codes*/
		      if(bitlen_lld_e.data[i] == 16) addBitsToStream(bp, deflateOut, bitlen_lld_e.data[++i], 2);
		      else if(bitlen_lld_e.data[i] == 17) addBitsToStream(bp, deflateOut, bitlen_lld_e.data[++i], 3);
		      else if(bitlen_lld_e.data[i] == 18) addBitsToStream(bp, deflateOut, bitlen_lld_e.data[++i], 7);
		    }
		    /*write the compressed data symbols*/
		    writeLZ77data(bp, deflateOut, &LZ77edData, &tree_ll, &tree_d);
		    /*error: the length of the end code 256 must be larger than 0*/
		    if(HuffmanTree_getLength(&tree_ll, 256) == 0) return ErrHuff256SymbolInvVal;

		    /*write the end code*/
		    addHuffmanSymbol(bp, deflateOut, HuffmanTree_getCode(&tree_ll, 256), HuffmanTree_getLength(&tree_ll, 256));
	}
	//Zlib ender
	ZlibData.size+=DeflateData.size;
	unsigned ADLER32 = adler32(1L, FilteredData.data,FilteredData.size);
	dvc_add32bitInt(&ZlibData, ADLER32);
	//set free memory space bellow zlib data. zlib data contains the image data for the png file
	Mem_FreeAddr=(void*)ZlibData.data-ZlibData.maxsize;
#ifdef USE_DEBUG_PRINTF
    printf("LZivData - addr:%p size:%zd\r\n",ZlibData.data,ZlibData.size);
    printf("DeflateData - addr:%p size:%zd\r\n",DeflateData.data,DeflateData.size);
#endif
	return error;
}

/*******************************************************************/
/** PNG File Format                                               **/
/*******************************************************************/
static unsigned lodepng_crc32_table[256] = {
           0u, 1996959894u, 3993919788u, 2567524794u,  124634137u, 1886057615u, 3915621685u, 2657392035u,
   249268274u, 2044508324u, 3772115230u, 2547177864u,  162941995u, 2125561021u, 3887607047u, 2428444049u,
   498536548u, 1789927666u, 4089016648u, 2227061214u,  450548861u, 1843258603u, 4107580753u, 2211677639u,
   325883990u, 1684777152u, 4251122042u, 2321926636u,  335633487u, 1661365465u, 4195302755u, 2366115317u,
   997073096u, 1281953886u, 3579855332u, 2724688242u, 1006888145u, 1258607687u, 3524101629u, 2768942443u,
   901097722u, 1119000684u, 3686517206u, 2898065728u,  853044451u, 1172266101u, 3705015759u, 2882616665u,
   651767980u, 1373503546u, 3369554304u, 3218104598u,  565507253u, 1454621731u, 3485111705u, 3099436303u,
   671266974u, 1594198024u, 3322730930u, 2970347812u,  795835527u, 1483230225u, 3244367275u, 3060149565u,
  1994146192u,   31158534u, 2563907772u, 4023717930u, 1907459465u,  112637215u, 2680153253u, 3904427059u,
  2013776290u,  251722036u, 2517215374u, 3775830040u, 2137656763u,  141376813u, 2439277719u, 3865271297u,
  1802195444u,  476864866u, 2238001368u, 4066508878u, 1812370925u,  453092731u, 2181625025u, 4111451223u,
  1706088902u,  314042704u, 2344532202u, 4240017532u, 1658658271u,  366619977u, 2362670323u, 4224994405u,
  1303535960u,  984961486u, 2747007092u, 3569037538u, 1256170817u, 1037604311u, 2765210733u, 3554079995u,
  1131014506u,  879679996u, 2909243462u, 3663771856u, 1141124467u,  855842277u, 2852801631u, 3708648649u,
  1342533948u,  654459306u, 3188396048u, 3373015174u, 1466479909u,  544179635u, 3110523913u, 3462522015u,
  1591671054u,  702138776u, 2966460450u, 3352799412u, 1504918807u,  783551873u, 3082640443u, 3233442989u,
  3988292384u, 2596254646u,   62317068u, 1957810842u, 3939845945u, 2647816111u,   81470997u, 1943803523u,
  3814918930u, 2489596804u,  225274430u, 2053790376u, 3826175755u, 2466906013u,  167816743u, 2097651377u,
  4027552580u, 2265490386u,  503444072u, 1762050814u, 4150417245u, 2154129355u,  426522225u, 1852507879u,
  4275313526u, 2312317920u,  282753626u, 1742555852u, 4189708143u, 2394877945u,  397917763u, 1622183637u,
  3604390888u, 2714866558u,  953729732u, 1340076626u, 3518719985u, 2797360999u, 1068828381u, 1219638859u,
  3624741850u, 2936675148u,  906185462u, 1090812512u, 3747672003u, 2825379669u,  829329135u, 1181335161u,
  3412177804u, 3160834842u,  628085408u, 1382605366u, 3423369109u, 3138078467u,  570562233u, 1426400815u,
  3317316542u, 2998733608u,  733239954u, 1555261956u, 3268935591u, 3050360625u,  752459403u, 1541320221u,
  2607071920u, 3965973030u, 1969922972u,   40735498u, 2617837225u, 3943577151u, 1913087877u,   83908371u,
  2512341634u, 3803740692u, 2075208622u,  213261112u, 2463272603u, 3855990285u, 2094854071u,  198958881u,
  2262029012u, 4057260610u, 1759359992u,  534414190u, 2176718541u, 4139329115u, 1873836001u,  414664567u,
  2282248934u, 4279200368u, 1711684554u,  285281116u, 2405801727u, 4167216745u, 1634467795u,  376229701u,
  2685067896u, 3608007406u, 1308918612u,  956543938u, 2808555105u, 3495958263u, 1231636301u, 1047427035u,
  2932959818u, 3654703836u, 1088359270u,  936918000u, 2847714899u, 3736837829u, 1202900863u,  817233897u,
  3183342108u, 3401237130u, 1404277552u,  615818150u, 3134207493u, 3453421203u, 1423857449u,  601450431u,
  3009837614u, 3294710456u, 1567103746u,  711928724u, 3020668471u, 3272380065u, 1510334235u,  755167117u
};
unsigned lodepng_crc32(const unsigned char* data, size_t length)
{
  unsigned r = 0xffffffffu;
  size_t i;
  for(i = 0; i < length; ++i)
  {
    r = lodepng_crc32_table[(r ^ data[i]) & 0xff] ^ (r >> 8);
  }
  return r ^ 0xffffffffu;
}
void lodepng_chunk_generate_crc(unsigned char* chunk,unsigned length){
   write32bit( (char*)chunk + 8 + length, lodepng_crc32(&chunk[4], length + 4));
}
unsigned PNGchunk_add(DV_char *dv, const char* type, unsigned char* data,unsigned length)
{
  unsigned i;
  unsigned char *chunk;
  unsigned *chunklen;
  chunk = (unsigned char*) &(dv->data[dv->size]);
  chunklen=(unsigned*)chunk;
  size_t new_length = dv->size + length + 12;
  if(new_length < length + 12 || new_length < dv->size) return 77; /*integer overflow happened*/


  /*1: length*/
  write32bit(chunk,(unsigned)length);

  /*2: chunk name (4 letters)*/
  chunk[4] = (unsigned char)type[0];
  chunk[5] = (unsigned char)type[1];
  chunk[6] = (unsigned char)type[2];
  chunk[7] = (unsigned char)type[3];

  /*3: the data*/
  for(i = 0; i != length; ++i) chunk[8 + i] = data[i];
  /*4: CRC (of the chunkname characters and the data)*/
  lodepng_chunk_generate_crc(chunk,length);
  /*setcorrect size*/
  dv->size=new_length;

  return 0;
}


static void writeSignature()
{
  /*8 bytes PNG signature, aka the magic bytes*/
	char signature[]={137, 80, 78, 71, 13, 10, 26, 10};
	dvc_writeS(&PNGchar,signature,8);
	return;
}
static unsigned addChunk_IHDR()
{
  unsigned error = 0;
  DV_char header;
  if(dvc_ini(&header,13,Mem_FreeAddr-13)) return ErrPngCreation;

  dvc_add32bitInt(&header, imginfo.width); /*width*/
  dvc_add32bitInt(&header, imginfo.heigth); /*height*/
  dvc_write(&header, (unsigned char)imginfo.bitdepth); /*bit depth*/
  dvc_write(&header, (unsigned char)imginfo.colortype); /*color type*/
  dvc_write(&header, 0); /*compression method*/
  dvc_write(&header, 0); /*filter method*/
  dvc_write(&header, imginfo.interlace_method); /*interlace method*/

  error = PNGchunk_add(&PNGchar, "IHDR", header.data, header.size);

  return error;
}


static unsigned addChunk_IDAT()
{
  /*compress with the Zlib compressor*/
  if(!error) error = PNGchunk_add(&PNGchar, "IDAT", ZlibData.data, ZlibData.size);
  return error;
}

static unsigned addChunk_IEND()
{
  unsigned error = 0;
  error = PNGchunk_add(&PNGchar, "IEND", 0, 0);
  return error;
}
/*******************************************************************/
/** Main Encoder Function                                         **/
/*******************************************************************/

int creatpng(){
	if(error){
			return error;
		}
	/*lets construct png file over filtered data
	 * filtered data is no longer useful
	 * resuzing avaible space for small memory requirements*/
	PNGchar.data=FilteredData.data;
	PNGchar.maxsize=FilteredData.maxsize;
	PNGchar.size=0;
	writeSignature(&PNGchar);
	addChunk_IHDR();
	addChunk_IDAT();
	addChunk_IEND();
}

int AzamiPNG_encode(){
	error=ErrOk;//set error flag to zero
	error=filter();
	error=deflate();
	error=creatpng();

	return error;
}

/*******************************************************************/
/** Examples compilation                                          **/
/*******************************************************************/
#ifdef EXAMPLE_RANDOM
	int main(){
		int w,h;
		w=500;
		h=500;
		set_baseencoder();
		if(set_memforimg(h,w)!=NoErr) return  ErrMallocFailure;
		char *img=(char*)(char*)(((size_t)Mem_MaxAddr - (h*w*3))&~0xf);
		set_inimg(img,w,h,24);
		if(imginfo.data==NULL) return ErrMallocFailure;
		printf("Head from %p to %p\r\n",Mem_MinAddr,Mem_MaxAddr);
			int x=0,y=0;
			char r=1,g=2,b=4;
			do{
			do{
				if(r==0)r=1;
				if(g==0)g=1;
				if(b==0)b=1;

				int i;
				for (i=x*3;i<w*3;i+=3) {
					img[y*w*3+i]=r;
					img[y*w*3+i+1]=g;
					img[y*w*3+i+2]=b;
				}
				for (i=y;i<h;i+=1) {
					img[(i*w+x)*3]=r;
					img[(i*w+x)*3+1]=g;
					img[(i*w+x)*3+2]=b;
				}
				r=r<<1;
				g=g<<1;
				b=b<<1;
				y+=(y<h)? 1 : 0;
				x+=(x<w)? 1 : 0;

			}while(x<w);
			}while(y<h);
		printf("ImageCreat %d %d\r\n",w,h);
		AzamiPNG_encode();
		printf("Err:%d\r\n",error);
		writefilefc(&PNGchar,"azami.png");
		free(Mem_MinAddr);
		return 0;
	}
#endif
