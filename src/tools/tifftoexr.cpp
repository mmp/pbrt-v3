/*
 g++ -g -Wall tifftoexr.cpp -I../../src/tangled_code/OpenEXR/include -ltiff -L../../src/tangled_code/OpenEXR/lib-linux -lIlmImf -lImath -lIex -lHalf  

*/

#include <stdio.h>
#include <stdlib.h>
#include <tiffio.h>
#include <ImfOutputFile.h>
#include <ImfChannelList.h>
#include <ImfFrameBuffer.h>
#include <half.h>

using namespace Imf;
using namespace Imath;

#define INV_255 .00392156862745098039f

static void WriteEXR(const char *name, float *rgba, int xRes, int yRes, bool hasAlpha);
static bool ReadTIFF(const char *name, float *&rgba, int &xRes, int &yRes, bool &hasAlpha);

int main(int argc, char *argv[]) 
{
    if (argc != 3) {
	fprintf(stderr, "usage: tifftoexr [foo.tiff] [foo.exr]\n");
	return 1;
    }

    float *rgba;
    int xRes, yRes;
    bool hasAlpha;
    if (ReadTIFF(argv[1], rgba, xRes, yRes, hasAlpha))
	WriteEXR(argv[2], rgba, xRes, yRes, hasAlpha);
    return 0;
}

void WriteEXR(const char *name, float *frgba, int xRes, int yRes, bool hasAlpha) 
{
    Header header(xRes, yRes);
    header.channels().insert("R", Channel (HALF));
    header.channels().insert("G", Channel (HALF));
    header.channels().insert("B", Channel (HALF));
    if (hasAlpha)
	header.channels().insert("A", Channel (HALF));
    int stride = hasAlpha ? 4 : 3;

    half *rgba = new half[xRes*yRes * stride];
    for (int i = 0; i < xRes*yRes * stride; ++i)
	rgba[i] = frgba[i];

    FrameBuffer fb;
    fb.insert("R", Slice(HALF, (char *)rgba, stride*sizeof(half),
			 stride*xRes*sizeof(half)));
    fb.insert("G", Slice(HALF, (char *)rgba+sizeof(half), stride*sizeof(half),
			 stride*xRes*sizeof(half)));
    fb.insert("B", Slice(HALF, (char *)rgba+2*sizeof(half), stride*sizeof(half),
			 stride*xRes*sizeof(half)));
    if (hasAlpha)
	fb.insert("A", Slice(HALF, (char *)rgba+3*sizeof(half), stride*sizeof(half),
			     stride*xRes*sizeof(half)));

    OutputFile file(name, header);
    file.setFrameBuffer(fb);
    file.writePixels(yRes);
}

static bool ReadTIFF(const char *name, float *&rgba, int &xRes, int &yRes,
		     bool &hasAlpha)
{
    // Try to open TIFF file
    TIFF *tiff = TIFFOpen(name, "r");
    if (!tiff) {
	fprintf(stderr, "Unable to open TIFF %s", name);
	return false;
    }
    // Get basic information from TIFF header
    short int nSamples;
    int xSize, ySize;
    TIFFGetField(tiff, TIFFTAG_IMAGEWIDTH, &xSize);
    TIFFGetField(tiff, TIFFTAG_IMAGELENGTH, &ySize);
    TIFFGetField(tiff, TIFFTAG_SAMPLESPERPIXEL, &nSamples);
    if (nSamples != 3 && nSamples != 4) {
	fprintf(stderr, "Sorry, only handle 3 and 4 sample TIFFs...\n");
	return false;
    }
    hasAlpha = (nSamples == 4);
    xRes = xSize;
    yRes = ySize;

    // Make sure this is a TIFF we can read
    short int bitsPerSample, sampleFormat = SAMPLEFORMAT_UINT;
    if (!TIFFGetField(tiff, TIFFTAG_BITSPERSAMPLE, &bitsPerSample)) {
	fprintf(stderr, "TIFFRead: bits per sample not set in TIFF");
	TIFFClose(tiff);
	return false;
    }

    if (!TIFFGetField(tiff, TIFFTAG_SAMPLEFORMAT, &sampleFormat)) {
	if (bitsPerSample == 32)
	    sampleFormat = SAMPLEFORMAT_IEEEFP;
	else
	    sampleFormat = SAMPLEFORMAT_UINT;
    }
	
    if (bitsPerSample == 32) {
	if (sampleFormat != SAMPLEFORMAT_IEEEFP) {
	    fprintf(stderr, "TIFFRead: 32 bit TIFF not stored in floating point format");
	    TIFFClose(tiff);
	    return false;
	}
    }
    else {
	if (bitsPerSample != 8 && bitsPerSample != 32) {
	    fprintf(stderr, "TIFFRead: only 8 and 32 bits per sample supported");
	    TIFFClose(tiff);
	    return false;
	}
	if (sampleFormat != SAMPLEFORMAT_UINT) {
	    fprintf(stderr, "TIFFRead: 8 bit TIFFs must be stored as unsigned ints");
	    TIFFClose(tiff);
	    return false;
	}
    }

    int bytesPerSample = bitsPerSample / 8;
    if (nSamples * xRes * bytesPerSample != TIFFScanlineSize(tiff)) {
	fprintf(stderr, "TIFFRead: RGB not interleaved in TIFF %s", name);
	TIFFClose(tiff);
	return false;
    }

    // Allocate space for [[pixels]] and buffers
    rgba = new float[nSamples * xRes * yRes];
    float *p = rgba;
    unsigned char *ubuf = NULL;
    if (bitsPerSample == 8) ubuf = new unsigned char[nSamples * xRes];

    for (int y = 0; y < yRes; ++y) {
	if (ubuf) {
	    // Read 8-bit TIFF scanline
	    if (TIFFReadScanline(tiff, ubuf, y, 1) == -1) {
		TIFFClose(tiff);
		return false;
	    }
	    for (int x = 0; x < nSamples*xRes; ++x)
		*p++ = ubuf[x] * INV_255;
	}
	else {
	    // Read floating point TIFF scanline
	    if (TIFFReadScanline(tiff, p, y, 1) == -1) {
		TIFFClose(tiff);
		return false;
	    }
	    p += nSamples * xRes;
	}
    }

    delete[] ubuf;
    TIFFClose(tiff);
    return rgba;
}

