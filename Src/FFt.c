#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define FFT_SIZE 4096        // FFT points (must be power of 2)
#define SAMPLE_RATE 21875
#define NEW 8192 
float real[NEW];           // Real part of FFT input
float imag[NEW];           // Imaginary part (initialized to zero)
float magnitude[NEW / 2];  // Magnitude of FFT output


void FFT(float real[], float imag[], uint16_t n);

// Cooley-Tukey FFT implementation
void FFT(float real[], float imag[], uint16_t n) {
  uint16_t i, j, k, m;
  uint16_t len, half;
  float cosVal, sinVal, tempReal, tempImag;
  float pi = 3.14159265358979323846;

  // Bit-reversal permutation
  j = 0;
  for (i = 1; i < n; i++) {
    m = n >> 1;
    while (j >= m && m > 0) {
      j -= m;
      m >>= 1;
    }
    j += m;
    if (i < j) {
      tempReal = real[i];
      real[i] = real[j];
      real[j] = tempReal;

      tempImag = imag[i];
      imag[i] = imag[j];
      imag[j] = tempImag;
    }
  }

  // Cooley-Tukey FFT computation
  for (len = 2; len <= n; len <<= 1) {
    half = len >> 1;
    for (i = 0; i < n; i += len) {
      for (j = 0; j < half; j++) {
        k = (j * n) / len;
        cosVal = cosf(-2.0f * pi * k / n);
        sinVal = sinf(-2.0f * pi * k / n);

        tempReal = cosVal * real[i + j + half] - sinVal * imag[i + j + half];
        tempImag = cosVal * imag[i + j + half] + sinVal * real[i + j + half];

        real[i + j + half] = real[i + j] - tempReal;
        imag[i + j + half] = imag[i + j] - tempImag;

        real[i + j] += tempReal;
        imag[i + j] += tempImag;
      }
    }
  }
}
