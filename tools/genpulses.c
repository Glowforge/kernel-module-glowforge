/**
 * genpulses.c
 *
 * Command-line tool to generate test pulse data.
 *
 * usage: genpulses <chunkwidth> <repeats> <chunks>
 * - chunkwidth is an integer specifying how many bytes wide each
 *   "chunk" should be
 * - repeats is an integer specifying the number of repetitions of the data
 * - chunks is a string where each character specifies the bits that should
 *   be set for each chunkwidth
 *
 * Currently, the supported character set for chunks is 0123456789abcdef,
 * where each character is a 4-bit 
 *
 * Examples:
 * genpulses 1024 1 0f
 *   generates a 2048-byte file:
 *     1024 steps with all bits off
 *     1024 steps with all bits on
 * genpulses 2048 16 1248
 *   generates a 32768-byte file:
 *     16 repetitions of each of the 4 bits turning on for 2048 steps
 *
 * To compile:
 *   gcc -o genpulses genpulses.c
 */

#include <err.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t bit_to_pinmask[8] = {
  0b00000001,
  0b00010000,
  0b00100000,
  0b01000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
};

static uint8_t output_byte_for_char(char c)
{
  uint8_t bits = 0;
  uint8_t output_byte = 0;
  int i;

  if (c >= '0' && c <= '9') {
    bits = c - '0';
  } else if (c >= 'a' && c <= 'f') {
    bits = 0xa + c - 'a';
  } else {
    warnx("pattern '%c' not supported, using '0'", c);
  }

  for (i = 0; i < 7; i++) {
    output_byte |= (bits & (1 << i)) ? bit_to_pinmask[i] : 0;
  }

  return output_byte;
}


int main(int argc, char **argv)
{
  int r;
  int chunkwidth;
  int repeats;
  char *chunk;
  char *ptr;

  if (argc != 4) {
    printf("usage: genpulses <chunkwidth> <repeats> <chunks>\n");
    return 1;
  }

  chunkwidth = atoi(argv[1]);
  if (chunkwidth <= 0) {
    errx(1, "chunkwidth must be greater than 0");
  }

  repeats = atoi(argv[2]);
  if (repeats <= 0) {
    errx(1, "repeats must be greater than 0");
  }

  chunk = malloc(chunkwidth);
  for (r = 0; r < repeats; r++) {
    for (ptr = argv[3]; *ptr; ptr++) {
      char output_byte = output_byte_for_char(*ptr);
      memset(chunk, output_byte, chunkwidth);
      fwrite(chunk, 1, chunkwidth, stdout);
    }
  }
  free(chunk);
  return 0;
}