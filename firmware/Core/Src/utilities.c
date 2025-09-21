#include "utilities.h"

int8_t safe_mod(int8_t value, int8_t mod)
{
  return (value % mod + mod) % mod;
}

