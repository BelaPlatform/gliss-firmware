#include "stringId.h"
char stringId[256] __attribute__((section(".stringIdSec"), used)) = "Gliss";
